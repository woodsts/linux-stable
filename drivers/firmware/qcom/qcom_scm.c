// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2010,2015,2019 The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 Linaro Ltd.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/cpumask.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/sizes.h>
#include <linux/types.h>

#include "qcom_scm.h"
#include "qcom_tzmem.h"

static u32 download_mode;

struct qcom_scm {
	struct device *dev;
	struct clk *core_clk;
	struct clk *iface_clk;
	struct clk *bus_clk;
	struct icc_path *path;
	struct completion waitq_comp;
	struct reset_controller_dev reset;

	/* control access to the interconnect path */
	struct mutex scm_bw_lock;
	int scm_vote_count;

	u64 dload_mode_addr;

	struct qcom_tzmem_pool *mempool;
};

struct qcom_scm_current_perm_info {
	__le32 vmid;
	__le32 perm;
	__le64 ctx;
	__le32 ctx_size;
	__le32 unused;
};

struct qcom_scm_mem_map_info {
	__le64 mem_addr;
	__le64 mem_size;
};

/**
 * struct qcom_scm_qseecom_resp - QSEECOM SCM call response.
 * @result:    Result or status of the SCM call. See &enum qcom_scm_qseecom_result.
 * @resp_type: Type of the response. See &enum qcom_scm_qseecom_resp_type.
 * @data:      Response data. The type of this data is given in @resp_type.
 */
struct qcom_scm_qseecom_resp {
	u64 result;
	u64 resp_type;
	u64 data;
};

enum qcom_scm_qseecom_result {
	QSEECOM_RESULT_SUCCESS			= 0,
	QSEECOM_RESULT_INCOMPLETE		= 1,
	QSEECOM_RESULT_BLOCKED_ON_LISTENER	= 2,
	QSEECOM_RESULT_FAILURE			= 0xFFFFFFFF,
};

enum qcom_scm_qseecom_resp_type {
	QSEECOM_SCM_RES_APP_ID			= 0xEE01,
	QSEECOM_SCM_RES_QSEOS_LISTENER_ID	= 0xEE02,
};

enum qcom_scm_qseecom_tz_owner {
	QSEECOM_TZ_OWNER_SIP			= 2,
	QSEECOM_TZ_OWNER_TZ_APPS		= 48,
	QSEECOM_TZ_OWNER_QSEE_OS		= 50
};

enum qcom_scm_qseecom_tz_svc {
	QSEECOM_TZ_SVC_APP_ID_PLACEHOLDER	= 0,
	QSEECOM_TZ_SVC_APP_MGR			= 1,
	QSEECOM_TZ_SVC_INFO			= 6,
};

enum qcom_scm_qseecom_tz_cmd_app {
	QSEECOM_TZ_CMD_APP_SEND			= 1,
	QSEECOM_TZ_CMD_APP_LOOKUP		= 3,
};

enum qcom_scm_qseecom_tz_cmd_info {
	QSEECOM_TZ_CMD_INFO_VERSION		= 3,
};

#define QSEECOM_MAX_APP_NAME_SIZE		64
#define SHMBRIDGE_RESULT_NOTSUPP		4

/* Each bit configures cold/warm boot address for one of the 4 CPUs */
static const u8 qcom_scm_cpu_cold_bits[QCOM_SCM_BOOT_MAX_CPUS] = {
	0, BIT(0), BIT(3), BIT(5)
};
static const u8 qcom_scm_cpu_warm_bits[QCOM_SCM_BOOT_MAX_CPUS] = {
	BIT(2), BIT(1), BIT(4), BIT(6)
};

#define QCOM_SMC_WAITQ_FLAG_WAKE_ONE	BIT(0)

#define QCOM_DLOAD_MASK		GENMASK(5, 4)
#define QCOM_DLOAD_NODUMP	0
#define QCOM_DLOAD_FULLDUMP	1
#define QCOM_DLOAD_MINIDUMP	2
#define QCOM_DLOAD_BOTHDUMP	3

static const char * const qcom_scm_convention_names[] = {
	[SMC_CONVENTION_UNKNOWN] = "unknown",
	[SMC_CONVENTION_ARM_32] = "smc arm 32",
	[SMC_CONVENTION_ARM_64] = "smc arm 64",
	[SMC_CONVENTION_LEGACY] = "smc legacy",
};

static const char * const download_mode_name[] = {
	[QCOM_DLOAD_NODUMP]	= "off",
	[QCOM_DLOAD_FULLDUMP]	= "full",
	[QCOM_DLOAD_MINIDUMP]	= "mini",
	[QCOM_DLOAD_BOTHDUMP]	= "full,mini",
};

static struct qcom_scm *__scm;

static int qcom_scm_clk_enable(void)
{
	int ret;

	ret = clk_prepare_enable(__scm->core_clk);
	if (ret)
		goto bail;

	ret = clk_prepare_enable(__scm->iface_clk);
	if (ret)
		goto disable_core;

	ret = clk_prepare_enable(__scm->bus_clk);
	if (ret)
		goto disable_iface;

	return 0;

disable_iface:
	clk_disable_unprepare(__scm->iface_clk);
disable_core:
	clk_disable_unprepare(__scm->core_clk);
bail:
	return ret;
}

static void qcom_scm_clk_disable(void)
{
	clk_disable_unprepare(__scm->core_clk);
	clk_disable_unprepare(__scm->iface_clk);
	clk_disable_unprepare(__scm->bus_clk);
}

static int qcom_scm_bw_enable(void)
{
	int ret = 0;

	if (!__scm->path)
		return 0;

	mutex_lock(&__scm->scm_bw_lock);
	if (!__scm->scm_vote_count) {
		ret = icc_set_bw(__scm->path, 0, UINT_MAX);
		if (ret < 0) {
			dev_err(__scm->dev, "failed to set bandwidth request\n");
			goto err_bw;
		}
	}
	__scm->scm_vote_count++;
err_bw:
	mutex_unlock(&__scm->scm_bw_lock);

	return ret;
}

static void qcom_scm_bw_disable(void)
{
	if (!__scm->path)
		return;

	mutex_lock(&__scm->scm_bw_lock);
	if (__scm->scm_vote_count-- == 1)
		icc_set_bw(__scm->path, 0, 0);
	mutex_unlock(&__scm->scm_bw_lock);
}

enum qcom_scm_convention qcom_scm_convention = SMC_CONVENTION_UNKNOWN;
static DEFINE_SPINLOCK(scm_query_lock);

struct qcom_tzmem_pool *qcom_scm_get_tzmem_pool(void)
{
	if (!qcom_scm_is_available())
		return NULL;

	return __scm->mempool;
}

static enum qcom_scm_convention __get_convention(void)
{
	unsigned long flags;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_INFO,
		.cmd = QCOM_SCM_INFO_IS_CALL_AVAIL,
		.args[0] = SCM_SMC_FNID(QCOM_SCM_SVC_INFO,
					   QCOM_SCM_INFO_IS_CALL_AVAIL) |
			   (ARM_SMCCC_OWNER_SIP << ARM_SMCCC_OWNER_SHIFT),
		.arginfo = QCOM_SCM_ARGS(1),
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	enum qcom_scm_convention probed_convention;
	int ret;
	bool forced = false;

	if (likely(qcom_scm_convention != SMC_CONVENTION_UNKNOWN))
		return qcom_scm_convention;

	/*
	 * Per the "SMC calling convention specification", the 64-bit calling
	 * convention can only be used when the client is 64-bit, otherwise
	 * system will encounter the undefined behaviour.
	 */
#if IS_ENABLED(CONFIG_ARM64)
	/*
	 * Device isn't required as there is only one argument - no device
	 * needed to dma_map_single to secure world
	 */
	probed_convention = SMC_CONVENTION_ARM_64;
	ret = __scm_smc_call(NULL, &desc, probed_convention, &res, true);
	if (!ret && res.result[0] == 1)
		goto found;

	/*
	 * Some SC7180 firmwares didn't implement the
	 * QCOM_SCM_INFO_IS_CALL_AVAIL call, so we fallback to forcing ARM_64
	 * calling conventions on these firmwares. Luckily we don't make any
	 * early calls into the firmware on these SoCs so the device pointer
	 * will be valid here to check if the compatible matches.
	 */
	if (of_device_is_compatible(__scm ? __scm->dev->of_node : NULL, "qcom,scm-sc7180")) {
		forced = true;
		goto found;
	}
#endif

	probed_convention = SMC_CONVENTION_ARM_32;
	ret = __scm_smc_call(NULL, &desc, probed_convention, &res, true);
	if (!ret && res.result[0] == 1)
		goto found;

	probed_convention = SMC_CONVENTION_LEGACY;
found:
	spin_lock_irqsave(&scm_query_lock, flags);
	if (probed_convention != qcom_scm_convention) {
		qcom_scm_convention = probed_convention;
		pr_info("qcom_scm: convention: %s%s\n",
			qcom_scm_convention_names[qcom_scm_convention],
			forced ? " (forced)" : "");
	}
	spin_unlock_irqrestore(&scm_query_lock, flags);

	return qcom_scm_convention;
}

/**
 * qcom_scm_call() - Invoke a syscall in the secure world
 * @dev:	device
 * @desc:	Descriptor structure containing arguments and return values
 * @res:        Structure containing results from SMC/HVC call
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 * This should *only* be called in pre-emptible context.
 */
static int qcom_scm_call(struct device *dev, const struct qcom_scm_desc *desc,
			 struct qcom_scm_res *res)
{
	might_sleep();
	switch (__get_convention()) {
	case SMC_CONVENTION_ARM_32:
	case SMC_CONVENTION_ARM_64:
		return scm_smc_call(dev, desc, res, false);
	case SMC_CONVENTION_LEGACY:
		return scm_legacy_call(dev, desc, res);
	default:
		pr_err("Unknown current SCM calling convention.\n");
		return -EINVAL;
	}
}

/**
 * qcom_scm_call_atomic() - atomic variation of qcom_scm_call()
 * @dev:	device
 * @desc:	Descriptor structure containing arguments and return values
 * @res:	Structure containing results from SMC/HVC call
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 * This can be called in atomic context.
 */
static int qcom_scm_call_atomic(struct device *dev,
				const struct qcom_scm_desc *desc,
				struct qcom_scm_res *res)
{
	switch (__get_convention()) {
	case SMC_CONVENTION_ARM_32:
	case SMC_CONVENTION_ARM_64:
		return scm_smc_call(dev, desc, res, true);
	case SMC_CONVENTION_LEGACY:
		return scm_legacy_call_atomic(dev, desc, res);
	default:
		pr_err("Unknown current SCM calling convention.\n");
		return -EINVAL;
	}
}

static bool __qcom_scm_is_call_available(struct device *dev, u32 svc_id,
					 u32 cmd_id)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_INFO,
		.cmd = QCOM_SCM_INFO_IS_CALL_AVAIL,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	desc.arginfo = QCOM_SCM_ARGS(1);
	switch (__get_convention()) {
	case SMC_CONVENTION_ARM_32:
	case SMC_CONVENTION_ARM_64:
		desc.args[0] = SCM_SMC_FNID(svc_id, cmd_id) |
				(ARM_SMCCC_OWNER_SIP << ARM_SMCCC_OWNER_SHIFT);
		break;
	case SMC_CONVENTION_LEGACY:
		desc.args[0] = SCM_LEGACY_FNID(svc_id, cmd_id);
		break;
	default:
		pr_err("Unknown SMC convention being used\n");
		return false;
	}

	ret = qcom_scm_call(dev, &desc, &res);

	return ret ? false : !!res.result[0];
}

static int qcom_scm_set_boot_addr(void *entry, const u8 *cpu_bits)
{
	int cpu;
	unsigned int flags = 0;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_ADDR,
		.arginfo = QCOM_SCM_ARGS(2),
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	for_each_present_cpu(cpu) {
		if (cpu >= QCOM_SCM_BOOT_MAX_CPUS)
			return -EINVAL;
		flags |= cpu_bits[cpu];
	}

	desc.args[0] = flags;
	desc.args[1] = virt_to_phys(entry);

	return qcom_scm_call_atomic(__scm ? __scm->dev : NULL, &desc, NULL);
}

static int qcom_scm_set_boot_addr_mc(void *entry, unsigned int flags)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_ADDR_MC,
		.owner = ARM_SMCCC_OWNER_SIP,
		.arginfo = QCOM_SCM_ARGS(6),
		.args = {
			virt_to_phys(entry),
			/* Apply to all CPUs in all affinity levels */
			~0ULL, ~0ULL, ~0ULL, ~0ULL,
			flags,
		},
	};

	/* Need a device for DMA of the additional arguments */
	if (!__scm || __get_convention() == SMC_CONVENTION_LEGACY)
		return -EOPNOTSUPP;

	return qcom_scm_call(__scm->dev, &desc, NULL);
}

/**
 * qcom_scm_set_warm_boot_addr() - Set the warm boot address for all cpus
 * @entry: Entry point function for the cpus
 *
 * Set the Linux entry point for the SCM to transfer control to when coming
 * out of a power down. CPU power down may be executed on cpuidle or hotplug.
 */
int qcom_scm_set_warm_boot_addr(void *entry)
{
	if (qcom_scm_set_boot_addr_mc(entry, QCOM_SCM_BOOT_MC_FLAG_WARMBOOT))
		/* Fallback to old SCM call */
		return qcom_scm_set_boot_addr(entry, qcom_scm_cpu_warm_bits);
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_set_warm_boot_addr);

/**
 * qcom_scm_set_cold_boot_addr() - Set the cold boot address for all cpus
 * @entry: Entry point function for the cpus
 */
int qcom_scm_set_cold_boot_addr(void *entry)
{
	if (qcom_scm_set_boot_addr_mc(entry, QCOM_SCM_BOOT_MC_FLAG_COLDBOOT))
		/* Fallback to old SCM call */
		return qcom_scm_set_boot_addr(entry, qcom_scm_cpu_cold_bits);
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_set_cold_boot_addr);

/**
 * qcom_scm_cpu_power_down() - Power down the cpu
 * @flags:	Flags to flush cache
 *
 * This is an end point to power down cpu. If there was a pending interrupt,
 * the control would return from this function, otherwise, the cpu jumps to the
 * warm boot entry point set for this cpu upon reset.
 */
void qcom_scm_cpu_power_down(u32 flags)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_TERMINATE_PC,
		.args[0] = flags & QCOM_SCM_FLUSH_FLAG_MASK,
		.arginfo = QCOM_SCM_ARGS(1),
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	qcom_scm_call_atomic(__scm ? __scm->dev : NULL, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_cpu_power_down);

int qcom_scm_set_remote_state(u32 state, u32 id)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_REMOTE_STATE,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = state,
		.args[1] = id,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_set_remote_state);

static int qcom_scm_disable_sdi(void)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SDI_CONFIG,
		.args[0] = 1, /* Disable watchdog debug */
		.args[1] = 0, /* Disable SDI */
		.arginfo = QCOM_SCM_ARGS(2),
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;
	ret = qcom_scm_call(__scm->dev, &desc, &res);

	qcom_scm_clk_disable();

	return ret ? : res.result[0];
}

static int __qcom_scm_set_dload_mode(struct device *dev, bool enable)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_DLOAD_MODE,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = QCOM_SCM_BOOT_SET_DLOAD_MODE,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	desc.args[1] = enable ? QCOM_SCM_BOOT_SET_DLOAD_MODE : 0;

	return qcom_scm_call_atomic(__scm->dev, &desc, NULL);
}

static int qcom_scm_io_rmw(phys_addr_t addr, unsigned int mask, unsigned int val)
{
	unsigned int old;
	unsigned int new;
	int ret;

	ret = qcom_scm_io_readl(addr, &old);
	if (ret)
		return ret;

	new = (old & ~mask) | (val & mask);

	return qcom_scm_io_writel(addr, new);
}

static void qcom_scm_set_download_mode(u32 dload_mode)
{
	int ret = 0;

	if (__scm->dload_mode_addr) {
		ret = qcom_scm_io_rmw(__scm->dload_mode_addr, QCOM_DLOAD_MASK,
				      FIELD_PREP(QCOM_DLOAD_MASK, dload_mode));
	} else if (__qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_BOOT,
						QCOM_SCM_BOOT_SET_DLOAD_MODE)) {
		ret = __qcom_scm_set_dload_mode(__scm->dev, !!dload_mode);
	} else if (dload_mode) {
		dev_err(__scm->dev,
			"No available mechanism for setting download mode\n");
	}

	if (ret)
		dev_err(__scm->dev, "failed to set download mode: %d\n", ret);
}

/**
 * qcom_scm_pas_init_image() - Initialize peripheral authentication service
 *			       state machine for a given peripheral, using the
 *			       metadata
 * @peripheral: peripheral id
 * @metadata:	pointer to memory containing ELF header, program header table
 *		and optional blob of data used for authenticating the metadata
 *		and the rest of the firmware
 * @size:	size of the metadata
 * @ctx:	optional metadata context
 *
 * Return: 0 on success.
 *
 * Upon successful return, the PAS metadata context (@ctx) will be used to
 * track the metadata allocation, this needs to be released by invoking
 * qcom_scm_pas_metadata_release() by the caller.
 */
int qcom_scm_pas_init_image(u32 peripheral, const void *metadata, size_t size,
			    struct qcom_scm_pas_metadata *ctx)
{
	dma_addr_t mdata_phys;
	void *mdata_buf;
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_INIT_IMAGE,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_VAL, QCOM_SCM_RW),
		.args[0] = peripheral,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	/*
	 * During the scm call memory protection will be enabled for the meta
	 * data blob, so make sure it's physically contiguous, 4K aligned and
	 * non-cachable to avoid XPU violations.
	 *
	 * For PIL calls the hypervisor creates SHM Bridges for the blob
	 * buffers on behalf of Linux so we must not do it ourselves hence
	 * not using the TZMem allocator here.
	 *
	 * If we pass a buffer that is already part of an SHM Bridge to this
	 * call, it will fail.
	 */
	mdata_buf = dma_alloc_coherent(__scm->dev, size, &mdata_phys,
				       GFP_KERNEL);
	if (!mdata_buf)
		return -ENOMEM;

	memcpy(mdata_buf, metadata, size);

	ret = qcom_scm_clk_enable();
	if (ret)
		goto out;

	ret = qcom_scm_bw_enable();
	if (ret)
		goto disable_clk;

	desc.args[1] = mdata_phys;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	qcom_scm_bw_disable();

disable_clk:
	qcom_scm_clk_disable();

out:
	if (ret < 0 || !ctx) {
		dma_free_coherent(__scm->dev, size, mdata_buf, mdata_phys);
	} else if (ctx) {
		ctx->ptr = mdata_buf;
		ctx->phys = mdata_phys;
		ctx->size = size;
	}

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_init_image);

/**
 * qcom_scm_pas_metadata_release() - release metadata context
 * @ctx:	metadata context
 */
void qcom_scm_pas_metadata_release(struct qcom_scm_pas_metadata *ctx)
{
	if (!ctx->ptr)
		return;

	dma_free_coherent(__scm->dev, ctx->size, ctx->ptr, ctx->phys);

	ctx->ptr = NULL;
	ctx->phys = 0;
	ctx->size = 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_metadata_release);

/**
 * qcom_scm_pas_mem_setup() - Prepare the memory related to a given peripheral
 *			      for firmware loading
 * @peripheral:	peripheral id
 * @addr:	start address of memory area to prepare
 * @size:	size of the memory area to prepare
 *
 * Returns 0 on success.
 */
int qcom_scm_pas_mem_setup(u32 peripheral, phys_addr_t addr, phys_addr_t size)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_MEM_SETUP,
		.arginfo = QCOM_SCM_ARGS(3),
		.args[0] = peripheral,
		.args[1] = addr,
		.args[2] = size,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;

	ret = qcom_scm_bw_enable();
	if (ret)
		goto disable_clk;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	qcom_scm_bw_disable();

disable_clk:
	qcom_scm_clk_disable();

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_mem_setup);

/**
 * qcom_scm_pas_auth_and_reset() - Authenticate the given peripheral firmware
 *				   and reset the remote processor
 * @peripheral:	peripheral id
 *
 * Return 0 on success.
 */
int qcom_scm_pas_auth_and_reset(u32 peripheral)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_AUTH_AND_RESET,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = peripheral,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;

	ret = qcom_scm_bw_enable();
	if (ret)
		goto disable_clk;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	qcom_scm_bw_disable();

disable_clk:
	qcom_scm_clk_disable();

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_auth_and_reset);

/**
 * qcom_scm_pas_shutdown() - Shut down the remote processor
 * @peripheral: peripheral id
 *
 * Returns 0 on success.
 */
int qcom_scm_pas_shutdown(u32 peripheral)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_SHUTDOWN,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = peripheral,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;

	ret = qcom_scm_bw_enable();
	if (ret)
		goto disable_clk;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	qcom_scm_bw_disable();

disable_clk:
	qcom_scm_clk_disable();

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_shutdown);

/**
 * qcom_scm_pas_supported() - Check if the peripheral authentication service is
 *			      available for the given peripherial
 * @peripheral:	peripheral id
 *
 * Returns true if PAS is supported for this peripheral, otherwise false.
 */
bool qcom_scm_pas_supported(u32 peripheral)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_IS_SUPPORTED,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = peripheral,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	if (!__qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_PIL,
					  QCOM_SCM_PIL_PAS_IS_SUPPORTED))
		return false;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? false : !!res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_supported);

static int __qcom_scm_pas_mss_reset(struct device *dev, bool reset)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_MSS_RESET,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = reset,
		.args[1] = 0,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}

static int qcom_scm_pas_reset_assert(struct reset_controller_dev *rcdev,
				     unsigned long idx)
{
	if (idx != 0)
		return -EINVAL;

	return __qcom_scm_pas_mss_reset(__scm->dev, 1);
}

static int qcom_scm_pas_reset_deassert(struct reset_controller_dev *rcdev,
				       unsigned long idx)
{
	if (idx != 0)
		return -EINVAL;

	return __qcom_scm_pas_mss_reset(__scm->dev, 0);
}

static const struct reset_control_ops qcom_scm_pas_reset_ops = {
	.assert = qcom_scm_pas_reset_assert,
	.deassert = qcom_scm_pas_reset_deassert,
};

int qcom_scm_io_readl(phys_addr_t addr, unsigned int *val)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_IO,
		.cmd = QCOM_SCM_IO_READ,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = addr,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;


	ret = qcom_scm_call_atomic(__scm->dev, &desc, &res);
	if (ret >= 0)
		*val = res.result[0];

	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_io_readl);

int qcom_scm_io_writel(phys_addr_t addr, unsigned int val)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_IO,
		.cmd = QCOM_SCM_IO_WRITE,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = addr,
		.args[1] = val,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call_atomic(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_io_writel);

/**
 * qcom_scm_restore_sec_cfg_available() - Check if secure environment
 * supports restore security config interface.
 *
 * Return true if restore-cfg interface is supported, false if not.
 */
bool qcom_scm_restore_sec_cfg_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_MP,
					    QCOM_SCM_MP_RESTORE_SEC_CFG);
}
EXPORT_SYMBOL_GPL(qcom_scm_restore_sec_cfg_available);

int qcom_scm_restore_sec_cfg(u32 device_id, u32 spare)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_RESTORE_SEC_CFG,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = device_id,
		.args[1] = spare,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_restore_sec_cfg);

#define QCOM_SCM_CP_APERTURE_CONTEXT_MASK	GENMASK(7, 0)

bool qcom_scm_set_gpu_smmu_aperture_is_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_MP,
					    QCOM_SCM_MP_CP_SMMU_APERTURE_ID);
}
EXPORT_SYMBOL_GPL(qcom_scm_set_gpu_smmu_aperture_is_available);

int qcom_scm_set_gpu_smmu_aperture(unsigned int context_bank)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_CP_SMMU_APERTURE_ID,
		.arginfo = QCOM_SCM_ARGS(4),
		.args[0] = 0xffff0000 | FIELD_PREP(QCOM_SCM_CP_APERTURE_CONTEXT_MASK, context_bank),
		.args[1] = 0xffffffff,
		.args[2] = 0xffffffff,
		.args[3] = 0xffffffff,
		.owner = ARM_SMCCC_OWNER_SIP
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_set_gpu_smmu_aperture);

int qcom_scm_iommu_secure_ptbl_size(u32 spare, size_t *size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_IOMMU_SECURE_PTBL_SIZE,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = spare,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (size)
		*size = res.result[0];

	return ret ? : res.result[1];
}
EXPORT_SYMBOL_GPL(qcom_scm_iommu_secure_ptbl_size);

int qcom_scm_iommu_secure_ptbl_init(u64 addr, u32 size, u32 spare)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_IOMMU_SECURE_PTBL_INIT,
		.arginfo = QCOM_SCM_ARGS(3, QCOM_SCM_RW, QCOM_SCM_VAL,
					 QCOM_SCM_VAL),
		.args[0] = addr,
		.args[1] = size,
		.args[2] = spare,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, NULL);

	/* the pg table has been initialized already, ignore the error */
	if (ret == -EPERM)
		ret = 0;

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_iommu_secure_ptbl_init);

int qcom_scm_iommu_set_cp_pool_size(u32 spare, u32 size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_IOMMU_SET_CP_POOL_SIZE,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = size,
		.args[1] = spare,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_iommu_set_cp_pool_size);

int qcom_scm_mem_protect_video_var(u32 cp_start, u32 cp_size,
				   u32 cp_nonpixel_start,
				   u32 cp_nonpixel_size)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_VIDEO_VAR,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_VAL, QCOM_SCM_VAL,
					 QCOM_SCM_VAL, QCOM_SCM_VAL),
		.args[0] = cp_start,
		.args[1] = cp_size,
		.args[2] = cp_nonpixel_start,
		.args[3] = cp_nonpixel_size,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_mem_protect_video_var);

static int __qcom_scm_assign_mem(struct device *dev, phys_addr_t mem_region,
				 size_t mem_sz, phys_addr_t src, size_t src_sz,
				 phys_addr_t dest, size_t dest_sz)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_ASSIGN,
		.arginfo = QCOM_SCM_ARGS(7, QCOM_SCM_RO, QCOM_SCM_VAL,
					 QCOM_SCM_RO, QCOM_SCM_VAL, QCOM_SCM_RO,
					 QCOM_SCM_VAL, QCOM_SCM_VAL),
		.args[0] = mem_region,
		.args[1] = mem_sz,
		.args[2] = src,
		.args[3] = src_sz,
		.args[4] = dest,
		.args[5] = dest_sz,
		.args[6] = 0,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(dev, &desc, &res);

	return ret ? : res.result[0];
}

/**
 * qcom_scm_assign_mem() - Make a secure call to reassign memory ownership
 * @mem_addr: mem region whose ownership need to be reassigned
 * @mem_sz:   size of the region.
 * @srcvm:    vmid for current set of owners, each set bit in
 *            flag indicate a unique owner
 * @newvm:    array having new owners and corresponding permission
 *            flags
 * @dest_cnt: number of owners in next set.
 *
 * Return negative errno on failure or 0 on success with @srcvm updated.
 */
int qcom_scm_assign_mem(phys_addr_t mem_addr, size_t mem_sz,
			u64 *srcvm,
			const struct qcom_scm_vmperm *newvm,
			unsigned int dest_cnt)
{
	struct qcom_scm_current_perm_info *destvm;
	struct qcom_scm_mem_map_info *mem_to_map;
	phys_addr_t mem_to_map_phys;
	phys_addr_t dest_phys;
	phys_addr_t ptr_phys;
	size_t mem_to_map_sz;
	size_t dest_sz;
	size_t src_sz;
	size_t ptr_sz;
	int next_vm;
	__le32 *src;
	int ret, i, b;
	u64 srcvm_bits = *srcvm;

	src_sz = hweight64(srcvm_bits) * sizeof(*src);
	mem_to_map_sz = sizeof(*mem_to_map);
	dest_sz = dest_cnt * sizeof(*destvm);
	ptr_sz = ALIGN(src_sz, SZ_64) + ALIGN(mem_to_map_sz, SZ_64) +
			ALIGN(dest_sz, SZ_64);

	void *ptr __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							ptr_sz, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	ptr_phys = qcom_tzmem_to_phys(ptr);

	/* Fill source vmid detail */
	src = ptr;
	i = 0;
	for (b = 0; b < BITS_PER_TYPE(u64); b++) {
		if (srcvm_bits & BIT(b))
			src[i++] = cpu_to_le32(b);
	}

	/* Fill details of mem buff to map */
	mem_to_map = ptr + ALIGN(src_sz, SZ_64);
	mem_to_map_phys = ptr_phys + ALIGN(src_sz, SZ_64);
	mem_to_map->mem_addr = cpu_to_le64(mem_addr);
	mem_to_map->mem_size = cpu_to_le64(mem_sz);

	next_vm = 0;
	/* Fill details of next vmid detail */
	destvm = ptr + ALIGN(mem_to_map_sz, SZ_64) + ALIGN(src_sz, SZ_64);
	dest_phys = ptr_phys + ALIGN(mem_to_map_sz, SZ_64) + ALIGN(src_sz, SZ_64);
	for (i = 0; i < dest_cnt; i++, destvm++, newvm++) {
		destvm->vmid = cpu_to_le32(newvm->vmid);
		destvm->perm = cpu_to_le32(newvm->perm);
		destvm->ctx = 0;
		destvm->ctx_size = 0;
		next_vm |= BIT(newvm->vmid);
	}

	ret = __qcom_scm_assign_mem(__scm->dev, mem_to_map_phys, mem_to_map_sz,
				    ptr_phys, src_sz, dest_phys, dest_sz);
	if (ret) {
		dev_err(__scm->dev,
			"Assign memory protection call failed %d\n", ret);
		return -EINVAL;
	}

	*srcvm = next_vm;
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_assign_mem);

/**
 * qcom_scm_ocmem_lock_available() - is OCMEM lock/unlock interface available
 */
bool qcom_scm_ocmem_lock_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_OCMEM,
					    QCOM_SCM_OCMEM_LOCK_CMD);
}
EXPORT_SYMBOL_GPL(qcom_scm_ocmem_lock_available);

/**
 * qcom_scm_ocmem_lock() - call OCMEM lock interface to assign an OCMEM
 * region to the specified initiator
 *
 * @id:     tz initiator id
 * @offset: OCMEM offset
 * @size:   OCMEM size
 * @mode:   access mode (WIDE/NARROW)
 */
int qcom_scm_ocmem_lock(enum qcom_scm_ocmem_client id, u32 offset, u32 size,
			u32 mode)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_OCMEM,
		.cmd = QCOM_SCM_OCMEM_LOCK_CMD,
		.args[0] = id,
		.args[1] = offset,
		.args[2] = size,
		.args[3] = mode,
		.arginfo = QCOM_SCM_ARGS(4),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_ocmem_lock);

/**
 * qcom_scm_ocmem_unlock() - call OCMEM unlock interface to release an OCMEM
 * region from the specified initiator
 *
 * @id:     tz initiator id
 * @offset: OCMEM offset
 * @size:   OCMEM size
 */
int qcom_scm_ocmem_unlock(enum qcom_scm_ocmem_client id, u32 offset, u32 size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_OCMEM,
		.cmd = QCOM_SCM_OCMEM_UNLOCK_CMD,
		.args[0] = id,
		.args[1] = offset,
		.args[2] = size,
		.arginfo = QCOM_SCM_ARGS(3),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_ocmem_unlock);

/**
 * qcom_scm_ice_available() - Is the ICE key programming interface available?
 *
 * Return: true iff the SCM calls wrapped by qcom_scm_ice_invalidate_key() and
 *	   qcom_scm_ice_set_key() are available.
 */
bool qcom_scm_ice_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_ES,
					    QCOM_SCM_ES_INVALIDATE_ICE_KEY) &&
		__qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_ES,
					     QCOM_SCM_ES_CONFIG_SET_ICE_KEY);
}
EXPORT_SYMBOL_GPL(qcom_scm_ice_available);

/**
 * qcom_scm_ice_invalidate_key() - Invalidate an inline encryption key
 * @index: the keyslot to invalidate
 *
 * The UFSHCI and eMMC standards define a standard way to do this, but it
 * doesn't work on these SoCs; only this SCM call does.
 *
 * It is assumed that the SoC has only one ICE instance being used, as this SCM
 * call doesn't specify which ICE instance the keyslot belongs to.
 *
 * Return: 0 on success; -errno on failure.
 */
int qcom_scm_ice_invalidate_key(u32 index)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd = QCOM_SCM_ES_INVALIDATE_ICE_KEY,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = index,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_ice_invalidate_key);

/**
 * qcom_scm_ice_set_key() - Set an inline encryption key
 * @index: the keyslot into which to set the key
 * @key: the key to program
 * @key_size: the size of the key in bytes
 * @cipher: the encryption algorithm the key is for
 * @data_unit_size: the encryption data unit size, i.e. the size of each
 *		    individual plaintext and ciphertext.  Given in 512-byte
 *		    units, e.g. 1 = 512 bytes, 8 = 4096 bytes, etc.
 *
 * Program a key into a keyslot of Qualcomm ICE (Inline Crypto Engine), where it
 * can then be used to encrypt/decrypt UFS or eMMC I/O requests inline.
 *
 * The UFSHCI and eMMC standards define a standard way to do this, but it
 * doesn't work on these SoCs; only this SCM call does.
 *
 * It is assumed that the SoC has only one ICE instance being used, as this SCM
 * call doesn't specify which ICE instance the keyslot belongs to.
 *
 * Return: 0 on success; -errno on failure.
 */
int qcom_scm_ice_set_key(u32 index, const u8 *key, u32 key_size,
			 enum qcom_scm_ice_cipher cipher, u32 data_unit_size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd = QCOM_SCM_ES_CONFIG_SET_ICE_KEY,
		.arginfo = QCOM_SCM_ARGS(5, QCOM_SCM_VAL, QCOM_SCM_RW,
					 QCOM_SCM_VAL, QCOM_SCM_VAL,
					 QCOM_SCM_VAL),
		.args[0] = index,
		.args[2] = key_size,
		.args[3] = cipher,
		.args[4] = data_unit_size,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	int ret;

	void *keybuf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							   key_size,
							   GFP_KERNEL);
	if (!keybuf)
		return -ENOMEM;
	memcpy(keybuf, key, key_size);
	desc.args[1] = qcom_tzmem_to_phys(keybuf);

	ret = qcom_scm_call(__scm->dev, &desc, NULL);

	memzero_explicit(keybuf, key_size);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_ice_set_key);

bool qcom_scm_has_wrapped_key_support(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_ES,
					    QCOM_SCM_ES_DERIVE_SW_SECRET) &&
	       __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_ES,
					    QCOM_SCM_ES_GENERATE_ICE_KEY) &&
	       __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_ES,
					    QCOM_SCM_ES_PREPARE_ICE_KEY) &&
	       __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_ES,
					    QCOM_SCM_ES_IMPORT_ICE_KEY);
}
EXPORT_SYMBOL_GPL(qcom_scm_has_wrapped_key_support);

/**
 * qcom_scm_derive_sw_secret() - Derive software secret from wrapped key
 * @eph_key: an ephemerally-wrapped key
 * @eph_key_size: size of @eph_key in bytes
 * @sw_secret: output buffer for the software secret
 * @sw_secret_size: size of the software secret to derive in bytes
 *
 * Derive a software secret from an ephemerally-wrapped key for software crypto
 * operations.  This is done by calling into the secure execution environment,
 * which then calls into the hardware to unwrap and derive the secret.
 *
 * For more information on sw_secret, see the "Hardware-wrapped keys" section of
 * Documentation/block/inline-encryption.rst.
 *
 * Return: 0 on success; -errno on failure.
 */
int qcom_scm_derive_sw_secret(const u8 *eph_key, size_t eph_key_size,
			      u8 *sw_secret, size_t sw_secret_size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd = QCOM_SCM_ES_DERIVE_SW_SECRET,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_RW, QCOM_SCM_VAL,
					 QCOM_SCM_RW, QCOM_SCM_VAL),
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	int ret;

	void *eph_key_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
								eph_key_size,
								GFP_KERNEL);
	if (!eph_key_buf)
		return -ENOMEM;

	void *sw_secret_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
								  sw_secret_size,
								  GFP_KERNEL);
	if (!sw_secret_buf)
		return -ENOMEM;

	memcpy(eph_key_buf, eph_key, eph_key_size);
	desc.args[0] = qcom_tzmem_to_phys(eph_key_buf);
	desc.args[1] = eph_key_size;
	desc.args[2] = qcom_tzmem_to_phys(sw_secret_buf);
	desc.args[3] = sw_secret_size;

	ret = qcom_scm_call(__scm->dev, &desc, NULL);
	if (!ret)
		memcpy(sw_secret, sw_secret_buf, sw_secret_size);

	memzero_explicit(eph_key_buf, eph_key_size);
	memzero_explicit(sw_secret_buf, sw_secret_size);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_derive_sw_secret);

/**
 * qcom_scm_generate_ice_key() - Generate a wrapped key for storage encryption
 * @lt_key: output buffer for the long-term wrapped key
 * @lt_key_size: size of @lt_key in bytes.  Must be the exact wrapped key size
 *		 used by the SoC.
 *
 * Generate a key using the built-in HW module in the SoC.  The resulting key is
 * returned wrapped with the platform-specific Key Encryption Key.
 *
 * Return: 0 on success; -errno on failure.
 */
int qcom_scm_generate_ice_key(u8 *lt_key, size_t lt_key_size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd =  QCOM_SCM_ES_GENERATE_ICE_KEY,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_RW, QCOM_SCM_VAL),
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	int ret;

	void *lt_key_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							       lt_key_size,
							       GFP_KERNEL);
	if (!lt_key_buf)
		return -ENOMEM;

	desc.args[0] = qcom_tzmem_to_phys(lt_key_buf);
	desc.args[1] = lt_key_size;

	ret = qcom_scm_call(__scm->dev, &desc, NULL);
	if (!ret)
		memcpy(lt_key, lt_key_buf, lt_key_size);

	memzero_explicit(lt_key_buf, lt_key_size);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_generate_ice_key);

/**
 * qcom_scm_prepare_ice_key() - Re-wrap a key with the per-boot ephemeral key
 * @lt_key: a long-term wrapped key
 * @lt_key_size: size of @lt_key in bytes
 * @eph_key: output buffer for the ephemerally-wrapped key
 * @eph_key_size: size of @eph_key in bytes.  Must be the exact wrapped key size
 *		  used by the SoC.
 *
 * Given a long-term wrapped key, re-wrap it with the per-boot ephemeral key for
 * added protection.  The resulting key will only be valid for the current boot.
 *
 * Return: 0 on success; -errno on failure.
 */
int qcom_scm_prepare_ice_key(const u8 *lt_key, size_t lt_key_size,
			     u8 *eph_key, size_t eph_key_size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd =  QCOM_SCM_ES_PREPARE_ICE_KEY,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_RO, QCOM_SCM_VAL,
					 QCOM_SCM_RW, QCOM_SCM_VAL),
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	int ret;

	void *lt_key_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							       lt_key_size,
							       GFP_KERNEL);
	if (!lt_key_buf)
		return -ENOMEM;

	void *eph_key_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
								eph_key_size,
								GFP_KERNEL);
	if (!eph_key_buf)
		return -ENOMEM;

	memcpy(lt_key_buf, lt_key, lt_key_size);
	desc.args[0] = qcom_tzmem_to_phys(lt_key_buf);
	desc.args[1] = lt_key_size;
	desc.args[2] = qcom_tzmem_to_phys(eph_key_buf);
	desc.args[3] = eph_key_size;

	ret = qcom_scm_call(__scm->dev, &desc, NULL);
	if (!ret)
		memcpy(eph_key, eph_key_buf, eph_key_size);

	memzero_explicit(lt_key_buf, lt_key_size);
	memzero_explicit(eph_key_buf, eph_key_size);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_prepare_ice_key);

/**
 * qcom_scm_import_ice_key() - Import key for storage encryption
 * @raw_key: the raw key to import
 * @raw_key_size: size of @raw_key in bytes
 * @lt_key: output buffer for the long-term wrapped key
 * @lt_key_size: size of @lt_key in bytes.  Must be the exact wrapped key size
 *		 used by the SoC.
 *
 * Import a raw key and return a long-term wrapped key.  Uses the SoC's HWKM to
 * wrap the raw key using the platform-specific Key Encryption Key.
 *
 * Return: 0 on success; -errno on failure.
 */
int qcom_scm_import_ice_key(const u8 *raw_key, size_t raw_key_size,
			    u8 *lt_key, size_t lt_key_size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd =  QCOM_SCM_ES_IMPORT_ICE_KEY,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_RO, QCOM_SCM_VAL,
					 QCOM_SCM_RW, QCOM_SCM_VAL),
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	int ret;

	void *raw_key_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
								raw_key_size,
								GFP_KERNEL);
	if (!raw_key_buf)
		return -ENOMEM;

	void *lt_key_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							       lt_key_size,
							       GFP_KERNEL);
	if (!lt_key_buf)
		return -ENOMEM;

	memcpy(raw_key_buf, raw_key, raw_key_size);
	desc.args[0] = qcom_tzmem_to_phys(raw_key_buf);
	desc.args[1] = raw_key_size;
	desc.args[2] = qcom_tzmem_to_phys(lt_key_buf);
	desc.args[3] = lt_key_size;

	ret = qcom_scm_call(__scm->dev, &desc, NULL);
	if (!ret)
		memcpy(lt_key, lt_key_buf, lt_key_size);

	memzero_explicit(raw_key_buf, raw_key_size);
	memzero_explicit(lt_key_buf, lt_key_size);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_import_ice_key);

/**
 * qcom_scm_hdcp_available() - Check if secure environment supports HDCP.
 *
 * Return true if HDCP is supported, false if not.
 */
bool qcom_scm_hdcp_available(void)
{
	bool avail;
	int ret = qcom_scm_clk_enable();

	if (ret)
		return ret;

	avail = __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_HDCP,
						QCOM_SCM_HDCP_INVOKE);

	qcom_scm_clk_disable();

	return avail;
}
EXPORT_SYMBOL_GPL(qcom_scm_hdcp_available);

/**
 * qcom_scm_hdcp_req() - Send HDCP request.
 * @req: HDCP request array
 * @req_cnt: HDCP request array count
 * @resp: response buffer passed to SCM
 *
 * Write HDCP register(s) through SCM.
 */
int qcom_scm_hdcp_req(struct qcom_scm_hdcp_req *req, u32 req_cnt, u32 *resp)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_HDCP,
		.cmd = QCOM_SCM_HDCP_INVOKE,
		.arginfo = QCOM_SCM_ARGS(10),
		.args = {
			req[0].addr,
			req[0].val,
			req[1].addr,
			req[1].val,
			req[2].addr,
			req[2].val,
			req[3].addr,
			req[3].val,
			req[4].addr,
			req[4].val
		},
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	if (req_cnt > QCOM_SCM_HDCP_MAX_REQ_CNT)
		return -ERANGE;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	*resp = res.result[0];

	qcom_scm_clk_disable();

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_hdcp_req);

int qcom_scm_iommu_set_pt_format(u32 sec_id, u32 ctx_num, u32 pt_fmt)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMMU_PROGRAM,
		.cmd = QCOM_SCM_SMMU_PT_FORMAT,
		.arginfo = QCOM_SCM_ARGS(3),
		.args[0] = sec_id,
		.args[1] = ctx_num,
		.args[2] = pt_fmt, /* 0: LPAE AArch32 - 1: AArch64 */
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_iommu_set_pt_format);

int qcom_scm_qsmmu500_wait_safe_toggle(bool en)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMMU_PROGRAM,
		.cmd = QCOM_SCM_SMMU_CONFIG_ERRATA1,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = QCOM_SCM_SMMU_CONFIG_ERRATA1_CLIENT_ALL,
		.args[1] = en,
		.owner = ARM_SMCCC_OWNER_SIP,
	};


	return qcom_scm_call_atomic(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_qsmmu500_wait_safe_toggle);

bool qcom_scm_lmh_dcvsh_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_LMH, QCOM_SCM_LMH_LIMIT_DCVSH);
}
EXPORT_SYMBOL_GPL(qcom_scm_lmh_dcvsh_available);

/*
 * This is only supposed to be called once by the TZMem module. It takes the
 * SCM struct device as argument and uses it to pass the call as at the time
 * the SHM Bridge is enabled, the SCM is not yet fully set up and doesn't
 * accept global user calls. Don't try to use the __scm pointer here.
 */
int qcom_scm_shm_bridge_enable(struct device *scm_dev)
{
	int ret;

	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_ENABLE,
		.owner = ARM_SMCCC_OWNER_SIP
	};

	struct qcom_scm_res res;

	if (!__qcom_scm_is_call_available(scm_dev, QCOM_SCM_SVC_MP,
					  QCOM_SCM_MP_SHM_BRIDGE_ENABLE))
		return -EOPNOTSUPP;

	ret = qcom_scm_call(scm_dev, &desc, &res);

	if (ret)
		return ret;

	if (res.result[0] == SHMBRIDGE_RESULT_NOTSUPP)
		return -EOPNOTSUPP;

	return res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_shm_bridge_enable);

int qcom_scm_shm_bridge_create(u64 pfn_and_ns_perm_flags,
			       u64 ipfn_and_s_perm_flags, u64 size_and_flags,
			       u64 ns_vmids, u64 *handle)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_CREATE,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = pfn_and_ns_perm_flags,
		.args[1] = ipfn_and_s_perm_flags,
		.args[2] = size_and_flags,
		.args[3] = ns_vmids,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_VAL, QCOM_SCM_VAL,
					 QCOM_SCM_VAL, QCOM_SCM_VAL),
	};

	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (handle && !ret)
		*handle = res.result[1];

	return ret ?: res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_shm_bridge_create);

int qcom_scm_shm_bridge_delete(u64 handle)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_DELETE,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = handle,
		.arginfo = QCOM_SCM_ARGS(1, QCOM_SCM_VAL),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_shm_bridge_delete);

int qcom_scm_lmh_profile_change(u32 profile_id)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_LMH,
		.cmd = QCOM_SCM_LMH_LIMIT_PROFILE_CHANGE,
		.arginfo = QCOM_SCM_ARGS(1, QCOM_SCM_VAL),
		.args[0] = profile_id,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_lmh_profile_change);

int qcom_scm_lmh_dcvsh(u32 payload_fn, u32 payload_reg, u32 payload_val,
		       u64 limit_node, u32 node_id, u64 version)
{
	int ret, payload_size = 5 * sizeof(u32);

	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_LMH,
		.cmd = QCOM_SCM_LMH_LIMIT_DCVSH,
		.arginfo = QCOM_SCM_ARGS(5, QCOM_SCM_RO, QCOM_SCM_VAL, QCOM_SCM_VAL,
					QCOM_SCM_VAL, QCOM_SCM_VAL),
		.args[1] = payload_size,
		.args[2] = limit_node,
		.args[3] = node_id,
		.args[4] = version,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	u32 *payload_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							       payload_size,
							       GFP_KERNEL);
	if (!payload_buf)
		return -ENOMEM;

	payload_buf[0] = payload_fn;
	payload_buf[1] = 0;
	payload_buf[2] = payload_reg;
	payload_buf[3] = 1;
	payload_buf[4] = payload_val;

	desc.args[0] = qcom_tzmem_to_phys(payload_buf);

	ret = qcom_scm_call(__scm->dev, &desc, NULL);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_lmh_dcvsh);

int qcom_scm_gpu_init_regs(u32 gpu_req)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_GPU,
		.cmd = QCOM_SCM_SVC_GPU_INIT_REGS,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = gpu_req,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_gpu_init_regs);

static int qcom_scm_find_dload_address(struct device *dev, u64 *addr)
{
	struct device_node *tcsr;
	struct device_node *np = dev->of_node;
	struct resource res;
	u32 offset;
	int ret;

	tcsr = of_parse_phandle(np, "qcom,dload-mode", 0);
	if (!tcsr)
		return 0;

	ret = of_address_to_resource(tcsr, 0, &res);
	of_node_put(tcsr);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(np, "qcom,dload-mode", 1, &offset);
	if (ret < 0)
		return ret;

	*addr = res.start + offset;

	return 0;
}

#ifdef CONFIG_QCOM_QSEECOM

/* Lock for QSEECOM SCM call executions */
static DEFINE_MUTEX(qcom_scm_qseecom_call_lock);

static int __qcom_scm_qseecom_call(const struct qcom_scm_desc *desc,
				   struct qcom_scm_qseecom_resp *res)
{
	struct qcom_scm_res scm_res = {};
	int status;

	/*
	 * QSEECOM SCM calls should not be executed concurrently. Therefore, we
	 * require the respective call lock to be held.
	 */
	lockdep_assert_held(&qcom_scm_qseecom_call_lock);

	status = qcom_scm_call(__scm->dev, desc, &scm_res);

	res->result = scm_res.result[0];
	res->resp_type = scm_res.result[1];
	res->data = scm_res.result[2];

	if (status)
		return status;

	return 0;
}

/**
 * qcom_scm_qseecom_call() - Perform a QSEECOM SCM call.
 * @desc: SCM call descriptor.
 * @res:  SCM call response (output).
 *
 * Performs the QSEECOM SCM call described by @desc, returning the response in
 * @rsp.
 *
 * Return: Zero on success, nonzero on failure.
 */
static int qcom_scm_qseecom_call(const struct qcom_scm_desc *desc,
				 struct qcom_scm_qseecom_resp *res)
{
	int status;

	/*
	 * Note: Multiple QSEECOM SCM calls should not be executed same time,
	 * so lock things here. This needs to be extended to callback/listener
	 * handling when support for that is implemented.
	 */

	mutex_lock(&qcom_scm_qseecom_call_lock);
	status = __qcom_scm_qseecom_call(desc, res);
	mutex_unlock(&qcom_scm_qseecom_call_lock);

	dev_dbg(__scm->dev, "%s: owner=%x, svc=%x, cmd=%x, result=%lld, type=%llx, data=%llx\n",
		__func__, desc->owner, desc->svc, desc->cmd, res->result,
		res->resp_type, res->data);

	if (status) {
		dev_err(__scm->dev, "qseecom: scm call failed with error %d\n", status);
		return status;
	}

	/*
	 * TODO: Handle incomplete and blocked calls:
	 *
	 * Incomplete and blocked calls are not supported yet. Some devices
	 * and/or commands require those, some don't. Let's warn about them
	 * prominently in case someone attempts to try these commands with a
	 * device/command combination that isn't supported yet.
	 */
	WARN_ON(res->result == QSEECOM_RESULT_INCOMPLETE);
	WARN_ON(res->result == QSEECOM_RESULT_BLOCKED_ON_LISTENER);

	return 0;
}

/**
 * qcom_scm_qseecom_get_version() - Query the QSEECOM version.
 * @version: Pointer where the QSEECOM version will be stored.
 *
 * Performs the QSEECOM SCM querying the QSEECOM version currently running in
 * the TrustZone.
 *
 * Return: Zero on success, nonzero on failure.
 */
static int qcom_scm_qseecom_get_version(u32 *version)
{
	struct qcom_scm_desc desc = {};
	struct qcom_scm_qseecom_resp res = {};
	u32 feature = 10;
	int ret;

	desc.owner = QSEECOM_TZ_OWNER_SIP;
	desc.svc = QSEECOM_TZ_SVC_INFO;
	desc.cmd = QSEECOM_TZ_CMD_INFO_VERSION;
	desc.arginfo = QCOM_SCM_ARGS(1, QCOM_SCM_VAL);
	desc.args[0] = feature;

	ret = qcom_scm_qseecom_call(&desc, &res);
	if (ret)
		return ret;

	*version = res.result;
	return 0;
}

/**
 * qcom_scm_qseecom_app_get_id() - Query the app ID for a given QSEE app name.
 * @app_name: The name of the app.
 * @app_id:   The returned app ID.
 *
 * Query and return the application ID of the SEE app identified by the given
 * name. This returned ID is the unique identifier of the app required for
 * subsequent communication.
 *
 * Return: Zero on success, nonzero on failure, -ENOENT if the app has not been
 * loaded or could not be found.
 */
int qcom_scm_qseecom_app_get_id(const char *app_name, u32 *app_id)
{
	unsigned long name_buf_size = QSEECOM_MAX_APP_NAME_SIZE;
	unsigned long app_name_len = strlen(app_name);
	struct qcom_scm_desc desc = {};
	struct qcom_scm_qseecom_resp res = {};
	int status;

	if (app_name_len >= name_buf_size)
		return -EINVAL;

	char *name_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							     name_buf_size,
							     GFP_KERNEL);
	if (!name_buf)
		return -ENOMEM;

	memcpy(name_buf, app_name, app_name_len);

	desc.owner = QSEECOM_TZ_OWNER_QSEE_OS;
	desc.svc = QSEECOM_TZ_SVC_APP_MGR;
	desc.cmd = QSEECOM_TZ_CMD_APP_LOOKUP;
	desc.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_RW, QCOM_SCM_VAL);
	desc.args[0] = qcom_tzmem_to_phys(name_buf);
	desc.args[1] = app_name_len;

	status = qcom_scm_qseecom_call(&desc, &res);

	if (status)
		return status;

	if (res.result == QSEECOM_RESULT_FAILURE)
		return -ENOENT;

	if (res.result != QSEECOM_RESULT_SUCCESS)
		return -EINVAL;

	if (res.resp_type != QSEECOM_SCM_RES_APP_ID)
		return -EINVAL;

	*app_id = res.data;
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_qseecom_app_get_id);

/**
 * qcom_scm_qseecom_app_send() - Send to and receive data from a given QSEE app.
 * @app_id:   The ID of the target app.
 * @req:      Request buffer sent to the app (must be TZ memory)
 * @req_size: Size of the request buffer.
 * @rsp:      Response buffer, written to by the app (must be TZ memory)
 * @rsp_size: Size of the response buffer.
 *
 * Sends a request to the QSEE app associated with the given ID and read back
 * its response. The caller must provide two DMA memory regions, one for the
 * request and one for the response, and fill out the @req region with the
 * respective (app-specific) request data. The QSEE app reads this and returns
 * its response in the @rsp region.
 *
 * Return: Zero on success, nonzero on failure.
 */
int qcom_scm_qseecom_app_send(u32 app_id, void *req, size_t req_size,
			      void *rsp, size_t rsp_size)
{
	struct qcom_scm_qseecom_resp res = {};
	struct qcom_scm_desc desc = {};
	phys_addr_t req_phys;
	phys_addr_t rsp_phys;
	int status;

	req_phys = qcom_tzmem_to_phys(req);
	rsp_phys = qcom_tzmem_to_phys(rsp);

	desc.owner = QSEECOM_TZ_OWNER_TZ_APPS;
	desc.svc = QSEECOM_TZ_SVC_APP_ID_PLACEHOLDER;
	desc.cmd = QSEECOM_TZ_CMD_APP_SEND;
	desc.arginfo = QCOM_SCM_ARGS(5, QCOM_SCM_VAL,
				     QCOM_SCM_RW, QCOM_SCM_VAL,
				     QCOM_SCM_RW, QCOM_SCM_VAL);
	desc.args[0] = app_id;
	desc.args[1] = req_phys;
	desc.args[2] = req_size;
	desc.args[3] = rsp_phys;
	desc.args[4] = rsp_size;

	status = qcom_scm_qseecom_call(&desc, &res);

	if (status)
		return status;

	if (res.result != QSEECOM_RESULT_SUCCESS)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_qseecom_app_send);

/*
 * We do not yet support re-entrant calls via the qseecom interface. To prevent
 + any potential issues with this, only allow validated machines for now.
 */
static const struct of_device_id qcom_scm_qseecom_allowlist[] __maybe_unused = {
	{ .compatible = "asus,vivobook-s15" },
	{ .compatible = "asus,zenbook-a14-ux3407qa" },
	{ .compatible = "asus,zenbook-a14-ux3407ra" },
	{ .compatible = "dell,xps13-9345" },
	{ .compatible = "hp,elitebook-ultra-g1q" },
	{ .compatible = "hp,omnibook-x14" },
	{ .compatible = "huawei,gaokun3" },
	{ .compatible = "lenovo,flex-5g" },
	{ .compatible = "lenovo,thinkpad-t14s" },
	{ .compatible = "lenovo,thinkpad-x13s", },
	{ .compatible = "lenovo,yoga-slim7x" },
	{ .compatible = "microsoft,arcata", },
	{ .compatible = "microsoft,blackrock" },
	{ .compatible = "microsoft,romulus13", },
	{ .compatible = "microsoft,romulus15", },
	{ .compatible = "qcom,sc8180x-primus" },
	{ .compatible = "qcom,x1e001de-devkit" },
	{ .compatible = "qcom,x1e80100-crd" },
	{ .compatible = "qcom,x1e80100-qcp" },
	{ .compatible = "qcom,x1p42100-crd" },
	{ }
};

static bool qcom_scm_qseecom_machine_is_allowed(void)
{
	struct device_node *np;
	bool match;

	np = of_find_node_by_path("/");
	if (!np)
		return false;

	match = of_match_node(qcom_scm_qseecom_allowlist, np);
	of_node_put(np);

	return match;
}

static void qcom_scm_qseecom_free(void *data)
{
	struct platform_device *qseecom_dev = data;

	platform_device_del(qseecom_dev);
	platform_device_put(qseecom_dev);
}

static int qcom_scm_qseecom_init(struct qcom_scm *scm)
{
	struct platform_device *qseecom_dev;
	u32 version;
	int ret;

	/*
	 * Note: We do two steps of validation here: First, we try to query the
	 * QSEECOM version as a check to see if the interface exists on this
	 * device. Second, we check against known good devices due to current
	 * driver limitations (see comment in qcom_scm_qseecom_allowlist).
	 *
	 * Note that we deliberately do the machine check after the version
	 * check so that we can log potentially supported devices. This should
	 * be safe as downstream sources indicate that the version query is
	 * neither blocking nor reentrant.
	 */
	ret = qcom_scm_qseecom_get_version(&version);
	if (ret)
		return 0;

	dev_info(scm->dev, "qseecom: found qseecom with version 0x%x\n", version);

	if (!qcom_scm_qseecom_machine_is_allowed()) {
		dev_info(scm->dev, "qseecom: untested machine, skipping\n");
		return 0;
	}

	/*
	 * Set up QSEECOM interface device. All application clients will be
	 * set up and managed by the corresponding driver for it.
	 */
	qseecom_dev = platform_device_alloc("qcom_qseecom", -1);
	if (!qseecom_dev)
		return -ENOMEM;

	qseecom_dev->dev.parent = scm->dev;

	ret = platform_device_add(qseecom_dev);
	if (ret) {
		platform_device_put(qseecom_dev);
		return ret;
	}

	return devm_add_action_or_reset(scm->dev, qcom_scm_qseecom_free, qseecom_dev);
}

#else /* CONFIG_QCOM_QSEECOM */

static int qcom_scm_qseecom_init(struct qcom_scm *scm)
{
	return 0;
}

#endif /* CONFIG_QCOM_QSEECOM */

/**
 * qcom_scm_is_available() - Checks if SCM is available
 */
bool qcom_scm_is_available(void)
{
	/* Paired with smp_store_release() in qcom_scm_probe */
	return !!smp_load_acquire(&__scm);
}
EXPORT_SYMBOL_GPL(qcom_scm_is_available);

static int qcom_scm_assert_valid_wq_ctx(u32 wq_ctx)
{
	/* FW currently only supports a single wq_ctx (zero).
	 * TODO: Update this logic to include dynamic allocation and lookup of
	 * completion structs when FW supports more wq_ctx values.
	 */
	if (wq_ctx != 0) {
		dev_err(__scm->dev, "Firmware unexpectedly passed non-zero wq_ctx\n");
		return -EINVAL;
	}

	return 0;
}

int qcom_scm_wait_for_wq_completion(u32 wq_ctx)
{
	int ret;

	ret = qcom_scm_assert_valid_wq_ctx(wq_ctx);
	if (ret)
		return ret;

	wait_for_completion(&__scm->waitq_comp);

	return 0;
}

static int qcom_scm_waitq_wakeup(unsigned int wq_ctx)
{
	int ret;

	ret = qcom_scm_assert_valid_wq_ctx(wq_ctx);
	if (ret)
		return ret;

	complete(&__scm->waitq_comp);

	return 0;
}

static irqreturn_t qcom_scm_irq_handler(int irq, void *data)
{
	int ret;
	struct qcom_scm *scm = data;
	u32 wq_ctx, flags, more_pending = 0;

	do {
		ret = scm_get_wq_ctx(&wq_ctx, &flags, &more_pending);
		if (ret) {
			dev_err(scm->dev, "GET_WQ_CTX SMC call failed: %d\n", ret);
			goto out;
		}

		if (flags != QCOM_SMC_WAITQ_FLAG_WAKE_ONE) {
			dev_err(scm->dev, "Invalid flags received for wq_ctx: %u\n", flags);
			goto out;
		}

		ret = qcom_scm_waitq_wakeup(wq_ctx);
		if (ret)
			goto out;
	} while (more_pending);

out:
	return IRQ_HANDLED;
}

static int get_download_mode(char *buffer, const struct kernel_param *kp)
{
	if (download_mode >= ARRAY_SIZE(download_mode_name))
		return sysfs_emit(buffer, "unknown mode\n");

	return sysfs_emit(buffer, "%s\n", download_mode_name[download_mode]);
}

static int set_download_mode(const char *val, const struct kernel_param *kp)
{
	bool tmp;
	int ret;

	ret = sysfs_match_string(download_mode_name, val);
	if (ret < 0) {
		ret = kstrtobool(val, &tmp);
		if (ret < 0) {
			pr_err("qcom_scm: err: %d\n", ret);
			return ret;
		}

		ret = tmp ? 1 : 0;
	}

	download_mode = ret;
	if (__scm)
		qcom_scm_set_download_mode(download_mode);

	return 0;
}

static const struct kernel_param_ops download_mode_param_ops = {
	.get = get_download_mode,
	.set = set_download_mode,
};

module_param_cb(download_mode, &download_mode_param_ops, NULL, 0644);
MODULE_PARM_DESC(download_mode, "download mode: off/0/N for no dump mode, full/on/1/Y for full dump mode, mini for minidump mode and full,mini for both full and minidump mode together are acceptable values");

static int qcom_scm_probe(struct platform_device *pdev)
{
	struct qcom_tzmem_pool_config pool_config;
	struct qcom_scm *scm;
	int irq, ret;

	scm = devm_kzalloc(&pdev->dev, sizeof(*scm), GFP_KERNEL);
	if (!scm)
		return -ENOMEM;

	scm->dev = &pdev->dev;
	ret = qcom_scm_find_dload_address(&pdev->dev, &scm->dload_mode_addr);
	if (ret < 0)
		return ret;

	init_completion(&scm->waitq_comp);
	mutex_init(&scm->scm_bw_lock);

	scm->path = devm_of_icc_get(&pdev->dev, NULL);
	if (IS_ERR(scm->path))
		return dev_err_probe(&pdev->dev, PTR_ERR(scm->path),
				     "failed to acquire interconnect path\n");

	scm->core_clk = devm_clk_get_optional(&pdev->dev, "core");
	if (IS_ERR(scm->core_clk))
		return PTR_ERR(scm->core_clk);

	scm->iface_clk = devm_clk_get_optional(&pdev->dev, "iface");
	if (IS_ERR(scm->iface_clk))
		return PTR_ERR(scm->iface_clk);

	scm->bus_clk = devm_clk_get_optional(&pdev->dev, "bus");
	if (IS_ERR(scm->bus_clk))
		return PTR_ERR(scm->bus_clk);

	scm->reset.ops = &qcom_scm_pas_reset_ops;
	scm->reset.nr_resets = 1;
	scm->reset.of_node = pdev->dev.of_node;
	ret = devm_reset_controller_register(&pdev->dev, &scm->reset);
	if (ret)
		return ret;

	/* vote for max clk rate for highest performance */
	ret = clk_set_rate(scm->core_clk, INT_MAX);
	if (ret)
		return ret;

	ret = of_reserved_mem_device_init(scm->dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(scm->dev, ret,
				     "Failed to setup the reserved memory region for TZ mem\n");

	ret = qcom_tzmem_enable(scm->dev);
	if (ret)
		return dev_err_probe(scm->dev, ret,
				     "Failed to enable the TrustZone memory allocator\n");

	memset(&pool_config, 0, sizeof(pool_config));
	pool_config.initial_size = 0;
	pool_config.policy = QCOM_TZMEM_POLICY_ON_DEMAND;
	pool_config.max_size = SZ_256K;

	scm->mempool = devm_qcom_tzmem_pool_new(scm->dev, &pool_config);
	if (IS_ERR(scm->mempool))
		return dev_err_probe(scm->dev, PTR_ERR(scm->mempool),
				     "Failed to create the SCM memory pool\n");

	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0) {
		if (irq != -ENXIO)
			return irq;
	} else {
		ret = devm_request_threaded_irq(scm->dev, irq, NULL, qcom_scm_irq_handler,
						IRQF_ONESHOT, "qcom-scm", scm);
		if (ret < 0)
			return dev_err_probe(scm->dev, ret,
					     "Failed to request qcom-scm irq\n");
	}

	/*
	 * Paired with smp_load_acquire() in qcom_scm_is_available().
	 *
	 * This marks the SCM API as ready to accept user calls and can only
	 * be called after the TrustZone memory pool is initialized and the
	 * waitqueue interrupt requested.
	 */
	smp_store_release(&__scm, scm);

	__get_convention();

	/*
	 * If "download mode" is requested, from this point on warmboot
	 * will cause the boot stages to enter download mode, unless
	 * disabled below by a clean shutdown/reboot.
	 */
	qcom_scm_set_download_mode(download_mode);

	/*
	 * Disable SDI if indicated by DT that it is enabled by default.
	 */
	if (of_property_read_bool(pdev->dev.of_node, "qcom,sdi-enabled") || !download_mode)
		qcom_scm_disable_sdi();

	/*
	 * Initialize the QSEECOM interface.
	 *
	 * Note: QSEECOM is fairly self-contained and this only adds the
	 * interface device (the driver of which does most of the heavy
	 * lifting). So any errors returned here should be either -ENOMEM or
	 * -EINVAL (with the latter only in case there's a bug in our code).
	 * This means that there is no need to bring down the whole SCM driver.
	 * Just log the error instead and let SCM live.
	 */
	ret = qcom_scm_qseecom_init(scm);
	WARN(ret < 0, "failed to initialize qseecom: %d\n", ret);

	return 0;
}

static void qcom_scm_shutdown(struct platform_device *pdev)
{
	/* Clean shutdown, disable download mode to allow normal restart */
	qcom_scm_set_download_mode(QCOM_DLOAD_NODUMP);
}

static const struct of_device_id qcom_scm_dt_match[] = {
	{ .compatible = "qcom,scm" },

	/* Legacy entries kept for backwards compatibility */
	{ .compatible = "qcom,scm-apq8064" },
	{ .compatible = "qcom,scm-apq8084" },
	{ .compatible = "qcom,scm-ipq4019" },
	{ .compatible = "qcom,scm-msm8953" },
	{ .compatible = "qcom,scm-msm8974" },
	{ .compatible = "qcom,scm-msm8996" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_scm_dt_match);

static struct platform_driver qcom_scm_driver = {
	.driver = {
		.name	= "qcom_scm",
		.of_match_table = qcom_scm_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = qcom_scm_probe,
	.shutdown = qcom_scm_shutdown,
};

static int __init qcom_scm_init(void)
{
	return platform_driver_register(&qcom_scm_driver);
}
subsys_initcall(qcom_scm_init);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. SCM driver");
MODULE_LICENSE("GPL v2");
