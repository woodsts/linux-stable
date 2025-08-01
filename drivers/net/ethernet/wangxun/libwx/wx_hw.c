// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2015 - 2022 Beijing WangXun Technology Co., Ltd. */

#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/iopoll.h>
#include <linux/pci.h>

#include "wx_type.h"
#include "wx_lib.h"
#include "wx_sriov.h"
#include "wx_vf.h"
#include "wx_hw.h"

static int wx_phy_read_reg_mdi(struct mii_bus *bus, int phy_addr, int devnum, int regnum)
{
	struct wx *wx = bus->priv;
	u32 command, val;
	int ret;

	/* setup and write the address cycle command */
	command = WX_MSCA_RA(regnum) |
		  WX_MSCA_PA(phy_addr) |
		  WX_MSCA_DA(devnum);
	wr32(wx, WX_MSCA, command);

	command = WX_MSCC_CMD(WX_MSCA_CMD_READ) | WX_MSCC_BUSY;
	if (wx->mac.type == wx_mac_em)
		command |= WX_MDIO_CLK(6);
	wr32(wx, WX_MSCC, command);

	/* wait to complete */
	ret = read_poll_timeout(rd32, val, !(val & WX_MSCC_BUSY), 1000,
				100000, false, wx, WX_MSCC);
	if (ret) {
		wx_err(wx, "Mdio read c22 command did not complete.\n");
		return ret;
	}

	return (u16)rd32(wx, WX_MSCC);
}

static int wx_phy_write_reg_mdi(struct mii_bus *bus, int phy_addr,
				int devnum, int regnum, u16 value)
{
	struct wx *wx = bus->priv;
	u32 command, val;
	int ret;

	/* setup and write the address cycle command */
	command = WX_MSCA_RA(regnum) |
		  WX_MSCA_PA(phy_addr) |
		  WX_MSCA_DA(devnum);
	wr32(wx, WX_MSCA, command);

	command = value | WX_MSCC_CMD(WX_MSCA_CMD_WRITE) | WX_MSCC_BUSY;
	if (wx->mac.type == wx_mac_em)
		command |= WX_MDIO_CLK(6);
	wr32(wx, WX_MSCC, command);

	/* wait to complete */
	ret = read_poll_timeout(rd32, val, !(val & WX_MSCC_BUSY), 1000,
				100000, false, wx, WX_MSCC);
	if (ret)
		wx_err(wx, "Mdio write c22 command did not complete.\n");

	return ret;
}

int wx_phy_read_reg_mdi_c22(struct mii_bus *bus, int phy_addr, int regnum)
{
	struct wx *wx = bus->priv;

	wr32(wx, WX_MDIO_CLAUSE_SELECT, 0xF);
	return wx_phy_read_reg_mdi(bus, phy_addr, 0, regnum);
}
EXPORT_SYMBOL(wx_phy_read_reg_mdi_c22);

int wx_phy_write_reg_mdi_c22(struct mii_bus *bus, int phy_addr, int regnum, u16 value)
{
	struct wx *wx = bus->priv;

	wr32(wx, WX_MDIO_CLAUSE_SELECT, 0xF);
	return wx_phy_write_reg_mdi(bus, phy_addr, 0, regnum, value);
}
EXPORT_SYMBOL(wx_phy_write_reg_mdi_c22);

int wx_phy_read_reg_mdi_c45(struct mii_bus *bus, int phy_addr, int devnum, int regnum)
{
	struct wx *wx = bus->priv;

	wr32(wx, WX_MDIO_CLAUSE_SELECT, 0);
	return wx_phy_read_reg_mdi(bus, phy_addr, devnum, regnum);
}
EXPORT_SYMBOL(wx_phy_read_reg_mdi_c45);

int wx_phy_write_reg_mdi_c45(struct mii_bus *bus, int phy_addr,
			     int devnum, int regnum, u16 value)
{
	struct wx *wx = bus->priv;

	wr32(wx, WX_MDIO_CLAUSE_SELECT, 0);
	return wx_phy_write_reg_mdi(bus, phy_addr, devnum, regnum, value);
}
EXPORT_SYMBOL(wx_phy_write_reg_mdi_c45);

static void wx_intr_disable(struct wx *wx, u64 qmask)
{
	u32 mask;

	mask = (qmask & U32_MAX);
	if (mask)
		wr32(wx, WX_PX_IMS(0), mask);

	if (test_bit(WX_FLAG_MULTI_64_FUNC, wx->flags)) {
		mask = (qmask >> 32);
		if (mask)
			wr32(wx, WX_PX_IMS(1), mask);
	}
}

void wx_intr_enable(struct wx *wx, u64 qmask)
{
	u32 mask;

	if (wx->pdev->is_virtfn) {
		wr32(wx, WX_VXIMC, qmask);
		return;
	}

	mask = (qmask & U32_MAX);
	if (mask)
		wr32(wx, WX_PX_IMC(0), mask);

	if (test_bit(WX_FLAG_MULTI_64_FUNC, wx->flags)) {
		mask = (qmask >> 32);
		if (mask)
			wr32(wx, WX_PX_IMC(1), mask);
	}
}
EXPORT_SYMBOL(wx_intr_enable);

/**
 * wx_irq_disable - Mask off interrupt generation on the NIC
 * @wx: board private structure
 **/
void wx_irq_disable(struct wx *wx)
{
	struct pci_dev *pdev = wx->pdev;

	wr32(wx, WX_PX_MISC_IEN, 0);
	wx_intr_disable(wx, WX_INTR_ALL);

	if (pdev->msix_enabled) {
		int vector;

		for (vector = 0; vector < wx->num_q_vectors; vector++)
			synchronize_irq(wx->msix_q_entries[vector].vector);

		synchronize_irq(wx->msix_entry->vector);
	} else {
		synchronize_irq(pdev->irq);
	}
}
EXPORT_SYMBOL(wx_irq_disable);

/* cmd_addr is used for some special command:
 * 1. to be sector address, when implemented erase sector command
 * 2. to be flash address when implemented read, write flash address
 */
static int wx_fmgr_cmd_op(struct wx *wx, u32 cmd, u32 cmd_addr)
{
	u32 cmd_val = 0, val = 0;

	cmd_val = WX_SPI_CMD_CMD(cmd) |
		  WX_SPI_CMD_CLK(WX_SPI_CLK_DIV) |
		  cmd_addr;
	wr32(wx, WX_SPI_CMD, cmd_val);

	return read_poll_timeout(rd32, val, (val & 0x1), 10, 100000,
				 false, wx, WX_SPI_STATUS);
}

static int wx_flash_read_dword(struct wx *wx, u32 addr, u32 *data)
{
	int ret = 0;

	ret = wx_fmgr_cmd_op(wx, WX_SPI_CMD_READ_DWORD, addr);
	if (ret < 0)
		return ret;

	*data = rd32(wx, WX_SPI_DATA);

	return ret;
}

int wx_check_flash_load(struct wx *hw, u32 check_bit)
{
	u32 reg = 0;
	int err = 0;

	/* if there's flash existing */
	if (!(rd32(hw, WX_SPI_STATUS) &
	      WX_SPI_STATUS_FLASH_BYPASS)) {
		/* wait hw load flash done */
		err = read_poll_timeout(rd32, reg, !(reg & check_bit), 20000, 2000000,
					false, hw, WX_SPI_ILDR_STATUS);
		if (err < 0)
			wx_err(hw, "Check flash load timeout.\n");
	}

	return err;
}
EXPORT_SYMBOL(wx_check_flash_load);

void wx_control_hw(struct wx *wx, bool drv)
{
	/* True : Let firmware know the driver has taken over
	 * False : Let firmware take over control of hw
	 */
	wr32m(wx, WX_CFG_PORT_CTL, WX_CFG_PORT_CTL_DRV_LOAD,
	      drv ? WX_CFG_PORT_CTL_DRV_LOAD : 0);
}
EXPORT_SYMBOL(wx_control_hw);

/**
 * wx_mng_present - returns 0 when management capability is present
 * @wx: pointer to hardware structure
 */
int wx_mng_present(struct wx *wx)
{
	u32 fwsm;

	fwsm = rd32(wx, WX_MIS_ST);
	if (fwsm & WX_MIS_ST_MNG_INIT_DN)
		return 0;
	else
		return -EACCES;
}
EXPORT_SYMBOL(wx_mng_present);

/* Software lock to be held while software semaphore is being accessed. */
static DEFINE_MUTEX(wx_sw_sync_lock);

/**
 *  wx_release_sw_sync - Release SW semaphore
 *  @wx: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to release
 *
 *  Releases the SW semaphore for the specified
 *  function (CSR, PHY0, PHY1, EEPROM, Flash)
 **/
static void wx_release_sw_sync(struct wx *wx, u32 mask)
{
	mutex_lock(&wx_sw_sync_lock);
	wr32m(wx, WX_MNG_SWFW_SYNC, mask, 0);
	mutex_unlock(&wx_sw_sync_lock);
}

/**
 *  wx_acquire_sw_sync - Acquire SW semaphore
 *  @wx: pointer to hardware structure
 *  @mask: Mask to specify which semaphore to acquire
 *
 *  Acquires the SW semaphore for the specified
 *  function (CSR, PHY0, PHY1, EEPROM, Flash)
 **/
static int wx_acquire_sw_sync(struct wx *wx, u32 mask)
{
	u32 sem = 0;
	int ret = 0;

	mutex_lock(&wx_sw_sync_lock);
	ret = read_poll_timeout(rd32, sem, !(sem & mask),
				5000, 2000000, false, wx, WX_MNG_SWFW_SYNC);
	if (!ret) {
		sem |= mask;
		wr32(wx, WX_MNG_SWFW_SYNC, sem);
	} else {
		wx_err(wx, "SW Semaphore not granted: 0x%x.\n", sem);
	}
	mutex_unlock(&wx_sw_sync_lock);

	return ret;
}

static int wx_host_interface_command_s(struct wx *wx, u32 *buffer,
				       u32 length, u32 timeout, bool return_data)
{
	u32 hdr_size = sizeof(struct wx_hic_hdr);
	u32 hicr, i, bi, buf[64] = {};
	int status = 0;
	u32 dword_len;
	u16 buf_len;

	status = wx_acquire_sw_sync(wx, WX_MNG_SWFW_SYNC_SW_MB);
	if (status != 0)
		return status;

	dword_len = length >> 2;

	/* The device driver writes the relevant command block
	 * into the ram area.
	 */
	for (i = 0; i < dword_len; i++) {
		wr32a(wx, WX_MNG_MBOX, i, (__force u32)cpu_to_le32(buffer[i]));
		/* write flush */
		buf[i] = rd32a(wx, WX_MNG_MBOX, i);
	}
	/* Setting this bit tells the ARC that a new command is pending. */
	wr32m(wx, WX_MNG_MBOX_CTL,
	      WX_MNG_MBOX_CTL_SWRDY, WX_MNG_MBOX_CTL_SWRDY);

	status = read_poll_timeout(rd32, hicr, hicr & WX_MNG_MBOX_CTL_FWRDY, 1000,
				   timeout * 1000, false, wx, WX_MNG_MBOX_CTL);

	buf[0] = rd32(wx, WX_MNG_MBOX);
	if ((buf[0] & 0xff0000) >> 16 == 0x80) {
		wx_err(wx, "Unknown FW command: 0x%x\n", buffer[0] & 0xff);
		status = -EINVAL;
		goto rel_out;
	}

	/* Check command completion */
	if (status) {
		wx_err(wx, "Command has failed with no status valid.\n");
		wx_dbg(wx, "write value:\n");
		for (i = 0; i < dword_len; i++)
			wx_dbg(wx, "%x ", buffer[i]);
		wx_dbg(wx, "read value:\n");
		for (i = 0; i < dword_len; i++)
			wx_dbg(wx, "%x ", buf[i]);
		wx_dbg(wx, "\ncheck: %x %x\n", buffer[0] & 0xff, ~buf[0] >> 24);

		goto rel_out;
	}

	if (!return_data)
		goto rel_out;

	/* Calculate length in DWORDs */
	dword_len = hdr_size >> 2;

	/* first pull in the header so we know the buffer length */
	for (bi = 0; bi < dword_len; bi++) {
		buffer[bi] = rd32a(wx, WX_MNG_MBOX, bi);
		le32_to_cpus(&buffer[bi]);
	}

	/* If there is any thing in data position pull it in */
	buf_len = ((struct wx_hic_hdr *)buffer)->buf_len;
	if (buf_len == 0)
		goto rel_out;

	if (length < buf_len + hdr_size) {
		wx_err(wx, "Buffer not large enough for reply message.\n");
		status = -EFAULT;
		goto rel_out;
	}

	/* Calculate length in DWORDs, add 3 for odd lengths */
	dword_len = (buf_len + 3) >> 2;

	/* Pull in the rest of the buffer (bi is where we left off) */
	for (; bi <= dword_len; bi++) {
		buffer[bi] = rd32a(wx, WX_MNG_MBOX, bi);
		le32_to_cpus(&buffer[bi]);
	}

rel_out:
	wx_release_sw_sync(wx, WX_MNG_SWFW_SYNC_SW_MB);
	return status;
}

static bool wx_poll_fw_reply(struct wx *wx, u32 *buffer, u8 send_cmd)
{
	u32 dword_len = sizeof(struct wx_hic_hdr) >> 2;
	struct wx_hic_hdr *recv_hdr;
	u32 i;

	/* read hdr */
	for (i = 0; i < dword_len; i++) {
		buffer[i] = rd32a(wx, WX_FW2SW_MBOX, i);
		le32_to_cpus(&buffer[i]);
	}

	/* check hdr */
	recv_hdr = (struct wx_hic_hdr *)buffer;
	if (recv_hdr->cmd == send_cmd &&
	    recv_hdr->index == wx->swfw_index)
		return true;

	return false;
}

static int wx_host_interface_command_r(struct wx *wx, u32 *buffer,
				       u32 length, u32 timeout, bool return_data)
{
	struct wx_hic_hdr *hdr = (struct wx_hic_hdr *)buffer;
	u32 hdr_size = sizeof(struct wx_hic_hdr);
	bool busy, reply;
	u32 dword_len;
	u16 buf_len;
	int err = 0;
	u8 send_cmd;
	u32 i;

	/* wait to get lock */
	might_sleep();
	err = read_poll_timeout(test_and_set_bit, busy, !busy, 1000, timeout * 1000,
				false, WX_STATE_SWFW_BUSY, wx->state);
	if (err)
		return err;

	/* index to unique seq id for each mbox message */
	hdr->index = wx->swfw_index;
	send_cmd = hdr->cmd;

	dword_len = length >> 2;
	/* write data to SW-FW mbox array */
	for (i = 0; i < dword_len; i++) {
		wr32a(wx, WX_SW2FW_MBOX, i, (__force u32)cpu_to_le32(buffer[i]));
		/* write flush */
		rd32a(wx, WX_SW2FW_MBOX, i);
	}

	/* generate interrupt to notify FW */
	wr32m(wx, WX_SW2FW_MBOX_CMD, WX_SW2FW_MBOX_CMD_VLD, 0);
	wr32m(wx, WX_SW2FW_MBOX_CMD, WX_SW2FW_MBOX_CMD_VLD, WX_SW2FW_MBOX_CMD_VLD);

	/* polling reply from FW */
	err = read_poll_timeout(wx_poll_fw_reply, reply, reply, 2000,
				timeout * 1000, true, wx, buffer, send_cmd);
	if (err) {
		wx_err(wx, "Polling from FW messages timeout, cmd: 0x%x, index: %d\n",
		       send_cmd, wx->swfw_index);
		goto rel_out;
	}

	if (hdr->cmd_or_resp.ret_status == 0x80) {
		wx_err(wx, "Unknown FW command: 0x%x\n", send_cmd);
		err = -EINVAL;
		goto rel_out;
	}

	/* expect no reply from FW then return */
	if (!return_data)
		goto rel_out;

	/* If there is any thing in data position pull it in */
	buf_len = hdr->buf_len;
	if (buf_len == 0)
		goto rel_out;

	if (length < buf_len + hdr_size) {
		wx_err(wx, "Buffer not large enough for reply message.\n");
		err = -EFAULT;
		goto rel_out;
	}

	/* Calculate length in DWORDs, add 3 for odd lengths */
	dword_len = (buf_len + 3) >> 2;
	for (i = hdr_size >> 2; i <= dword_len; i++) {
		buffer[i] = rd32a(wx, WX_FW2SW_MBOX, i);
		le32_to_cpus(&buffer[i]);
	}

rel_out:
	/* index++, index replace wx_hic_hdr.checksum */
	if (wx->swfw_index == WX_HIC_HDR_INDEX_MAX)
		wx->swfw_index = 0;
	else
		wx->swfw_index++;

	clear_bit(WX_STATE_SWFW_BUSY, wx->state);
	return err;
}

/**
 *  wx_host_interface_command - Issue command to manageability block
 *  @wx: pointer to the HW structure
 *  @buffer: contains the command to write and where the return status will
 *   be placed
 *  @length: length of buffer, must be multiple of 4 bytes
 *  @timeout: time in ms to wait for command completion
 *  @return_data: read and return data from the buffer (true) or not (false)
 *   Needed because FW structures are big endian and decoding of
 *   these fields can be 8 bit or 16 bit based on command. Decoding
 *   is not easily understood without making a table of commands.
 *   So we will leave this up to the caller to read back the data
 *   in these cases.
 **/
int wx_host_interface_command(struct wx *wx, u32 *buffer,
			      u32 length, u32 timeout, bool return_data)
{
	if (length == 0 || length > WX_HI_MAX_BLOCK_BYTE_LENGTH) {
		wx_err(wx, "Buffer length failure buffersize=%d.\n", length);
		return -EINVAL;
	}

	/* Calculate length in DWORDs. We must be DWORD aligned */
	if ((length % (sizeof(u32))) != 0) {
		wx_err(wx, "Buffer length failure, not aligned to dword");
		return -EINVAL;
	}

	if (test_bit(WX_FLAG_SWFW_RING, wx->flags))
		return wx_host_interface_command_r(wx, buffer, length,
						   timeout, return_data);

	return wx_host_interface_command_s(wx, buffer, length, timeout, return_data);
}
EXPORT_SYMBOL(wx_host_interface_command);

int wx_set_pps(struct wx *wx, bool enable, u64 nsec, u64 cycles)
{
	struct wx_hic_set_pps pps_cmd;

	pps_cmd.hdr.cmd = FW_PPS_SET_CMD;
	pps_cmd.hdr.buf_len = FW_PPS_SET_LEN;
	pps_cmd.hdr.cmd_or_resp.cmd_resv = FW_CEM_CMD_RESERVED;
	pps_cmd.lan_id = wx->bus.func;
	pps_cmd.enable = (u8)enable;
	pps_cmd.nsec = nsec;
	pps_cmd.cycles = cycles;
	pps_cmd.hdr.checksum = FW_DEFAULT_CHECKSUM;

	return wx_host_interface_command(wx, (u32 *)&pps_cmd,
					 sizeof(pps_cmd),
					 WX_HI_COMMAND_TIMEOUT,
					 false);
}

/**
 *  wx_read_ee_hostif_data - Read EEPROM word using a host interface cmd
 *  assuming that the semaphore is already obtained.
 *  @wx: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM using the hostif.
 **/
static int wx_read_ee_hostif_data(struct wx *wx, u16 offset, u16 *data)
{
	struct wx_hic_read_shadow_ram buffer;
	int status;

	buffer.hdr.req.cmd = FW_READ_SHADOW_RAM_CMD;
	buffer.hdr.req.buf_lenh = 0;
	buffer.hdr.req.buf_lenl = FW_READ_SHADOW_RAM_LEN;
	buffer.hdr.req.checksum = FW_DEFAULT_CHECKSUM;

	/* convert offset from words to bytes */
	buffer.address = (__force u32)cpu_to_be32(offset * 2);
	/* one word */
	buffer.length = (__force u16)cpu_to_be16(sizeof(u16));

	status = wx_host_interface_command(wx, (u32 *)&buffer, sizeof(buffer),
					   WX_HI_COMMAND_TIMEOUT, false);

	if (status != 0)
		return status;

	if (!test_bit(WX_FLAG_SWFW_RING, wx->flags))
		*data = (u16)rd32a(wx, WX_MNG_MBOX, FW_NVM_DATA_OFFSET);
	else
		*data = (u16)rd32a(wx, WX_FW2SW_MBOX, FW_NVM_DATA_OFFSET);

	return status;
}

/**
 *  wx_read_ee_hostif - Read EEPROM word using a host interface cmd
 *  @wx: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @data: word read from the EEPROM
 *
 *  Reads a 16 bit word from the EEPROM using the hostif.
 **/
int wx_read_ee_hostif(struct wx *wx, u16 offset, u16 *data)
{
	int status = 0;

	status = wx_acquire_sw_sync(wx, WX_MNG_SWFW_SYNC_SW_FLASH);
	if (status == 0) {
		status = wx_read_ee_hostif_data(wx, offset, data);
		wx_release_sw_sync(wx, WX_MNG_SWFW_SYNC_SW_FLASH);
	}

	return status;
}
EXPORT_SYMBOL(wx_read_ee_hostif);

/**
 *  wx_read_ee_hostif_buffer- Read EEPROM word(s) using hostif
 *  @wx: pointer to hardware structure
 *  @offset: offset of  word in the EEPROM to read
 *  @words: number of words
 *  @data: word(s) read from the EEPROM
 *
 *  Reads a 16 bit word(s) from the EEPROM using the hostif.
 **/
int wx_read_ee_hostif_buffer(struct wx *wx,
			     u16 offset, u16 words, u16 *data)
{
	struct wx_hic_read_shadow_ram buffer;
	u32 current_word = 0;
	u16 words_to_read;
	u32 value = 0;
	int status;
	u32 mbox;
	u32 i;

	/* Take semaphore for the entire operation. */
	status = wx_acquire_sw_sync(wx, WX_MNG_SWFW_SYNC_SW_FLASH);
	if (status != 0)
		return status;

	while (words) {
		if (words > FW_MAX_READ_BUFFER_SIZE / 2)
			words_to_read = FW_MAX_READ_BUFFER_SIZE / 2;
		else
			words_to_read = words;

		buffer.hdr.req.cmd = FW_READ_SHADOW_RAM_CMD;
		buffer.hdr.req.buf_lenh = 0;
		buffer.hdr.req.buf_lenl = FW_READ_SHADOW_RAM_LEN;
		buffer.hdr.req.checksum = FW_DEFAULT_CHECKSUM;

		/* convert offset from words to bytes */
		buffer.address = (__force u32)cpu_to_be32((offset + current_word) * 2);
		buffer.length = (__force u16)cpu_to_be16(words_to_read * 2);

		status = wx_host_interface_command(wx, (u32 *)&buffer,
						   sizeof(buffer),
						   WX_HI_COMMAND_TIMEOUT,
						   false);

		if (status != 0) {
			wx_err(wx, "Host interface command failed\n");
			goto out;
		}

		if (!test_bit(WX_FLAG_SWFW_RING, wx->flags))
			mbox = WX_MNG_MBOX;
		else
			mbox = WX_FW2SW_MBOX;
		for (i = 0; i < words_to_read; i++) {
			u32 reg = mbox + (FW_NVM_DATA_OFFSET << 2) + 2 * i;

			value = rd32(wx, reg);
			data[current_word] = (u16)(value & 0xffff);
			current_word++;
			i++;
			if (i < words_to_read) {
				value >>= 16;
				data[current_word] = (u16)(value & 0xffff);
				current_word++;
			}
		}
		words -= words_to_read;
	}

out:
	wx_release_sw_sync(wx, WX_MNG_SWFW_SYNC_SW_FLASH);
	return status;
}
EXPORT_SYMBOL(wx_read_ee_hostif_buffer);

/**
 *  wx_init_eeprom_params - Initialize EEPROM params
 *  @wx: pointer to hardware structure
 *
 *  Initializes the EEPROM parameters wx_eeprom_info within the
 *  wx_hw struct in order to set up EEPROM access.
 **/
void wx_init_eeprom_params(struct wx *wx)
{
	struct wx_eeprom_info *eeprom = &wx->eeprom;
	u16 eeprom_size;
	u16 data = 0x80;

	if (eeprom->type == wx_eeprom_uninitialized) {
		eeprom->semaphore_delay = 10;
		eeprom->type = wx_eeprom_none;

		if (!(rd32(wx, WX_SPI_STATUS) &
		      WX_SPI_STATUS_FLASH_BYPASS)) {
			eeprom->type = wx_flash;

			eeprom_size = 4096;
			eeprom->word_size = eeprom_size >> 1;

			wx_dbg(wx, "Eeprom params: type = %d, size = %d\n",
			       eeprom->type, eeprom->word_size);
		}
	}

	switch (wx->mac.type) {
	case wx_mac_sp:
	case wx_mac_aml:
	case wx_mac_aml40:
		if (wx_read_ee_hostif(wx, WX_SW_REGION_PTR, &data)) {
			wx_err(wx, "NVM Read Error\n");
			return;
		}
		data = data >> 1;
		break;
	default:
		break;
	}

	eeprom->sw_region_offset = data;
}
EXPORT_SYMBOL(wx_init_eeprom_params);

/**
 *  wx_get_mac_addr - Generic get MAC address
 *  @wx: pointer to hardware structure
 *  @mac_addr: Adapter MAC address
 *
 *  Reads the adapter's MAC address from first Receive Address Register (RAR0)
 *  A reset of the adapter must be performed prior to calling this function
 *  in order for the MAC address to have been loaded from the EEPROM into RAR0
 **/
void wx_get_mac_addr(struct wx *wx, u8 *mac_addr)
{
	u32 rar_high;
	u32 rar_low;
	u16 i;

	wr32(wx, WX_PSR_MAC_SWC_IDX, 0);
	rar_high = rd32(wx, WX_PSR_MAC_SWC_AD_H);
	rar_low = rd32(wx, WX_PSR_MAC_SWC_AD_L);

	for (i = 0; i < 2; i++)
		mac_addr[i] = (u8)(rar_high >> (1 - i) * 8);

	for (i = 0; i < 4; i++)
		mac_addr[i + 2] = (u8)(rar_low >> (3 - i) * 8);
}
EXPORT_SYMBOL(wx_get_mac_addr);

/**
 *  wx_set_rar - Set Rx address register
 *  @wx: pointer to hardware structure
 *  @index: Receive address register to write
 *  @addr: Address to put into receive address register
 *  @pools: VMDq "set" or "pool" index
 *  @enable_addr: set flag that address is active
 *
 *  Puts an ethernet address into a receive address register.
 **/
static int wx_set_rar(struct wx *wx, u32 index, u8 *addr, u64 pools,
		      u32 enable_addr)
{
	u32 rar_entries = wx->mac.num_rar_entries;
	u32 rar_low, rar_high;

	/* Make sure we are using a valid rar index range */
	if (index >= rar_entries) {
		wx_err(wx, "RAR index %d is out of range.\n", index);
		return -EINVAL;
	}

	/* select the MAC address */
	wr32(wx, WX_PSR_MAC_SWC_IDX, index);

	/* setup VMDq pool mapping */
	wr32(wx, WX_PSR_MAC_SWC_VM_L, pools & 0xFFFFFFFF);

	if (test_bit(WX_FLAG_MULTI_64_FUNC, wx->flags))
		wr32(wx, WX_PSR_MAC_SWC_VM_H, pools >> 32);

	/* HW expects these in little endian so we reverse the byte
	 * order from network order (big endian) to little endian
	 *
	 * Some parts put the VMDq setting in the extra RAH bits,
	 * so save everything except the lower 16 bits that hold part
	 * of the address and the address valid bit.
	 */
	rar_low = ((u32)addr[5] |
		  ((u32)addr[4] << 8) |
		  ((u32)addr[3] << 16) |
		  ((u32)addr[2] << 24));
	rar_high = ((u32)addr[1] |
		   ((u32)addr[0] << 8));
	if (enable_addr != 0)
		rar_high |= WX_PSR_MAC_SWC_AD_H_AV;

	wr32(wx, WX_PSR_MAC_SWC_AD_L, rar_low);
	wr32m(wx, WX_PSR_MAC_SWC_AD_H,
	      (WX_PSR_MAC_SWC_AD_H_AD(U16_MAX) |
	       WX_PSR_MAC_SWC_AD_H_ADTYPE(1) |
	       WX_PSR_MAC_SWC_AD_H_AV),
	      rar_high);

	return 0;
}

/**
 *  wx_clear_rar - Remove Rx address register
 *  @wx: pointer to hardware structure
 *  @index: Receive address register to write
 *
 *  Clears an ethernet address from a receive address register.
 **/
static int wx_clear_rar(struct wx *wx, u32 index)
{
	u32 rar_entries = wx->mac.num_rar_entries;

	/* Make sure we are using a valid rar index range */
	if (index >= rar_entries) {
		wx_err(wx, "RAR index %d is out of range.\n", index);
		return -EINVAL;
	}

	/* Some parts put the VMDq setting in the extra RAH bits,
	 * so save everything except the lower 16 bits that hold part
	 * of the address and the address valid bit.
	 */
	wr32(wx, WX_PSR_MAC_SWC_IDX, index);

	wr32(wx, WX_PSR_MAC_SWC_VM_L, 0);
	wr32(wx, WX_PSR_MAC_SWC_VM_H, 0);

	wr32(wx, WX_PSR_MAC_SWC_AD_L, 0);
	wr32m(wx, WX_PSR_MAC_SWC_AD_H,
	      (WX_PSR_MAC_SWC_AD_H_AD(U16_MAX) |
	       WX_PSR_MAC_SWC_AD_H_ADTYPE(1) |
	       WX_PSR_MAC_SWC_AD_H_AV),
	      0);

	return 0;
}

/**
 *  wx_clear_vmdq - Disassociate a VMDq pool index from a rx address
 *  @wx: pointer to hardware struct
 *  @rar: receive address register index to disassociate
 *  @vmdq: VMDq pool index to remove from the rar
 **/
static int wx_clear_vmdq(struct wx *wx, u32 rar, u32 __maybe_unused vmdq)
{
	u32 rar_entries = wx->mac.num_rar_entries;
	u32 mpsar_lo, mpsar_hi;

	/* Make sure we are using a valid rar index range */
	if (rar >= rar_entries) {
		wx_err(wx, "RAR index %d is out of range.\n", rar);
		return -EINVAL;
	}

	wr32(wx, WX_PSR_MAC_SWC_IDX, rar);
	mpsar_lo = rd32(wx, WX_PSR_MAC_SWC_VM_L);
	mpsar_hi = rd32(wx, WX_PSR_MAC_SWC_VM_H);

	if (!mpsar_lo && !mpsar_hi)
		return 0;

	/* was that the last pool using this rar? */
	if (mpsar_lo == 0 && mpsar_hi == 0 && rar != 0)
		wx_clear_rar(wx, rar);

	return 0;
}

/**
 *  wx_init_uta_tables - Initialize the Unicast Table Array
 *  @wx: pointer to hardware structure
 **/
static void wx_init_uta_tables(struct wx *wx)
{
	int i;

	wx_dbg(wx, " Clearing UTA\n");

	for (i = 0; i < 128; i++)
		wr32(wx, WX_PSR_UC_TBL(i), 0);
}

/**
 *  wx_init_rx_addrs - Initializes receive address filters.
 *  @wx: pointer to hardware structure
 *
 *  Places the MAC address in receive address register 0 and clears the rest
 *  of the receive address registers. Clears the multicast table. Assumes
 *  the receiver is in reset when the routine is called.
 **/
void wx_init_rx_addrs(struct wx *wx)
{
	u32 rar_entries = wx->mac.num_rar_entries;
	u32 psrctl;
	int i;

	/* If the current mac address is valid, assume it is a software override
	 * to the permanent address.
	 * Otherwise, use the permanent address from the eeprom.
	 */
	if (!is_valid_ether_addr(wx->mac.addr)) {
		/* Get the MAC address from the RAR0 for later reference */
		wx_get_mac_addr(wx, wx->mac.addr);
		wx_dbg(wx, "Keeping Current RAR0 Addr = %pM\n", wx->mac.addr);
	} else {
		/* Setup the receive address. */
		wx_dbg(wx, "Overriding MAC Address in RAR[0]\n");
		wx_dbg(wx, "New MAC Addr = %pM\n", wx->mac.addr);

		wx_set_rar(wx, 0, wx->mac.addr, 0, WX_PSR_MAC_SWC_AD_H_AV);

		if (test_bit(WX_FLAG_MULTI_64_FUNC, wx->flags)) {
			/* clear VMDq pool/queue selection for RAR 0 */
			wx_clear_vmdq(wx, 0, WX_CLEAR_VMDQ_ALL);
		}
	}

	/* Zero out the other receive addresses. */
	wx_dbg(wx, "Clearing RAR[1-%d]\n", rar_entries - 1);
	for (i = 1; i < rar_entries; i++) {
		wr32(wx, WX_PSR_MAC_SWC_IDX, i);
		wr32(wx, WX_PSR_MAC_SWC_AD_L, 0);
		wr32(wx, WX_PSR_MAC_SWC_AD_H, 0);
	}

	/* Clear the MTA */
	wx->addr_ctrl.mta_in_use = 0;
	psrctl = rd32(wx, WX_PSR_CTL);
	psrctl &= ~(WX_PSR_CTL_MO | WX_PSR_CTL_MFE);
	psrctl |= wx->mac.mc_filter_type << WX_PSR_CTL_MO_SHIFT;
	wr32(wx, WX_PSR_CTL, psrctl);
	wx_dbg(wx, " Clearing MTA\n");
	for (i = 0; i < wx->mac.mcft_size; i++)
		wr32(wx, WX_PSR_MC_TBL(i), 0);

	wx_init_uta_tables(wx);
}
EXPORT_SYMBOL(wx_init_rx_addrs);

static void wx_sync_mac_table(struct wx *wx)
{
	int i;

	for (i = 0; i < wx->mac.num_rar_entries; i++) {
		if (wx->mac_table[i].state & WX_MAC_STATE_MODIFIED) {
			if (wx->mac_table[i].state & WX_MAC_STATE_IN_USE) {
				wx_set_rar(wx, i,
					   wx->mac_table[i].addr,
					   wx->mac_table[i].pools,
					   WX_PSR_MAC_SWC_AD_H_AV);
			} else {
				wx_clear_rar(wx, i);
			}
			wx->mac_table[i].state &= ~(WX_MAC_STATE_MODIFIED);
		}
	}
}

static void wx_full_sync_mac_table(struct wx *wx)
{
	int i;

	for (i = 0; i < wx->mac.num_rar_entries; i++) {
		if (wx->mac_table[i].state & WX_MAC_STATE_IN_USE) {
			wx_set_rar(wx, i,
				   wx->mac_table[i].addr,
				   wx->mac_table[i].pools,
				   WX_PSR_MAC_SWC_AD_H_AV);
		} else {
			wx_clear_rar(wx, i);
		}
		wx->mac_table[i].state &= ~(WX_MAC_STATE_MODIFIED);
	}
}

/* this function destroys the first RAR entry */
void wx_mac_set_default_filter(struct wx *wx, u8 *addr)
{
	memcpy(&wx->mac_table[0].addr, addr, ETH_ALEN);
	wx->mac_table[0].pools = BIT(VMDQ_P(0));
	wx->mac_table[0].state = (WX_MAC_STATE_DEFAULT | WX_MAC_STATE_IN_USE);
	wx_set_rar(wx, 0, wx->mac_table[0].addr,
		   wx->mac_table[0].pools,
		   WX_PSR_MAC_SWC_AD_H_AV);
}
EXPORT_SYMBOL(wx_mac_set_default_filter);

void wx_flush_sw_mac_table(struct wx *wx)
{
	u32 i;

	for (i = 0; i < wx->mac.num_rar_entries; i++) {
		if (!(wx->mac_table[i].state & WX_MAC_STATE_IN_USE))
			continue;

		wx->mac_table[i].state |= WX_MAC_STATE_MODIFIED;
		wx->mac_table[i].state &= ~WX_MAC_STATE_IN_USE;
		memset(wx->mac_table[i].addr, 0, ETH_ALEN);
		wx->mac_table[i].pools = 0;
	}
	wx_sync_mac_table(wx);
}
EXPORT_SYMBOL(wx_flush_sw_mac_table);

int wx_add_mac_filter(struct wx *wx, u8 *addr, u16 pool)
{
	u32 i;

	if (is_zero_ether_addr(addr))
		return -EINVAL;

	for (i = 0; i < wx->mac.num_rar_entries; i++) {
		if (wx->mac_table[i].state & WX_MAC_STATE_IN_USE) {
			if (ether_addr_equal(addr, wx->mac_table[i].addr)) {
				if (wx->mac_table[i].pools != (1ULL << pool)) {
					memcpy(wx->mac_table[i].addr, addr, ETH_ALEN);
					wx->mac_table[i].pools |= (1ULL << pool);
					wx_sync_mac_table(wx);
					return i;
				}
			}
		}

		if (wx->mac_table[i].state & WX_MAC_STATE_IN_USE)
			continue;
		wx->mac_table[i].state |= (WX_MAC_STATE_MODIFIED |
					   WX_MAC_STATE_IN_USE);
		memcpy(wx->mac_table[i].addr, addr, ETH_ALEN);
		wx->mac_table[i].pools |= (1ULL << pool);
		wx_sync_mac_table(wx);
		return i;
	}
	return -ENOMEM;
}

int wx_del_mac_filter(struct wx *wx, u8 *addr, u16 pool)
{
	u32 i;

	if (is_zero_ether_addr(addr))
		return -EINVAL;

	/* search table for addr, if found, set to 0 and sync */
	for (i = 0; i < wx->mac.num_rar_entries; i++) {
		if (!ether_addr_equal(addr, wx->mac_table[i].addr))
			continue;

		wx->mac_table[i].state |= WX_MAC_STATE_MODIFIED;
		wx->mac_table[i].pools &= ~(1ULL << pool);
		if (!wx->mac_table[i].pools) {
			wx->mac_table[i].state &= ~WX_MAC_STATE_IN_USE;
			memset(wx->mac_table[i].addr, 0, ETH_ALEN);
		}
		wx_sync_mac_table(wx);
		return 0;
	}
	return -ENOMEM;
}

static int wx_available_rars(struct wx *wx)
{
	u32 i, count = 0;

	for (i = 0; i < wx->mac.num_rar_entries; i++) {
		if (wx->mac_table[i].state == 0)
			count++;
	}

	return count;
}

/**
 * wx_write_uc_addr_list - write unicast addresses to RAR table
 * @netdev: network interface device structure
 * @pool: index for mac table
 *
 * Writes unicast address list to the RAR table.
 * Returns: -ENOMEM on failure/insufficient address space
 *                0 on no addresses written
 *                X on writing X addresses to the RAR table
 **/
static int wx_write_uc_addr_list(struct net_device *netdev, int pool)
{
	struct wx *wx = netdev_priv(netdev);
	int count = 0;

	/* return ENOMEM indicating insufficient memory for addresses */
	if (netdev_uc_count(netdev) > wx_available_rars(wx))
		return -ENOMEM;

	if (!netdev_uc_empty(netdev)) {
		struct netdev_hw_addr *ha;

		netdev_for_each_uc_addr(ha, netdev) {
			wx_del_mac_filter(wx, ha->addr, pool);
			wx_add_mac_filter(wx, ha->addr, pool);
			count++;
		}
	}
	return count;
}

/**
 *  wx_mta_vector - Determines bit-vector in multicast table to set
 *  @wx: pointer to private structure
 *  @mc_addr: the multicast address
 *
 *  Extracts the 12 bits, from a multicast address, to determine which
 *  bit-vector to set in the multicast table. The hardware uses 12 bits, from
 *  incoming rx multicast addresses, to determine the bit-vector to check in
 *  the MTA. Which of the 4 combination, of 12-bits, the hardware uses is set
 *  by the MO field of the MCSTCTRL. The MO field is set during initialization
 *  to mc_filter_type.
 **/
u32 wx_mta_vector(struct wx *wx, u8 *mc_addr)
{
	u32 vector = 0;

	switch (wx->mac.mc_filter_type) {
	case 0:   /* use bits [47:36] of the address */
		vector = ((mc_addr[4] >> 4) | (((u16)mc_addr[5]) << 4));
		break;
	case 1:   /* use bits [46:35] of the address */
		vector = ((mc_addr[4] >> 3) | (((u16)mc_addr[5]) << 5));
		break;
	case 2:   /* use bits [45:34] of the address */
		vector = ((mc_addr[4] >> 2) | (((u16)mc_addr[5]) << 6));
		break;
	case 3:   /* use bits [43:32] of the address */
		vector = ((mc_addr[4]) | (((u16)mc_addr[5]) << 8));
		break;
	default:  /* Invalid mc_filter_type */
		wx_err(wx, "MC filter type param set incorrectly\n");
		break;
	}

	/* vector can only be 12-bits or boundary will be exceeded */
	vector &= 0xFFF;
	return vector;
}

/**
 *  wx_set_mta - Set bit-vector in multicast table
 *  @wx: pointer to private structure
 *  @mc_addr: Multicast address
 *
 *  Sets the bit-vector in the multicast table.
 **/
static void wx_set_mta(struct wx *wx, u8 *mc_addr)
{
	u32 vector, vector_bit, vector_reg;

	wx->addr_ctrl.mta_in_use++;

	vector = wx_mta_vector(wx, mc_addr);
	wx_dbg(wx, " bit-vector = 0x%03X\n", vector);

	/* The MTA is a register array of 128 32-bit registers. It is treated
	 * like an array of 4096 bits.  We want to set bit
	 * BitArray[vector_value]. So we figure out what register the bit is
	 * in, read it, OR in the new bit, then write back the new value.  The
	 * register is determined by the upper 7 bits of the vector value and
	 * the bit within that register are determined by the lower 5 bits of
	 * the value.
	 */
	vector_reg = (vector >> 5) & 0x7F;
	vector_bit = vector & 0x1F;
	wx->mac.mta_shadow[vector_reg] |= (1 << vector_bit);
}

/**
 *  wx_update_mc_addr_list - Updates MAC list of multicast addresses
 *  @wx: pointer to private structure
 *  @netdev: pointer to net device structure
 *
 *  The given list replaces any existing list. Clears the MC addrs from receive
 *  address registers and the multicast table. Uses unused receive address
 *  registers for the first multicast addresses, and hashes the rest into the
 *  multicast table.
 **/
static void wx_update_mc_addr_list(struct wx *wx, struct net_device *netdev)
{
	struct netdev_hw_addr *ha;
	u32 i, psrctl;

	/* Set the new number of MC addresses that we are being requested to
	 * use.
	 */
	wx->addr_ctrl.num_mc_addrs = netdev_mc_count(netdev);
	wx->addr_ctrl.mta_in_use = 0;

	/* Clear mta_shadow */
	wx_dbg(wx, " Clearing MTA\n");
	memset(&wx->mac.mta_shadow, 0, sizeof(wx->mac.mta_shadow));

	/* Update mta_shadow */
	netdev_for_each_mc_addr(ha, netdev) {
		wx_dbg(wx, " Adding the multicast addresses:\n");
		wx_set_mta(wx, ha->addr);
	}

	/* Enable mta */
	for (i = 0; i < wx->mac.mcft_size; i++)
		wr32a(wx, WX_PSR_MC_TBL(0), i,
		      wx->mac.mta_shadow[i]);

	if (wx->addr_ctrl.mta_in_use > 0) {
		psrctl = rd32(wx, WX_PSR_CTL);
		psrctl &= ~(WX_PSR_CTL_MO | WX_PSR_CTL_MFE);
		psrctl |= WX_PSR_CTL_MFE |
			  (wx->mac.mc_filter_type << WX_PSR_CTL_MO_SHIFT);
		wr32(wx, WX_PSR_CTL, psrctl);
	}

	wx_dbg(wx, "Update mc addr list Complete\n");
}

static void wx_restore_vf_multicasts(struct wx *wx)
{
	u32 i, j, vector_bit, vector_reg;
	struct vf_data_storage *vfinfo;

	for (i = 0; i < wx->num_vfs; i++) {
		u32 vmolr = rd32(wx, WX_PSR_VM_L2CTL(i));

		vfinfo = &wx->vfinfo[i];
		for (j = 0; j < vfinfo->num_vf_mc_hashes; j++) {
			wx->addr_ctrl.mta_in_use++;
			vector_reg = WX_PSR_MC_TBL_REG(vfinfo->vf_mc_hashes[j]);
			vector_bit = WX_PSR_MC_TBL_BIT(vfinfo->vf_mc_hashes[j]);
			wr32m(wx, WX_PSR_MC_TBL(vector_reg),
			      BIT(vector_bit), BIT(vector_bit));
			/* errata 5: maintain a copy of the reg table conf */
			wx->mac.mta_shadow[vector_reg] |= BIT(vector_bit);
		}
		if (vfinfo->num_vf_mc_hashes)
			vmolr |= WX_PSR_VM_L2CTL_ROMPE;
		else
			vmolr &= ~WX_PSR_VM_L2CTL_ROMPE;
		wr32(wx, WX_PSR_VM_L2CTL(i), vmolr);
	}

	/* Restore any VF macvlans */
	wx_full_sync_mac_table(wx);
}

/**
 * wx_write_mc_addr_list - write multicast addresses to MTA
 * @netdev: network interface device structure
 *
 * Writes multicast address list to the MTA hash table.
 * Returns: 0 on no addresses written
 *          X on writing X addresses to MTA
 **/
static int wx_write_mc_addr_list(struct net_device *netdev)
{
	struct wx *wx = netdev_priv(netdev);

	if (!netif_running(netdev))
		return 0;

	wx_update_mc_addr_list(wx, netdev);

	if (test_bit(WX_FLAG_SRIOV_ENABLED, wx->flags))
		wx_restore_vf_multicasts(wx);

	return netdev_mc_count(netdev);
}

/**
 * wx_set_mac - Change the Ethernet Address of the NIC
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
int wx_set_mac(struct net_device *netdev, void *p)
{
	struct wx *wx = netdev_priv(netdev);
	struct sockaddr *addr = p;
	int retval;

	retval = eth_prepare_mac_addr_change(netdev, addr);
	if (retval)
		return retval;

	wx_del_mac_filter(wx, wx->mac.addr, VMDQ_P(0));
	eth_hw_addr_set(netdev, addr->sa_data);
	memcpy(wx->mac.addr, addr->sa_data, netdev->addr_len);

	wx_mac_set_default_filter(wx, wx->mac.addr);

	return 0;
}
EXPORT_SYMBOL(wx_set_mac);

void wx_disable_rx(struct wx *wx)
{
	u32 pfdtxgswc;
	u32 rxctrl;

	rxctrl = rd32(wx, WX_RDB_PB_CTL);
	if (rxctrl & WX_RDB_PB_CTL_RXEN) {
		pfdtxgswc = rd32(wx, WX_PSR_CTL);
		if (pfdtxgswc & WX_PSR_CTL_SW_EN) {
			pfdtxgswc &= ~WX_PSR_CTL_SW_EN;
			wr32(wx, WX_PSR_CTL, pfdtxgswc);
			wx->mac.set_lben = true;
		} else {
			wx->mac.set_lben = false;
		}
		rxctrl &= ~WX_RDB_PB_CTL_RXEN;
		wr32(wx, WX_RDB_PB_CTL, rxctrl);

		if (!(((wx->subsystem_device_id & WX_NCSI_MASK) == WX_NCSI_SUP) ||
		      ((wx->subsystem_device_id & WX_WOL_MASK) == WX_WOL_SUP))) {
			/* disable mac receiver */
			wr32m(wx, WX_MAC_RX_CFG,
			      WX_MAC_RX_CFG_RE, 0);
		}
	}
}
EXPORT_SYMBOL(wx_disable_rx);

static void wx_enable_rx(struct wx *wx)
{
	u32 psrctl;

	/* enable mac receiver */
	wr32m(wx, WX_MAC_RX_CFG,
	      WX_MAC_RX_CFG_RE, WX_MAC_RX_CFG_RE);

	wr32m(wx, WX_RDB_PB_CTL,
	      WX_RDB_PB_CTL_RXEN, WX_RDB_PB_CTL_RXEN);

	if (wx->mac.set_lben) {
		psrctl = rd32(wx, WX_PSR_CTL);
		psrctl |= WX_PSR_CTL_SW_EN;
		wr32(wx, WX_PSR_CTL, psrctl);
		wx->mac.set_lben = false;
	}
}

/**
 * wx_set_rxpba - Initialize Rx packet buffer
 * @wx: pointer to private structure
 **/
static void wx_set_rxpba(struct wx *wx)
{
	u32 rxpktsize, txpktsize, txpbthresh;
	u32 pbsize = wx->mac.rx_pb_size;

	if (test_bit(WX_FLAG_FDIR_CAPABLE, wx->flags)) {
		if (test_bit(WX_FLAG_FDIR_HASH, wx->flags) ||
		    test_bit(WX_FLAG_FDIR_PERFECT, wx->flags))
			pbsize -= 64; /* Default 64KB */
	}

	rxpktsize = pbsize << WX_RDB_PB_SZ_SHIFT;
	wr32(wx, WX_RDB_PB_SZ(0), rxpktsize);

	/* Only support an equally distributed Tx packet buffer strategy. */
	txpktsize = wx->mac.tx_pb_size;
	txpbthresh = (txpktsize / 1024) - WX_TXPKT_SIZE_MAX;
	wr32(wx, WX_TDB_PB_SZ(0), txpktsize);
	wr32(wx, WX_TDM_PB_THRE(0), txpbthresh);
}

#define WX_ETH_FRAMING 20

/**
 * wx_hpbthresh - calculate high water mark for flow control
 *
 * @wx: board private structure to calculate for
 **/
static int wx_hpbthresh(struct wx *wx)
{
	struct net_device *dev = wx->netdev;
	int link, tc, kb, marker;
	u32 dv_id, rx_pba;

	/* Calculate max LAN frame size */
	link = dev->mtu + ETH_HLEN + ETH_FCS_LEN + WX_ETH_FRAMING;
	tc = link;

	/* Calculate delay value for device */
	dv_id = WX_DV(link, tc);

	/* Loopback switch introduces additional latency */
	if (test_bit(WX_FLAG_SRIOV_ENABLED, wx->flags))
		dv_id += WX_B2BT(tc);

	/* Delay value is calculated in bit times convert to KB */
	kb = WX_BT2KB(dv_id);
	rx_pba = rd32(wx, WX_RDB_PB_SZ(0)) >> WX_RDB_PB_SZ_SHIFT;

	marker = rx_pba - kb;

	/* It is possible that the packet buffer is not large enough
	 * to provide required headroom. In this case throw an error
	 * to user and a do the best we can.
	 */
	if (marker < 0) {
		dev_warn(&wx->pdev->dev,
			 "Packet Buffer can not provide enough headroom to support flow control. Decrease MTU or number of traffic classes\n");
		marker = tc + 1;
	}

	return marker;
}

/**
 * wx_lpbthresh - calculate low water mark for flow control
 *
 * @wx: board private structure to calculate for
 **/
static int wx_lpbthresh(struct wx *wx)
{
	struct net_device *dev = wx->netdev;
	u32 dv_id;
	int tc;

	/* Calculate max LAN frame size */
	tc = dev->mtu + ETH_HLEN + ETH_FCS_LEN;

	/* Calculate delay value for device */
	dv_id = WX_LOW_DV(tc);

	/* Delay value is calculated in bit times convert to KB */
	return WX_BT2KB(dv_id);
}

/**
 * wx_pbthresh_setup - calculate and setup high low water marks
 *
 * @wx: board private structure to calculate for
 **/
static void wx_pbthresh_setup(struct wx *wx)
{
	wx->fc.high_water = wx_hpbthresh(wx);
	wx->fc.low_water = wx_lpbthresh(wx);

	/* Low water marks must not be larger than high water marks */
	if (wx->fc.low_water > wx->fc.high_water)
		wx->fc.low_water = 0;
}

static void wx_set_ethertype_anti_spoofing(struct wx *wx, bool enable, int vf)
{
	u32 pfvfspoof, reg_offset, vf_shift;

	vf_shift = WX_VF_IND_SHIFT(vf);
	reg_offset = WX_VF_REG_OFFSET(vf);

	pfvfspoof = rd32(wx, WX_TDM_ETYPE_AS(reg_offset));
	if (enable)
		pfvfspoof |= BIT(vf_shift);
	else
		pfvfspoof &= ~BIT(vf_shift);
	wr32(wx, WX_TDM_ETYPE_AS(reg_offset), pfvfspoof);
}

int wx_set_vf_spoofchk(struct net_device *netdev, int vf, bool setting)
{
	u32 index = WX_VF_REG_OFFSET(vf), vf_bit = WX_VF_IND_SHIFT(vf);
	struct wx *wx = netdev_priv(netdev);
	u32 regval;

	if (vf >= wx->num_vfs)
		return -EINVAL;

	wx->vfinfo[vf].spoofchk_enabled = setting;

	regval = (setting << vf_bit);
	wr32m(wx, WX_TDM_MAC_AS(index), regval | BIT(vf_bit), regval);

	if (wx->vfinfo[vf].vlan_count)
		wr32m(wx, WX_TDM_VLAN_AS(index), regval | BIT(vf_bit), regval);

	return 0;
}

static void wx_configure_virtualization(struct wx *wx)
{
	u16 pool = wx->num_rx_pools;
	u32 reg_offset, vf_shift;
	u32 i;

	if (!test_bit(WX_FLAG_SRIOV_ENABLED, wx->flags))
		return;

	wr32m(wx, WX_PSR_VM_CTL,
	      WX_PSR_VM_CTL_POOL_MASK | WX_PSR_VM_CTL_REPLEN,
	      FIELD_PREP(WX_PSR_VM_CTL_POOL_MASK, VMDQ_P(0)) |
	      WX_PSR_VM_CTL_REPLEN);
	while (pool--)
		wr32m(wx, WX_PSR_VM_L2CTL(pool),
		      WX_PSR_VM_L2CTL_AUPE, WX_PSR_VM_L2CTL_AUPE);

	if (!test_bit(WX_FLAG_MULTI_64_FUNC, wx->flags)) {
		vf_shift = BIT(VMDQ_P(0));
		/* Enable only the PF pools for Tx/Rx */
		wr32(wx, WX_RDM_VF_RE(0), vf_shift);
		wr32(wx, WX_TDM_VF_TE(0), vf_shift);
	} else {
		vf_shift = WX_VF_IND_SHIFT(VMDQ_P(0));
		reg_offset = WX_VF_REG_OFFSET(VMDQ_P(0));

		/* Enable only the PF pools for Tx/Rx */
		wr32(wx, WX_RDM_VF_RE(reg_offset), GENMASK(31, vf_shift));
		wr32(wx, WX_RDM_VF_RE(reg_offset ^ 1), reg_offset - 1);
		wr32(wx, WX_TDM_VF_TE(reg_offset), GENMASK(31, vf_shift));
		wr32(wx, WX_TDM_VF_TE(reg_offset ^ 1), reg_offset - 1);
	}

	/* clear VLAN promisc flag so VFTA will be updated if necessary */
	clear_bit(WX_FLAG_VLAN_PROMISC, wx->flags);

	for (i = 0; i < wx->num_vfs; i++) {
		if (!wx->vfinfo[i].spoofchk_enabled)
			wx_set_vf_spoofchk(wx->netdev, i, false);
		/* enable ethertype anti spoofing if hw supports it */
		wx_set_ethertype_anti_spoofing(wx, true, i);
	}
}

static void wx_configure_port(struct wx *wx)
{
	u32 value, i;

	if (!test_bit(WX_FLAG_MULTI_64_FUNC, wx->flags)) {
		value = (wx->num_vfs == 0) ?
			WX_CFG_PORT_CTL_NUM_VT_NONE :
			WX_CFG_PORT_CTL_NUM_VT_8;
	} else {
		if (test_bit(WX_FLAG_VMDQ_ENABLED, wx->flags)) {
			if (wx->ring_feature[RING_F_RSS].indices == 4)
				value = WX_CFG_PORT_CTL_NUM_VT_32;
			else
				value = WX_CFG_PORT_CTL_NUM_VT_64;
		} else {
			value = 0;
		}
	}

	value |= WX_CFG_PORT_CTL_D_VLAN | WX_CFG_PORT_CTL_QINQ;
	wr32m(wx, WX_CFG_PORT_CTL,
	      WX_CFG_PORT_CTL_NUM_VT_MASK |
	      WX_CFG_PORT_CTL_D_VLAN |
	      WX_CFG_PORT_CTL_QINQ,
	      value);

	wr32(wx, WX_CFG_TAG_TPID(0),
	     ETH_P_8021Q | ETH_P_8021AD << 16);
	wx->tpid[0] = ETH_P_8021Q;
	wx->tpid[1] = ETH_P_8021AD;
	for (i = 1; i < 4; i++)
		wr32(wx, WX_CFG_TAG_TPID(i),
		     ETH_P_8021Q | ETH_P_8021Q << 16);
	for (i = 2; i < 8; i++)
		wx->tpid[i] = ETH_P_8021Q;
}

/**
 *  wx_disable_sec_rx_path - Stops the receive data path
 *  @wx: pointer to private structure
 *
 *  Stops the receive data path and waits for the HW to internally empty
 *  the Rx security block
 **/
int wx_disable_sec_rx_path(struct wx *wx)
{
	u32 secrx;

	wr32m(wx, WX_RSC_CTL,
	      WX_RSC_CTL_RX_DIS, WX_RSC_CTL_RX_DIS);

	return read_poll_timeout(rd32, secrx, secrx & WX_RSC_ST_RSEC_RDY,
				 1000, 40000, false, wx, WX_RSC_ST);
}
EXPORT_SYMBOL(wx_disable_sec_rx_path);

/**
 *  wx_enable_sec_rx_path - Enables the receive data path
 *  @wx: pointer to private structure
 *
 *  Enables the receive data path.
 **/
void wx_enable_sec_rx_path(struct wx *wx)
{
	wr32m(wx, WX_RSC_CTL, WX_RSC_CTL_RX_DIS, 0);
	WX_WRITE_FLUSH(wx);
}
EXPORT_SYMBOL(wx_enable_sec_rx_path);

static void wx_vlan_strip_control(struct wx *wx, bool enable)
{
	int i, j;

	for (i = 0; i < wx->num_rx_queues; i++) {
		struct wx_ring *ring = wx->rx_ring[i];

		j = ring->reg_idx;
		wr32m(wx, WX_PX_RR_CFG(j), WX_PX_RR_CFG_VLAN,
		      enable ? WX_PX_RR_CFG_VLAN : 0);
	}
}

static void wx_vlan_promisc_enable(struct wx *wx)
{
	u32 vlnctrl, i, vind, bits, reg_idx;

	vlnctrl = rd32(wx, WX_PSR_VLAN_CTL);
	if (test_bit(WX_FLAG_VMDQ_ENABLED, wx->flags)) {
		/* we need to keep the VLAN filter on in SRIOV */
		vlnctrl |= WX_PSR_VLAN_CTL_VFE;
		wr32(wx, WX_PSR_VLAN_CTL, vlnctrl);
	} else {
		vlnctrl &= ~WX_PSR_VLAN_CTL_VFE;
		wr32(wx, WX_PSR_VLAN_CTL, vlnctrl);
		return;
	}
	/* We are already in VLAN promisc, nothing to do */
	if (test_bit(WX_FLAG_VLAN_PROMISC, wx->flags))
		return;
	/* Set flag so we don't redo unnecessary work */
	set_bit(WX_FLAG_VLAN_PROMISC, wx->flags);
	/* Add PF to all active pools */
	for (i = WX_PSR_VLAN_SWC_ENTRIES; --i;) {
		wr32(wx, WX_PSR_VLAN_SWC_IDX, i);
		vind = WX_VF_IND_SHIFT(VMDQ_P(0));
		reg_idx = WX_VF_REG_OFFSET(VMDQ_P(0));
		bits = rd32(wx, WX_PSR_VLAN_SWC_VM(reg_idx));
		bits |= BIT(vind);
		wr32(wx, WX_PSR_VLAN_SWC_VM(reg_idx), bits);
	}
	/* Set all bits in the VLAN filter table array */
	for (i = 0; i < wx->mac.vft_size; i++)
		wr32(wx, WX_PSR_VLAN_TBL(i), U32_MAX);
}

static void wx_scrub_vfta(struct wx *wx)
{
	u32 i, vid, bits, vfta, vind, vlvf, reg_idx;

	for (i = WX_PSR_VLAN_SWC_ENTRIES; --i;) {
		wr32(wx, WX_PSR_VLAN_SWC_IDX, i);
		vlvf = rd32(wx, WX_PSR_VLAN_SWC_IDX);
		/* pull VLAN ID from VLVF */
		vid = vlvf & ~WX_PSR_VLAN_SWC_VIEN;
		if (vlvf & WX_PSR_VLAN_SWC_VIEN) {
			/* if PF is part of this then continue */
			if (test_bit(vid, wx->active_vlans))
				continue;
		}
		/* remove PF from the pool */
		vind = WX_VF_IND_SHIFT(VMDQ_P(0));
		reg_idx = WX_VF_REG_OFFSET(VMDQ_P(0));
		bits = rd32(wx, WX_PSR_VLAN_SWC_VM(reg_idx));
		bits &= ~BIT(vind);
		wr32(wx, WX_PSR_VLAN_SWC_VM(reg_idx), bits);
	}
	/* extract values from vft_shadow and write back to VFTA */
	for (i = 0; i < wx->mac.vft_size; i++) {
		vfta = wx->mac.vft_shadow[i];
		wr32(wx, WX_PSR_VLAN_TBL(i), vfta);
	}
}

static void wx_vlan_promisc_disable(struct wx *wx)
{
	u32 vlnctrl;

	/* configure vlan filtering */
	vlnctrl = rd32(wx, WX_PSR_VLAN_CTL);
	vlnctrl |= WX_PSR_VLAN_CTL_VFE;
	wr32(wx, WX_PSR_VLAN_CTL, vlnctrl);
	/* We are not in VLAN promisc, nothing to do */
	if (!test_bit(WX_FLAG_VLAN_PROMISC, wx->flags))
		return;
	/* Set flag so we don't redo unnecessary work */
	clear_bit(WX_FLAG_VLAN_PROMISC, wx->flags);
	wx_scrub_vfta(wx);
}

void wx_set_rx_mode(struct net_device *netdev)
{
	struct wx *wx = netdev_priv(netdev);
	netdev_features_t features;
	u32 fctrl, vmolr, vlnctrl;
	int count;

	features = netdev->features;

	/* Check for Promiscuous and All Multicast modes */
	fctrl = rd32(wx, WX_PSR_CTL);
	fctrl &= ~(WX_PSR_CTL_UPE | WX_PSR_CTL_MPE);
	vmolr = rd32(wx, WX_PSR_VM_L2CTL(VMDQ_P(0)));
	vmolr &= ~(WX_PSR_VM_L2CTL_UPE |
		   WX_PSR_VM_L2CTL_MPE |
		   WX_PSR_VM_L2CTL_ROPE |
		   WX_PSR_VM_L2CTL_ROMPE);
	vlnctrl = rd32(wx, WX_PSR_VLAN_CTL);
	vlnctrl &= ~(WX_PSR_VLAN_CTL_VFE | WX_PSR_VLAN_CTL_CFIEN);

	/* set all bits that we expect to always be set */
	fctrl |= WX_PSR_CTL_BAM | WX_PSR_CTL_MFE;
	vmolr |= WX_PSR_VM_L2CTL_BAM |
		 WX_PSR_VM_L2CTL_AUPE |
		 WX_PSR_VM_L2CTL_VACC;
	vlnctrl |= WX_PSR_VLAN_CTL_VFE;

	wx->addr_ctrl.user_set_promisc = false;
	if (netdev->flags & IFF_PROMISC) {
		wx->addr_ctrl.user_set_promisc = true;
		fctrl |= WX_PSR_CTL_UPE | WX_PSR_CTL_MPE;
		/* pf don't want packets routing to vf, so clear UPE */
		vmolr |= WX_PSR_VM_L2CTL_MPE;
		if (test_bit(WX_FLAG_VMDQ_ENABLED, wx->flags) &&
		    test_bit(WX_FLAG_SRIOV_ENABLED, wx->flags))
			vlnctrl |= WX_PSR_VLAN_CTL_VFE;
		features &= ~NETIF_F_HW_VLAN_CTAG_FILTER;
	}

	if (netdev->flags & IFF_ALLMULTI) {
		fctrl |= WX_PSR_CTL_MPE;
		vmolr |= WX_PSR_VM_L2CTL_MPE;
	}

	if (netdev->features & NETIF_F_RXALL) {
		vmolr |= (WX_PSR_VM_L2CTL_UPE | WX_PSR_VM_L2CTL_MPE);
		vlnctrl &= ~WX_PSR_VLAN_CTL_VFE;
		/* receive bad packets */
		wr32m(wx, WX_RSC_CTL,
		      WX_RSC_CTL_SAVE_MAC_ERR,
		      WX_RSC_CTL_SAVE_MAC_ERR);
	} else {
		vmolr |= WX_PSR_VM_L2CTL_ROPE | WX_PSR_VM_L2CTL_ROMPE;
	}

	/* Write addresses to available RAR registers, if there is not
	 * sufficient space to store all the addresses then enable
	 * unicast promiscuous mode
	 */
	count = wx_write_uc_addr_list(netdev, VMDQ_P(0));
	if (count < 0) {
		vmolr &= ~WX_PSR_VM_L2CTL_ROPE;
		vmolr |= WX_PSR_VM_L2CTL_UPE;
	}

	/* Write addresses to the MTA, if the attempt fails
	 * then we should just turn on promiscuous mode so
	 * that we can at least receive multicast traffic
	 */
	count = wx_write_mc_addr_list(netdev);
	if (count < 0) {
		vmolr &= ~WX_PSR_VM_L2CTL_ROMPE;
		vmolr |= WX_PSR_VM_L2CTL_MPE;
	}

	wr32(wx, WX_PSR_VLAN_CTL, vlnctrl);
	wr32(wx, WX_PSR_CTL, fctrl);
	wr32(wx, WX_PSR_VM_L2CTL(VMDQ_P(0)), vmolr);

	if ((features & NETIF_F_HW_VLAN_CTAG_RX) &&
	    (features & NETIF_F_HW_VLAN_STAG_RX))
		wx_vlan_strip_control(wx, true);
	else
		wx_vlan_strip_control(wx, false);

	if (features & NETIF_F_HW_VLAN_CTAG_FILTER)
		wx_vlan_promisc_disable(wx);
	else
		wx_vlan_promisc_enable(wx);
}
EXPORT_SYMBOL(wx_set_rx_mode);

static void wx_set_rx_buffer_len(struct wx *wx)
{
	struct net_device *netdev = wx->netdev;
	u32 mhadd, max_frame;

	max_frame = netdev->mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;
	/* adjust max frame to be at least the size of a standard frame */
	if (max_frame < (ETH_FRAME_LEN + ETH_FCS_LEN))
		max_frame = (ETH_FRAME_LEN + ETH_FCS_LEN);

	mhadd = rd32(wx, WX_PSR_MAX_SZ);
	if (max_frame != mhadd)
		wr32(wx, WX_PSR_MAX_SZ, max_frame);
}

/**
 * wx_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
int wx_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct wx *wx = netdev_priv(netdev);

	WRITE_ONCE(netdev->mtu, new_mtu);
	wx_set_rx_buffer_len(wx);

	return 0;
}
EXPORT_SYMBOL(wx_change_mtu);

/* Disable the specified rx queue */
void wx_disable_rx_queue(struct wx *wx, struct wx_ring *ring)
{
	u8 reg_idx = ring->reg_idx;
	u32 rxdctl;
	int ret;

	/* write value back with RRCFG.EN bit cleared */
	wr32m(wx, WX_PX_RR_CFG(reg_idx),
	      WX_PX_RR_CFG_RR_EN, 0);

	/* the hardware may take up to 100us to really disable the rx queue */
	ret = read_poll_timeout(rd32, rxdctl, !(rxdctl & WX_PX_RR_CFG_RR_EN),
				10, 100, true, wx, WX_PX_RR_CFG(reg_idx));

	if (ret == -ETIMEDOUT) {
		/* Just for information */
		wx_err(wx,
		       "RRCFG.EN on Rx queue %d not cleared within the polling period\n",
		       reg_idx);
	}
}
EXPORT_SYMBOL(wx_disable_rx_queue);

void wx_enable_rx_queue(struct wx *wx, struct wx_ring *ring)
{
	u8 reg_idx = ring->reg_idx;
	u32 rxdctl;
	int ret;

	ret = read_poll_timeout(rd32, rxdctl, rxdctl & WX_PX_RR_CFG_RR_EN,
				1000, 10000, true, wx, WX_PX_RR_CFG(reg_idx));

	if (ret == -ETIMEDOUT) {
		/* Just for information */
		wx_err(wx,
		       "RRCFG.EN on Rx queue %d not set within the polling period\n",
		       reg_idx);
	}
}
EXPORT_SYMBOL(wx_enable_rx_queue);

static void wx_configure_srrctl(struct wx *wx,
				struct wx_ring *rx_ring)
{
	u16 reg_idx = rx_ring->reg_idx;
	u32 srrctl;

	srrctl = rd32(wx, WX_PX_RR_CFG(reg_idx));
	srrctl &= ~(WX_PX_RR_CFG_RR_HDR_SZ |
		    WX_PX_RR_CFG_RR_BUF_SZ |
		    WX_PX_RR_CFG_SPLIT_MODE);
	/* configure header buffer length, needed for RSC */
	srrctl |= WX_RXBUFFER_256 << WX_PX_RR_CFG_BHDRSIZE_SHIFT;

	/* configure the packet buffer length */
	srrctl |= WX_RX_BUFSZ >> WX_PX_RR_CFG_BSIZEPKT_SHIFT;

	wr32(wx, WX_PX_RR_CFG(reg_idx), srrctl);
}

static void wx_configure_tx_ring(struct wx *wx,
				 struct wx_ring *ring)
{
	u32 txdctl = WX_PX_TR_CFG_ENABLE;
	u8 reg_idx = ring->reg_idx;
	u64 tdba = ring->dma;
	int ret;

	/* disable queue to avoid issues while updating state */
	wr32(wx, WX_PX_TR_CFG(reg_idx), WX_PX_TR_CFG_SWFLSH);
	WX_WRITE_FLUSH(wx);

	wr32(wx, WX_PX_TR_BAL(reg_idx), tdba & DMA_BIT_MASK(32));
	wr32(wx, WX_PX_TR_BAH(reg_idx), upper_32_bits(tdba));

	/* reset head and tail pointers */
	wr32(wx, WX_PX_TR_RP(reg_idx), 0);
	wr32(wx, WX_PX_TR_WP(reg_idx), 0);
	ring->tail = wx->hw_addr + WX_PX_TR_WP(reg_idx);

	if (ring->count < WX_MAX_TXD)
		txdctl |= ring->count / 128 << WX_PX_TR_CFG_TR_SIZE_SHIFT;
	txdctl |= 0x20 << WX_PX_TR_CFG_WTHRESH_SHIFT;

	ring->atr_count = 0;
	if (test_bit(WX_FLAG_FDIR_CAPABLE, wx->flags) &&
	    test_bit(WX_FLAG_FDIR_HASH, wx->flags))
		ring->atr_sample_rate = wx->atr_sample_rate;
	else
		ring->atr_sample_rate = 0;

	/* reinitialize tx_buffer_info */
	memset(ring->tx_buffer_info, 0,
	       sizeof(struct wx_tx_buffer) * ring->count);

	/* enable queue */
	wr32(wx, WX_PX_TR_CFG(reg_idx), txdctl);

	/* poll to verify queue is enabled */
	ret = read_poll_timeout(rd32, txdctl, txdctl & WX_PX_TR_CFG_ENABLE,
				1000, 10000, true, wx, WX_PX_TR_CFG(reg_idx));
	if (ret == -ETIMEDOUT)
		wx_err(wx, "Could not enable Tx Queue %d\n", reg_idx);
}

static void wx_configure_rx_ring(struct wx *wx,
				 struct wx_ring *ring)
{
	u16 reg_idx = ring->reg_idx;
	u64 rdba = ring->dma;
	u32 rxdctl;

	/* disable queue to avoid issues while updating state */
	rxdctl = rd32(wx, WX_PX_RR_CFG(reg_idx));
	wx_disable_rx_queue(wx, ring);

	wr32(wx, WX_PX_RR_BAL(reg_idx), rdba & DMA_BIT_MASK(32));
	wr32(wx, WX_PX_RR_BAH(reg_idx), upper_32_bits(rdba));

	if (ring->count == WX_MAX_RXD)
		rxdctl |= 0 << WX_PX_RR_CFG_RR_SIZE_SHIFT;
	else
		rxdctl |= (ring->count / 128) << WX_PX_RR_CFG_RR_SIZE_SHIFT;

	rxdctl |= 0x1 << WX_PX_RR_CFG_RR_THER_SHIFT;
	wr32(wx, WX_PX_RR_CFG(reg_idx), rxdctl);

	/* reset head and tail pointers */
	wr32(wx, WX_PX_RR_RP(reg_idx), 0);
	wr32(wx, WX_PX_RR_WP(reg_idx), 0);
	ring->tail = wx->hw_addr + WX_PX_RR_WP(reg_idx);

	wx_configure_srrctl(wx, ring);

	/* initialize rx_buffer_info */
	memset(ring->rx_buffer_info, 0,
	       sizeof(struct wx_rx_buffer) * ring->count);

	/* reset ntu and ntc to place SW in sync with hardware */
	ring->next_to_clean = 0;
	ring->next_to_use = 0;

	/* enable receive descriptor ring */
	wr32m(wx, WX_PX_RR_CFG(reg_idx),
	      WX_PX_RR_CFG_RR_EN, WX_PX_RR_CFG_RR_EN);

	wx_enable_rx_queue(wx, ring);
	wx_alloc_rx_buffers(ring, wx_desc_unused(ring));
}

/**
 * wx_configure_tx - Configure Transmit Unit after Reset
 * @wx: pointer to private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
static void wx_configure_tx(struct wx *wx)
{
	u32 i;

	/* TDM_CTL.TE must be before Tx queues are enabled */
	wr32m(wx, WX_TDM_CTL,
	      WX_TDM_CTL_TE, WX_TDM_CTL_TE);

	/* Setup the HW Tx Head and Tail descriptor pointers */
	for (i = 0; i < wx->num_tx_queues; i++)
		wx_configure_tx_ring(wx, wx->tx_ring[i]);

	wr32m(wx, WX_TSC_BUF_AE, WX_TSC_BUF_AE_THR, 0x10);

	if (wx->mac.type == wx_mac_em)
		wr32m(wx, WX_TSC_CTL, WX_TSC_CTL_TX_DIS | WX_TSC_CTL_TSEC_DIS, 0x1);

	/* enable mac transmitter */
	wr32m(wx, WX_MAC_TX_CFG,
	      WX_MAC_TX_CFG_TE, WX_MAC_TX_CFG_TE);
}

static void wx_restore_vlan(struct wx *wx)
{
	u16 vid = 1;

	wx_vlan_rx_add_vid(wx->netdev, htons(ETH_P_8021Q), 0);

	for_each_set_bit_from(vid, wx->active_vlans, VLAN_N_VID)
		wx_vlan_rx_add_vid(wx->netdev, htons(ETH_P_8021Q), vid);
}

static void wx_store_reta(struct wx *wx)
{
	u8 *indir_tbl = wx->rss_indir_tbl;
	u32 reta = 0;
	u32 i;

	/* Fill out the redirection table as follows:
	 *  - 8 bit wide entries containing 4 bit RSS index
	 */
	for (i = 0; i < WX_MAX_RETA_ENTRIES; i++) {
		reta |= indir_tbl[i] << (i & 0x3) * 8;
		if ((i & 3) == 3) {
			wr32(wx, WX_RDB_RSSTBL(i >> 2), reta);
			reta = 0;
		}
	}
}

static void wx_setup_reta(struct wx *wx)
{
	u16 rss_i = wx->ring_feature[RING_F_RSS].indices;
	u32 random_key_size = WX_RSS_KEY_SIZE / 4;
	u32 i, j;

	if (test_bit(WX_FLAG_SRIOV_ENABLED, wx->flags)) {
		if (wx->mac.type == wx_mac_em)
			rss_i = 1;
		else
			rss_i = rss_i < 4 ? 4 : rss_i;
	}

	/* Fill out hash function seeds */
	for (i = 0; i < random_key_size; i++)
		wr32(wx, WX_RDB_RSSRK(i), wx->rss_key[i]);

	/* Fill out redirection table */
	memset(wx->rss_indir_tbl, 0, sizeof(wx->rss_indir_tbl));

	for (i = 0, j = 0; i < WX_MAX_RETA_ENTRIES; i++, j++) {
		if (j == rss_i)
			j = 0;

		wx->rss_indir_tbl[i] = j;
	}

	wx_store_reta(wx);
}

#define WX_RDB_RSS_PL_2		FIELD_PREP(GENMASK(31, 29), 1)
#define WX_RDB_RSS_PL_4		FIELD_PREP(GENMASK(31, 29), 2)
static void wx_setup_psrtype(struct wx *wx)
{
	int rss_i = wx->ring_feature[RING_F_RSS].indices;
	u32 psrtype;
	int pool;

	psrtype = WX_RDB_PL_CFG_L4HDR |
		  WX_RDB_PL_CFG_L3HDR |
		  WX_RDB_PL_CFG_L2HDR |
		  WX_RDB_PL_CFG_TUN_OUTL2HDR |
		  WX_RDB_PL_CFG_TUN_TUNHDR;

	if (!test_bit(WX_FLAG_MULTI_64_FUNC, wx->flags)) {
		for_each_set_bit(pool, &wx->fwd_bitmask, 8)
			wr32(wx, WX_RDB_PL_CFG(VMDQ_P(pool)), psrtype);
	} else {
		if (rss_i > 3)
			psrtype |= WX_RDB_RSS_PL_4;
		else if (rss_i > 1)
			psrtype |= WX_RDB_RSS_PL_2;

		for_each_set_bit(pool, &wx->fwd_bitmask, 32)
			wr32(wx, WX_RDB_PL_CFG(VMDQ_P(pool)), psrtype);
	}
}

static void wx_setup_mrqc(struct wx *wx)
{
	u32 rss_field = 0;

	/* VT, and RSS do not coexist at the same time */
	if (test_bit(WX_FLAG_VMDQ_ENABLED, wx->flags))
		return;

	/* Disable indicating checksum in descriptor, enables RSS hash */
	wr32m(wx, WX_PSR_CTL, WX_PSR_CTL_PCSD, WX_PSR_CTL_PCSD);

	/* Perform hash on these packet types */
	rss_field = WX_RDB_RA_CTL_RSS_IPV4 |
		    WX_RDB_RA_CTL_RSS_IPV4_TCP |
		    WX_RDB_RA_CTL_RSS_IPV4_UDP |
		    WX_RDB_RA_CTL_RSS_IPV6 |
		    WX_RDB_RA_CTL_RSS_IPV6_TCP |
		    WX_RDB_RA_CTL_RSS_IPV6_UDP;

	netdev_rss_key_fill(wx->rss_key, sizeof(wx->rss_key));

	wx_setup_reta(wx);

	if (wx->rss_enabled)
		rss_field |= WX_RDB_RA_CTL_RSS_EN;

	wr32(wx, WX_RDB_RA_CTL, rss_field);
}

/**
 * wx_configure_rx - Configure Receive Unit after Reset
 * @wx: pointer to private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
void wx_configure_rx(struct wx *wx)
{
	int ret;
	u32 i;

	wx_disable_rx(wx);
	wx_setup_psrtype(wx);

	/* enable hw crc stripping */
	wr32m(wx, WX_RSC_CTL, WX_RSC_CTL_CRC_STRIP, WX_RSC_CTL_CRC_STRIP);

	if (test_bit(WX_FLAG_RSC_CAPABLE, wx->flags)) {
		u32 psrctl;

		/* RSC Setup */
		psrctl = rd32(wx, WX_PSR_CTL);
		psrctl |= WX_PSR_CTL_RSC_ACK; /* Disable RSC for ACK packets */
		psrctl |= WX_PSR_CTL_RSC_DIS;
		wr32(wx, WX_PSR_CTL, psrctl);
	}

	wx_setup_mrqc(wx);

	/* set_rx_buffer_len must be called before ring initialization */
	wx_set_rx_buffer_len(wx);

	/* Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	for (i = 0; i < wx->num_rx_queues; i++)
		wx_configure_rx_ring(wx, wx->rx_ring[i]);

	/* Enable all receives, disable security engine prior to block traffic */
	ret = wx_disable_sec_rx_path(wx);
	if (ret < 0)
		wx_err(wx, "The register status is abnormal, please check device.");

	wx_enable_rx(wx);
	wx_enable_sec_rx_path(wx);
}
EXPORT_SYMBOL(wx_configure_rx);

static void wx_configure_isb(struct wx *wx)
{
	/* set ISB Address */
	wr32(wx, WX_PX_ISB_ADDR_L, wx->isb_dma & DMA_BIT_MASK(32));
	if (IS_ENABLED(CONFIG_ARCH_DMA_ADDR_T_64BIT))
		wr32(wx, WX_PX_ISB_ADDR_H, upper_32_bits(wx->isb_dma));
}

void wx_configure(struct wx *wx)
{
	wx_set_rxpba(wx);
	wx_pbthresh_setup(wx);
	wx_configure_virtualization(wx);
	wx_configure_port(wx);

	wx_set_rx_mode(wx->netdev);
	wx_restore_vlan(wx);

	if (test_bit(WX_FLAG_FDIR_CAPABLE, wx->flags))
		wx->configure_fdir(wx);

	wx_configure_tx(wx);
	wx_configure_rx(wx);
	wx_configure_isb(wx);
}
EXPORT_SYMBOL(wx_configure);

/**
 *  wx_disable_pcie_master - Disable PCI-express master access
 *  @wx: pointer to hardware structure
 *
 *  Disables PCI-Express master access and verifies there are no pending
 *  requests.
 **/
int wx_disable_pcie_master(struct wx *wx)
{
	int status = 0;
	u32 val;

	/* Always set this bit to ensure any future transactions are blocked */
	pci_clear_master(wx->pdev);

	/* Exit if master requests are blocked */
	if (!(rd32(wx, WX_PX_TRANSACTION_PENDING)))
		return 0;

	/* Poll for master request bit to clear */
	status = read_poll_timeout(rd32, val, !val, 100, WX_PCI_MASTER_DISABLE_TIMEOUT,
				   false, wx, WX_PX_TRANSACTION_PENDING);
	if (status < 0)
		wx_err(wx, "PCIe transaction pending bit did not clear.\n");

	return status;
}
EXPORT_SYMBOL(wx_disable_pcie_master);

/**
 *  wx_stop_adapter - Generic stop Tx/Rx units
 *  @wx: pointer to hardware structure
 *
 *  Sets the adapter_stopped flag within wx_hw struct. Clears interrupts,
 *  disables transmit and receive units. The adapter_stopped flag is used by
 *  the shared code and drivers to determine if the adapter is in a stopped
 *  state and should not touch the hardware.
 **/
int wx_stop_adapter(struct wx *wx)
{
	u16 i;

	/* Set the adapter_stopped flag so other driver functions stop touching
	 * the hardware
	 */
	wx->adapter_stopped = true;

	/* Disable the receive unit */
	wx_disable_rx(wx);

	/* Set interrupt mask to stop interrupts from being generated */
	wx_intr_disable(wx, WX_INTR_ALL);

	/* Clear any pending interrupts, flush previous writes */
	wr32(wx, WX_PX_MISC_IC, 0xffffffff);
	wr32(wx, WX_BME_CTL, 0x3);

	/* Disable the transmit unit.  Each queue must be disabled. */
	for (i = 0; i < wx->mac.max_tx_queues; i++) {
		wr32m(wx, WX_PX_TR_CFG(i),
		      WX_PX_TR_CFG_SWFLSH | WX_PX_TR_CFG_ENABLE,
		      WX_PX_TR_CFG_SWFLSH);
	}

	/* Disable the receive unit by stopping each queue */
	for (i = 0; i < wx->mac.max_rx_queues; i++) {
		wr32m(wx, WX_PX_RR_CFG(i),
		      WX_PX_RR_CFG_RR_EN, 0);
	}

	/* flush all queues disables */
	WX_WRITE_FLUSH(wx);

	/* Prevent the PCI-E bus from hanging by disabling PCI-E master
	 * access and verify no pending requests
	 */
	return wx_disable_pcie_master(wx);
}
EXPORT_SYMBOL(wx_stop_adapter);

void wx_reset_mac(struct wx *wx)
{
	/* receive packets that size > 2048 */
	wr32m(wx, WX_MAC_RX_CFG, WX_MAC_RX_CFG_JE, WX_MAC_RX_CFG_JE);

	/* clear counters on read */
	wr32m(wx, WX_MMC_CONTROL,
	      WX_MMC_CONTROL_RSTONRD, WX_MMC_CONTROL_RSTONRD);

	wr32m(wx, WX_MAC_RX_FLOW_CTRL,
	      WX_MAC_RX_FLOW_CTRL_RFE, WX_MAC_RX_FLOW_CTRL_RFE);

	wr32(wx, WX_MAC_PKT_FLT, WX_MAC_PKT_FLT_PR);
}
EXPORT_SYMBOL(wx_reset_mac);

void wx_reset_misc(struct wx *wx)
{
	int i;

	wx_reset_mac(wx);

	wr32m(wx, WX_MIS_RST_ST,
	      WX_MIS_RST_ST_RST_INIT, 0x1E00);

	/* errata 4: initialize mng flex tbl and wakeup flex tbl*/
	wr32(wx, WX_PSR_MNG_FLEX_SEL, 0);
	for (i = 0; i < 16; i++) {
		wr32(wx, WX_PSR_MNG_FLEX_DW_L(i), 0);
		wr32(wx, WX_PSR_MNG_FLEX_DW_H(i), 0);
		wr32(wx, WX_PSR_MNG_FLEX_MSK(i), 0);
	}
	wr32(wx, WX_PSR_LAN_FLEX_SEL, 0);
	for (i = 0; i < 16; i++) {
		wr32(wx, WX_PSR_LAN_FLEX_DW_L(i), 0);
		wr32(wx, WX_PSR_LAN_FLEX_DW_H(i), 0);
		wr32(wx, WX_PSR_LAN_FLEX_MSK(i), 0);
	}

	/* set pause frame dst mac addr */
	wr32(wx, WX_RDB_PFCMACDAL, 0xC2000001);
	wr32(wx, WX_RDB_PFCMACDAH, 0x0180);
}
EXPORT_SYMBOL(wx_reset_misc);

/**
 *  wx_get_pcie_msix_counts - Gets MSI-X vector count
 *  @wx: pointer to hardware structure
 *  @msix_count: number of MSI interrupts that can be obtained
 *  @max_msix_count: number of MSI interrupts that mac need
 *
 *  Read PCIe configuration space, and get the MSI-X vector count from
 *  the capabilities table.
 **/
int wx_get_pcie_msix_counts(struct wx *wx, u16 *msix_count, u16 max_msix_count)
{
	struct pci_dev *pdev = wx->pdev;
	struct device *dev = &pdev->dev;
	int pos;

	*msix_count = 1;
	pos = pci_find_capability(pdev, PCI_CAP_ID_MSIX);
	if (!pos) {
		dev_err(dev, "Unable to find MSI-X Capabilities\n");
		return -EINVAL;
	}
	pci_read_config_word(pdev,
			     pos + PCI_MSIX_FLAGS,
			     msix_count);
	*msix_count &= WX_PCIE_MSIX_TBL_SZ_MASK;
	/* MSI-X count is zero-based in HW */
	*msix_count += 1;

	if (*msix_count > max_msix_count)
		*msix_count = max_msix_count;

	return 0;
}
EXPORT_SYMBOL(wx_get_pcie_msix_counts);

/**
 * wx_init_rss_key - Initialize wx RSS key
 * @wx: device handle
 *
 * Allocates and initializes the RSS key if it is not allocated.
 **/
static int wx_init_rss_key(struct wx *wx)
{
	u32 *rss_key;

	if (!wx->rss_key) {
		rss_key = kzalloc(WX_RSS_KEY_SIZE, GFP_KERNEL);
		if (unlikely(!rss_key))
			return -ENOMEM;

		netdev_rss_key_fill(rss_key, WX_RSS_KEY_SIZE);
		wx->rss_key = rss_key;
	}

	return 0;
}

int wx_sw_init(struct wx *wx)
{
	struct pci_dev *pdev = wx->pdev;
	u32 ssid = 0;
	int err = 0;

	wx->vendor_id = pdev->vendor;
	wx->device_id = pdev->device;
	wx->revision_id = pdev->revision;
	wx->oem_svid = pdev->subsystem_vendor;
	wx->oem_ssid = pdev->subsystem_device;
	wx->bus.device = PCI_SLOT(pdev->devfn);
	wx->bus.func = PCI_FUNC(pdev->devfn);

	if (wx->oem_svid == PCI_VENDOR_ID_WANGXUN ||
	    pdev->is_virtfn) {
		wx->subsystem_vendor_id = pdev->subsystem_vendor;
		wx->subsystem_device_id = pdev->subsystem_device;
	} else {
		err = wx_flash_read_dword(wx, 0xfffdc, &ssid);
		if (err < 0) {
			wx_err(wx, "read of internal subsystem device id failed\n");
			return err;
		}

		wx->subsystem_device_id = swab16((u16)ssid);
	}

	err = wx_init_rss_key(wx);
	if (err < 0) {
		wx_err(wx, "rss key allocation failed\n");
		return err;
	}

	wx->mac_table = kcalloc(wx->mac.num_rar_entries,
				sizeof(struct wx_mac_addr),
				GFP_KERNEL);
	if (!wx->mac_table) {
		wx_err(wx, "mac_table allocation failed\n");
		kfree(wx->rss_key);
		return -ENOMEM;
	}

	bitmap_zero(wx->state, WX_STATE_NBITS);
	bitmap_zero(wx->flags, WX_PF_FLAGS_NBITS);
	wx->misc_irq_domain = false;

	return 0;
}
EXPORT_SYMBOL(wx_sw_init);

/**
 *  wx_find_vlvf_slot - find the vlanid or the first empty slot
 *  @wx: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *
 *  return the VLVF index where this VLAN id should be placed
 *
 **/
static int wx_find_vlvf_slot(struct wx *wx, u32 vlan)
{
	u32 bits = 0, first_empty_slot = 0;
	int regindex;

	/* short cut the special case */
	if (vlan == 0)
		return 0;

	/* Search for the vlan id in the VLVF entries. Save off the first empty
	 * slot found along the way
	 */
	for (regindex = 1; regindex < WX_PSR_VLAN_SWC_ENTRIES; regindex++) {
		wr32(wx, WX_PSR_VLAN_SWC_IDX, regindex);
		bits = rd32(wx, WX_PSR_VLAN_SWC);
		if (!bits && !(first_empty_slot))
			first_empty_slot = regindex;
		else if ((bits & 0x0FFF) == vlan)
			break;
	}

	if (regindex >= WX_PSR_VLAN_SWC_ENTRIES) {
		if (first_empty_slot)
			regindex = first_empty_slot;
		else
			regindex = -ENOMEM;
	}

	return regindex;
}

/**
 *  wx_set_vlvf - Set VLAN Pool Filter
 *  @wx: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vind: VMDq output index that maps queue to VLAN id in VFVFB
 *  @vlan_on: boolean flag to turn on/off VLAN in VFVF
 *  @vfta_changed: pointer to boolean flag which indicates whether VFTA
 *                 should be changed
 *
 *  Turn on/off specified bit in VLVF table.
 **/
static int wx_set_vlvf(struct wx *wx, u32 vlan, u32 vind, bool vlan_on,
		       bool *vfta_changed)
{
	int vlvf_index;
	u32 vt, bits;

	/* If VT Mode is set
	 *   Either vlan_on
	 *     make sure the vlan is in VLVF
	 *     set the vind bit in the matching VLVFB
	 *   Or !vlan_on
	 *     clear the pool bit and possibly the vind
	 */
	vt = rd32(wx, WX_CFG_PORT_CTL);
	if (!(vt & WX_CFG_PORT_CTL_NUM_VT_MASK))
		return 0;

	vlvf_index = wx_find_vlvf_slot(wx, vlan);
	if (vlvf_index < 0)
		return vlvf_index;

	wr32(wx, WX_PSR_VLAN_SWC_IDX, vlvf_index);
	if (vlan_on) {
		/* set the pool bit */
		if (vind < 32) {
			bits = rd32(wx, WX_PSR_VLAN_SWC_VM_L);
			bits |= (1 << vind);
			wr32(wx, WX_PSR_VLAN_SWC_VM_L, bits);
		} else {
			bits = rd32(wx, WX_PSR_VLAN_SWC_VM_H);
			bits |= (1 << (vind - 32));
			wr32(wx, WX_PSR_VLAN_SWC_VM_H, bits);
		}
	} else {
		/* clear the pool bit */
		if (vind < 32) {
			bits = rd32(wx, WX_PSR_VLAN_SWC_VM_L);
			bits &= ~(1 << vind);
			wr32(wx, WX_PSR_VLAN_SWC_VM_L, bits);
			bits |= rd32(wx, WX_PSR_VLAN_SWC_VM_H);
		} else {
			bits = rd32(wx, WX_PSR_VLAN_SWC_VM_H);
			bits &= ~(1 << (vind - 32));
			wr32(wx, WX_PSR_VLAN_SWC_VM_H, bits);
			bits |= rd32(wx, WX_PSR_VLAN_SWC_VM_L);
		}
	}

	if (bits) {
		wr32(wx, WX_PSR_VLAN_SWC, (WX_PSR_VLAN_SWC_VIEN | vlan));
		if (!vlan_on && vfta_changed)
			*vfta_changed = false;
	} else {
		wr32(wx, WX_PSR_VLAN_SWC, 0);
	}

	return 0;
}

/**
 *  wx_set_vfta - Set VLAN filter table
 *  @wx: pointer to hardware structure
 *  @vlan: VLAN id to write to VLAN filter
 *  @vind: VMDq output index that maps queue to VLAN id in VFVFB
 *  @vlan_on: boolean flag to turn on/off VLAN in VFVF
 *
 *  Turn on/off specified VLAN in the VLAN filter table.
 **/
int wx_set_vfta(struct wx *wx, u32 vlan, u32 vind, bool vlan_on)
{
	u32 bitindex, vfta, targetbit;
	bool vfta_changed = false;
	int regindex, ret;

	/* this is a 2 part operation - first the VFTA, then the
	 * VLVF and VLVFB if VT Mode is set
	 * We don't write the VFTA until we know the VLVF part succeeded.
	 */

	/* Part 1
	 * The VFTA is a bitstring made up of 128 32-bit registers
	 * that enable the particular VLAN id, much like the MTA:
	 *    bits[11-5]: which register
	 *    bits[4-0]:  which bit in the register
	 */
	regindex = (vlan >> 5) & 0x7F;
	bitindex = vlan & 0x1F;
	targetbit = (1 << bitindex);
	/* errata 5 */
	vfta = wx->mac.vft_shadow[regindex];
	if (vlan_on) {
		if (!(vfta & targetbit)) {
			vfta |= targetbit;
			vfta_changed = true;
		}
	} else {
		if ((vfta & targetbit)) {
			vfta &= ~targetbit;
			vfta_changed = true;
		}
	}
	/* Part 2
	 * Call wx_set_vlvf to set VLVFB and VLVF
	 */
	ret = wx_set_vlvf(wx, vlan, vind, vlan_on, &vfta_changed);
	if (ret != 0)
		return ret;

	if (vfta_changed)
		wr32(wx, WX_PSR_VLAN_TBL(regindex), vfta);
	wx->mac.vft_shadow[regindex] = vfta;

	return 0;
}

/**
 *  wx_clear_vfta - Clear VLAN filter table
 *  @wx: pointer to hardware structure
 *
 *  Clears the VLAN filer table, and the VMDq index associated with the filter
 **/
static void wx_clear_vfta(struct wx *wx)
{
	u32 offset;

	for (offset = 0; offset < wx->mac.vft_size; offset++) {
		wr32(wx, WX_PSR_VLAN_TBL(offset), 0);
		wx->mac.vft_shadow[offset] = 0;
	}

	for (offset = 0; offset < WX_PSR_VLAN_SWC_ENTRIES; offset++) {
		wr32(wx, WX_PSR_VLAN_SWC_IDX, offset);
		wr32(wx, WX_PSR_VLAN_SWC, 0);
		wr32(wx, WX_PSR_VLAN_SWC_VM_L, 0);
		wr32(wx, WX_PSR_VLAN_SWC_VM_H, 0);
	}
}

int wx_vlan_rx_add_vid(struct net_device *netdev,
		       __be16 proto, u16 vid)
{
	struct wx *wx = netdev_priv(netdev);

	/* add VID to filter table */
	wx_set_vfta(wx, vid, VMDQ_P(0), true);
	set_bit(vid, wx->active_vlans);

	return 0;
}
EXPORT_SYMBOL(wx_vlan_rx_add_vid);

int wx_vlan_rx_kill_vid(struct net_device *netdev, __be16 proto, u16 vid)
{
	struct wx *wx = netdev_priv(netdev);

	/* remove VID from filter table */
	if (vid)
		wx_set_vfta(wx, vid, VMDQ_P(0), false);
	clear_bit(vid, wx->active_vlans);

	return 0;
}
EXPORT_SYMBOL(wx_vlan_rx_kill_vid);

static void wx_enable_rx_drop(struct wx *wx, struct wx_ring *ring)
{
	u16 reg_idx = ring->reg_idx;
	u32 srrctl;

	srrctl = rd32(wx, WX_PX_RR_CFG(reg_idx));
	srrctl |= WX_PX_RR_CFG_DROP_EN;

	wr32(wx, WX_PX_RR_CFG(reg_idx), srrctl);
}

static void wx_disable_rx_drop(struct wx *wx, struct wx_ring *ring)
{
	u16 reg_idx = ring->reg_idx;
	u32 srrctl;

	srrctl = rd32(wx, WX_PX_RR_CFG(reg_idx));
	srrctl &= ~WX_PX_RR_CFG_DROP_EN;

	wr32(wx, WX_PX_RR_CFG(reg_idx), srrctl);
}

int wx_fc_enable(struct wx *wx, bool tx_pause, bool rx_pause)
{
	u16 pause_time = WX_DEFAULT_FCPAUSE;
	u32 mflcn_reg, fccfg_reg, reg;
	u32 fcrtl, fcrth;
	int i;

	/* Low water mark of zero causes XOFF floods */
	if (tx_pause && wx->fc.high_water) {
		if (!wx->fc.low_water || wx->fc.low_water >= wx->fc.high_water) {
			wx_err(wx, "Invalid water mark configuration\n");
			return -EINVAL;
		}
	}

	/* Disable any previous flow control settings */
	mflcn_reg = rd32(wx, WX_MAC_RX_FLOW_CTRL);
	mflcn_reg &= ~WX_MAC_RX_FLOW_CTRL_RFE;

	fccfg_reg = rd32(wx, WX_RDB_RFCC);
	fccfg_reg &= ~WX_RDB_RFCC_RFCE_802_3X;

	if (rx_pause)
		mflcn_reg |= WX_MAC_RX_FLOW_CTRL_RFE;
	if (tx_pause)
		fccfg_reg |= WX_RDB_RFCC_RFCE_802_3X;

	/* Set 802.3x based flow control settings. */
	wr32(wx, WX_MAC_RX_FLOW_CTRL, mflcn_reg);
	wr32(wx, WX_RDB_RFCC, fccfg_reg);

	/* Set up and enable Rx high/low water mark thresholds, enable XON. */
	if (tx_pause && wx->fc.high_water) {
		fcrtl = (wx->fc.low_water << 10) | WX_RDB_RFCL_XONE;
		wr32(wx, WX_RDB_RFCL, fcrtl);
		fcrth = (wx->fc.high_water << 10) | WX_RDB_RFCH_XOFFE;
	} else {
		wr32(wx, WX_RDB_RFCL, 0);
		/* In order to prevent Tx hangs when the internal Tx
		 * switch is enabled we must set the high water mark
		 * to the Rx packet buffer size - 24KB.  This allows
		 * the Tx switch to function even under heavy Rx
		 * workloads.
		 */
		fcrth = rd32(wx, WX_RDB_PB_SZ(0)) - 24576;
	}

	wr32(wx, WX_RDB_RFCH, fcrth);

	/* Configure pause time */
	reg = pause_time * 0x00010001;
	wr32(wx, WX_RDB_RFCV, reg);

	/* Configure flow control refresh threshold value */
	wr32(wx, WX_RDB_RFCRT, pause_time / 2);

	/*  We should set the drop enable bit if:
	 *  Number of Rx queues > 1 and flow control is disabled
	 *
	 *  This allows us to avoid head of line blocking for security
	 *  and performance reasons.
	 */
	if (wx->num_rx_queues > 1 && !tx_pause) {
		for (i = 0; i < wx->num_rx_queues; i++)
			wx_enable_rx_drop(wx, wx->rx_ring[i]);
	} else {
		for (i = 0; i < wx->num_rx_queues; i++)
			wx_disable_rx_drop(wx, wx->rx_ring[i]);
	}

	return 0;
}
EXPORT_SYMBOL(wx_fc_enable);

/**
 * wx_update_stats - Update the board statistics counters.
 * @wx: board private structure
 **/
void wx_update_stats(struct wx *wx)
{
	struct wx_hw_stats *hwstats = &wx->stats;

	u64 non_eop_descs = 0, alloc_rx_buff_failed = 0;
	u64 hw_csum_rx_good = 0, hw_csum_rx_error = 0;
	u64 restart_queue = 0, tx_busy = 0;
	u32 i;

	/* gather some stats to the wx struct that are per queue */
	for (i = 0; i < wx->num_rx_queues; i++) {
		struct wx_ring *rx_ring = wx->rx_ring[i];

		non_eop_descs += rx_ring->rx_stats.non_eop_descs;
		alloc_rx_buff_failed += rx_ring->rx_stats.alloc_rx_buff_failed;
		hw_csum_rx_good += rx_ring->rx_stats.csum_good_cnt;
		hw_csum_rx_error += rx_ring->rx_stats.csum_err;
	}
	wx->non_eop_descs = non_eop_descs;
	wx->alloc_rx_buff_failed = alloc_rx_buff_failed;
	wx->hw_csum_rx_error = hw_csum_rx_error;
	wx->hw_csum_rx_good = hw_csum_rx_good;

	for (i = 0; i < wx->num_tx_queues; i++) {
		struct wx_ring *tx_ring = wx->tx_ring[i];

		restart_queue += tx_ring->tx_stats.restart_queue;
		tx_busy += tx_ring->tx_stats.tx_busy;
	}
	wx->restart_queue = restart_queue;
	wx->tx_busy = tx_busy;

	hwstats->gprc += rd32(wx, WX_RDM_PKT_CNT);
	hwstats->gptc += rd32(wx, WX_TDM_PKT_CNT);
	hwstats->gorc += rd64(wx, WX_RDM_BYTE_CNT_LSB);
	hwstats->gotc += rd64(wx, WX_TDM_BYTE_CNT_LSB);
	hwstats->tpr += rd64(wx, WX_RX_FRAME_CNT_GOOD_BAD_L);
	hwstats->tpt += rd64(wx, WX_TX_FRAME_CNT_GOOD_BAD_L);
	hwstats->crcerrs += rd64(wx, WX_RX_CRC_ERROR_FRAMES_L);
	hwstats->rlec += rd64(wx, WX_RX_LEN_ERROR_FRAMES_L);
	hwstats->bprc += rd64(wx, WX_RX_BC_FRAMES_GOOD_L);
	hwstats->bptc += rd64(wx, WX_TX_BC_FRAMES_GOOD_L);
	hwstats->mprc += rd64(wx, WX_RX_MC_FRAMES_GOOD_L);
	hwstats->mptc += rd64(wx, WX_TX_MC_FRAMES_GOOD_L);
	hwstats->roc += rd32(wx, WX_RX_OVERSIZE_FRAMES_GOOD);
	hwstats->ruc += rd32(wx, WX_RX_UNDERSIZE_FRAMES_GOOD);
	hwstats->lxonoffrxc += rd32(wx, WX_MAC_LXONOFFRXC);
	hwstats->lxontxc += rd32(wx, WX_RDB_LXONTXC);
	hwstats->lxofftxc += rd32(wx, WX_RDB_LXOFFTXC);
	hwstats->o2bgptc += rd32(wx, WX_TDM_OS2BMC_CNT);
	hwstats->b2ospc += rd32(wx, WX_MNG_BMC2OS_CNT);
	hwstats->o2bspc += rd32(wx, WX_MNG_OS2BMC_CNT);
	hwstats->b2ogprc += rd32(wx, WX_RDM_BMC2OS_CNT);
	hwstats->rdmdrop += rd32(wx, WX_RDM_DRP_PKT);

	if (test_bit(WX_FLAG_FDIR_CAPABLE, wx->flags)) {
		hwstats->fdirmatch += rd32(wx, WX_RDB_FDIR_MATCH);
		hwstats->fdirmiss += rd32(wx, WX_RDB_FDIR_MISS);
	}

	/* qmprc is not cleared on read, manual reset it */
	hwstats->qmprc = 0;
	for (i = wx->num_vfs * wx->num_rx_queues_per_pool;
	     i < wx->mac.max_rx_queues; i++)
		hwstats->qmprc += rd32(wx, WX_PX_MPRC(i));
}
EXPORT_SYMBOL(wx_update_stats);

/**
 *  wx_clear_hw_cntrs - Generic clear hardware counters
 *  @wx: board private structure
 *
 *  Clears all hardware statistics counters by reading them from the hardware
 *  Statistics counters are clear on read.
 **/
void wx_clear_hw_cntrs(struct wx *wx)
{
	u16 i = 0;

	for (i = 0; i < wx->mac.max_rx_queues; i++)
		wr32(wx, WX_PX_MPRC(i), 0);

	rd32(wx, WX_RDM_PKT_CNT);
	rd32(wx, WX_TDM_PKT_CNT);
	rd64(wx, WX_RDM_BYTE_CNT_LSB);
	rd32(wx, WX_TDM_BYTE_CNT_LSB);
	rd32(wx, WX_RDM_DRP_PKT);
	rd32(wx, WX_RX_UNDERSIZE_FRAMES_GOOD);
	rd32(wx, WX_RX_OVERSIZE_FRAMES_GOOD);
	rd64(wx, WX_RX_FRAME_CNT_GOOD_BAD_L);
	rd64(wx, WX_TX_FRAME_CNT_GOOD_BAD_L);
	rd64(wx, WX_RX_MC_FRAMES_GOOD_L);
	rd64(wx, WX_TX_MC_FRAMES_GOOD_L);
	rd64(wx, WX_RX_BC_FRAMES_GOOD_L);
	rd64(wx, WX_TX_BC_FRAMES_GOOD_L);
	rd64(wx, WX_RX_CRC_ERROR_FRAMES_L);
	rd64(wx, WX_RX_LEN_ERROR_FRAMES_L);
	rd32(wx, WX_RDB_LXONTXC);
	rd32(wx, WX_RDB_LXOFFTXC);
	rd32(wx, WX_MAC_LXONOFFRXC);
}
EXPORT_SYMBOL(wx_clear_hw_cntrs);

/**
 *  wx_start_hw - Prepare hardware for Tx/Rx
 *  @wx: pointer to hardware structure
 *
 *  Starts the hardware using the generic start_hw function
 *  and the generation start_hw function.
 *  Then performs revision-specific operations, if any.
 **/
void wx_start_hw(struct wx *wx)
{
	int i;

	/* Clear the VLAN filter table */
	wx_clear_vfta(wx);
	WX_WRITE_FLUSH(wx);
	/* Clear the rate limiters */
	for (i = 0; i < wx->mac.max_tx_queues; i++) {
		wr32(wx, WX_TDM_RP_IDX, i);
		wr32(wx, WX_TDM_RP_RATE, 0);
	}
}
EXPORT_SYMBOL(wx_start_hw);

MODULE_LICENSE("GPL");
