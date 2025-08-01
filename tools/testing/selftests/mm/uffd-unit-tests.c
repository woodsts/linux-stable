// SPDX-License-Identifier: GPL-2.0-only
/*
 * Userfaultfd unit tests.
 *
 *  Copyright (C) 2015-2023  Red Hat, Inc.
 */

#include "uffd-common.h"

#include "../../../../mm/gup_test.h"

#ifdef __NR_userfaultfd

/* The unit test doesn't need a large or random size, make it 32MB for now */
#define  UFFD_TEST_MEM_SIZE               (32UL << 20)

#define  MEM_ANON                         BIT_ULL(0)
#define  MEM_SHMEM                        BIT_ULL(1)
#define  MEM_SHMEM_PRIVATE                BIT_ULL(2)
#define  MEM_HUGETLB                      BIT_ULL(3)
#define  MEM_HUGETLB_PRIVATE              BIT_ULL(4)

#define  MEM_ALL  (MEM_ANON | MEM_SHMEM | MEM_SHMEM_PRIVATE | \
		   MEM_HUGETLB | MEM_HUGETLB_PRIVATE)

#define ALIGN_UP(x, align_to) \
	((__typeof__(x))((((unsigned long)(x)) + ((align_to)-1)) & ~((align_to)-1)))

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

struct mem_type {
	const char *name;
	unsigned int mem_flag;
	uffd_test_ops_t *mem_ops;
	bool shared;
};
typedef struct mem_type mem_type_t;

mem_type_t mem_types[] = {
	{
		.name = "anon",
		.mem_flag = MEM_ANON,
		.mem_ops = &anon_uffd_test_ops,
		.shared = false,
	},
	{
		.name = "shmem",
		.mem_flag = MEM_SHMEM,
		.mem_ops = &shmem_uffd_test_ops,
		.shared = true,
	},
	{
		.name = "shmem-private",
		.mem_flag = MEM_SHMEM_PRIVATE,
		.mem_ops = &shmem_uffd_test_ops,
		.shared = false,
	},
	{
		.name = "hugetlb",
		.mem_flag = MEM_HUGETLB,
		.mem_ops = &hugetlb_uffd_test_ops,
		.shared = true,
	},
	{
		.name = "hugetlb-private",
		.mem_flag = MEM_HUGETLB_PRIVATE,
		.mem_ops = &hugetlb_uffd_test_ops,
		.shared = false,
	},
};

/* Arguments to be passed over to each uffd unit test */
struct uffd_test_args {
	mem_type_t *mem_type;
};
typedef struct uffd_test_args uffd_test_args_t;

/* Returns: UFFD_TEST_* */
typedef void (*uffd_test_fn)(uffd_test_args_t *);

typedef struct {
	const char *name;
	uffd_test_fn uffd_fn;
	unsigned int mem_targets;
	uint64_t uffd_feature_required;
	uffd_test_case_ops_t *test_case_ops;
} uffd_test_case_t;

static void uffd_test_report(void)
{
	printf("Userfaults unit tests: pass=%u, skip=%u, fail=%u (total=%u)\n",
	       ksft_get_pass_cnt(),
	       ksft_get_xskip_cnt(),
	       ksft_get_fail_cnt(),
	       ksft_test_num());
}

static void uffd_test_pass(void)
{
	printf("done\n");
	ksft_inc_pass_cnt();
}

#define  uffd_test_start(...)  do {		\
		printf("Testing ");		\
		printf(__VA_ARGS__);		\
		printf("... ");			\
		fflush(stdout);			\
	} while (0)

#define  uffd_test_fail(...)  do {		\
		printf("failed [reason: ");	\
		printf(__VA_ARGS__);		\
		printf("]\n");			\
		ksft_inc_fail_cnt();		\
	} while (0)

static void uffd_test_skip(const char *message)
{
	printf("skipped [reason: %s]\n", message);
	ksft_inc_xskip_cnt();
}

/*
 * Returns 1 if specific userfaultfd supported, 0 otherwise.  Note, we'll
 * return 1 even if some test failed as long as uffd supported, because in
 * that case we still want to proceed with the rest uffd unit tests.
 */
static int test_uffd_api(bool use_dev)
{
	struct uffdio_api uffdio_api;
	int uffd;

	uffd_test_start("UFFDIO_API (with %s)",
			use_dev ? "/dev/userfaultfd" : "syscall");

	if (use_dev)
		uffd = uffd_open_dev(UFFD_FLAGS);
	else
		uffd = uffd_open_sys(UFFD_FLAGS);
	if (uffd < 0) {
		uffd_test_skip("cannot open userfaultfd handle");
		return 0;
	}

	/* Test wrong UFFD_API */
	uffdio_api.api = 0xab;
	uffdio_api.features = 0;
	if (ioctl(uffd, UFFDIO_API, &uffdio_api) == 0) {
		uffd_test_fail("UFFDIO_API should fail with wrong api but didn't");
		goto out;
	}

	/* Test wrong feature bit */
	uffdio_api.api = UFFD_API;
	uffdio_api.features = BIT_ULL(63);
	if (ioctl(uffd, UFFDIO_API, &uffdio_api) == 0) {
		uffd_test_fail("UFFDIO_API should fail with wrong feature but didn't");
		goto out;
	}

	/* Test normal UFFDIO_API */
	uffdio_api.api = UFFD_API;
	uffdio_api.features = 0;
	if (ioctl(uffd, UFFDIO_API, &uffdio_api)) {
		uffd_test_fail("UFFDIO_API should succeed but failed");
		goto out;
	}

	/* Test double requests of UFFDIO_API with a random feature set */
	uffdio_api.features = BIT_ULL(0);
	if (ioctl(uffd, UFFDIO_API, &uffdio_api) == 0) {
		uffd_test_fail("UFFDIO_API should reject initialized uffd");
		goto out;
	}

	uffd_test_pass();
out:
	close(uffd);
	/* We have a valid uffd handle */
	return 1;
}

/*
 * This function initializes the global variables.  TODO: remove global
 * vars and then remove this.
 */
static int
uffd_setup_environment(uffd_test_args_t *args, uffd_test_case_t *test,
		       mem_type_t *mem_type, const char **errmsg)
{
	map_shared = mem_type->shared;
	uffd_test_ops = mem_type->mem_ops;
	uffd_test_case_ops = test->test_case_ops;

	if (mem_type->mem_flag & (MEM_HUGETLB_PRIVATE | MEM_HUGETLB))
		page_size = default_huge_page_size();
	else
		page_size = psize();

	/* Ensure we have at least 2 pages */
	nr_pages = MAX(UFFD_TEST_MEM_SIZE, page_size * 2) / page_size;
	/* TODO: remove this global var.. it's so ugly */
	nr_parallel = 1;

	/* Initialize test arguments */
	args->mem_type = mem_type;

	return uffd_test_ctx_init(test->uffd_feature_required, errmsg);
}

static bool uffd_feature_supported(uffd_test_case_t *test)
{
	uint64_t features;

	if (uffd_get_features(&features))
		return false;

	return (features & test->uffd_feature_required) ==
	    test->uffd_feature_required;
}

static int pagemap_open(void)
{
	int fd = open("/proc/self/pagemap", O_RDONLY);

	if (fd < 0)
		err("open pagemap");

	return fd;
}

/* This macro let __LINE__ works in err() */
#define  pagemap_check_wp(value, wp) do {				\
		if (!!(value & PM_UFFD_WP) != wp)			\
			err("pagemap uffd-wp bit error: 0x%"PRIx64, value); \
	} while (0)

typedef struct {
	int parent_uffd, child_uffd;
} fork_event_args;

static void *fork_event_consumer(void *data)
{
	fork_event_args *args = data;
	struct uffd_msg msg = { 0 };

	ready_for_fork = true;

	/* Read until a full msg received */
	while (uffd_read_msg(args->parent_uffd, &msg));

	if (msg.event != UFFD_EVENT_FORK)
		err("wrong message: %u\n", msg.event);

	/* Just to be properly freed later */
	args->child_uffd = msg.arg.fork.ufd;
	return NULL;
}

typedef struct {
	int gup_fd;
	bool pinned;
} pin_args;

/*
 * Returns 0 if succeed, <0 for errors.  pin_pages() needs to be paired
 * with unpin_pages().  Currently it needs to be RO longterm pin to satisfy
 * all needs of the test cases (e.g., trigger unshare, trigger fork() early
 * CoW, etc.).
 */
static int pin_pages(pin_args *args, void *buffer, size_t size)
{
	struct pin_longterm_test test = {
		.addr = (uintptr_t)buffer,
		.size = size,
		/* Read-only pins */
		.flags = 0,
	};

	if (args->pinned)
		err("already pinned");

	args->gup_fd = open("/sys/kernel/debug/gup_test", O_RDWR);
	if (args->gup_fd < 0)
		return -errno;

	if (ioctl(args->gup_fd, PIN_LONGTERM_TEST_START, &test)) {
		/* Even if gup_test existed, can be an old gup_test / kernel */
		close(args->gup_fd);
		return -errno;
	}
	args->pinned = true;
	return 0;
}

static void unpin_pages(pin_args *args)
{
	if (!args->pinned)
		err("unpin without pin first");
	if (ioctl(args->gup_fd, PIN_LONGTERM_TEST_STOP))
		err("PIN_LONGTERM_TEST_STOP");
	close(args->gup_fd);
	args->pinned = false;
}

static int pagemap_test_fork(int uffd, bool with_event, bool test_pin)
{
	fork_event_args args = { .parent_uffd = uffd, .child_uffd = -1 };
	pthread_t thread;
	pid_t child;
	uint64_t value;
	int fd, result;

	/* Prepare a thread to resolve EVENT_FORK */
	if (with_event) {
		ready_for_fork = false;
		if (pthread_create(&thread, NULL, fork_event_consumer, &args))
			err("pthread_create()");
		while (!ready_for_fork)
			; /* Wait for the poll_thread to start executing before forking */
	}

	child = fork();
	if (!child) {
		/* Open the pagemap fd of the child itself */
		pin_args args = {};

		fd = pagemap_open();

		if (test_pin && pin_pages(&args, area_dst, page_size))
			/*
			 * Normally when reach here we have pinned in
			 * previous tests, so shouldn't fail anymore
			 */
			err("pin page failed in child");

		value = pagemap_get_entry(fd, area_dst);
		/*
		 * After fork(), we should handle uffd-wp bit differently:
		 *
		 * (1) when with EVENT_FORK, it should persist
		 * (2) when without EVENT_FORK, it should be dropped
		 */
		pagemap_check_wp(value, with_event);
		if (test_pin)
			unpin_pages(&args);
		/* Succeed */
		exit(0);
	}
	waitpid(child, &result, 0);

	if (with_event) {
		if (pthread_join(thread, NULL))
			err("pthread_join()");
		if (args.child_uffd < 0)
			err("Didn't receive child uffd");
		close(args.child_uffd);
	}

	return result;
}

static void uffd_wp_unpopulated_test(uffd_test_args_t *args)
{
	uint64_t value;
	int pagemap_fd;

	if (uffd_register(uffd, area_dst, nr_pages * page_size,
			  false, true, false))
		err("register failed");

	pagemap_fd = pagemap_open();

	/* Test applying pte marker to anon unpopulated */
	wp_range(uffd, (uint64_t)area_dst, page_size, true);
	value = pagemap_get_entry(pagemap_fd, area_dst);
	pagemap_check_wp(value, true);

	/* Test unprotect on anon pte marker */
	wp_range(uffd, (uint64_t)area_dst, page_size, false);
	value = pagemap_get_entry(pagemap_fd, area_dst);
	pagemap_check_wp(value, false);

	/* Test zap on anon marker */
	wp_range(uffd, (uint64_t)area_dst, page_size, true);
	if (madvise(area_dst, page_size, MADV_DONTNEED))
		err("madvise(MADV_DONTNEED) failed");
	value = pagemap_get_entry(pagemap_fd, area_dst);
	pagemap_check_wp(value, false);

	/* Test fault in after marker removed */
	*area_dst = 1;
	value = pagemap_get_entry(pagemap_fd, area_dst);
	pagemap_check_wp(value, false);
	/* Drop it to make pte none again */
	if (madvise(area_dst, page_size, MADV_DONTNEED))
		err("madvise(MADV_DONTNEED) failed");

	/* Test read-zero-page upon pte marker */
	wp_range(uffd, (uint64_t)area_dst, page_size, true);
	*(volatile char *)area_dst;
	/* Drop it to make pte none again */
	if (madvise(area_dst, page_size, MADV_DONTNEED))
		err("madvise(MADV_DONTNEED) failed");

	uffd_test_pass();
}

static void uffd_wp_fork_test_common(uffd_test_args_t *args,
				     bool with_event)
{
	int pagemap_fd;
	uint64_t value;

	if (uffd_register(uffd, area_dst, nr_pages * page_size,
			  false, true, false))
		err("register failed");

	pagemap_fd = pagemap_open();

	/* Touch the page */
	*area_dst = 1;
	wp_range(uffd, (uint64_t)area_dst, page_size, true);
	value = pagemap_get_entry(pagemap_fd, area_dst);
	pagemap_check_wp(value, true);
	if (pagemap_test_fork(uffd, with_event, false)) {
		uffd_test_fail("Detected %s uffd-wp bit in child in present pte",
			       with_event ? "missing" : "stall");
		goto out;
	}

	/*
	 * This is an attempt for zapping the pgtable so as to test the
	 * markers.
	 *
	 * For private mappings, PAGEOUT will only work on exclusive ptes
	 * (PM_MMAP_EXCLUSIVE) which we should satisfy.
	 *
	 * For shared, PAGEOUT may not work.  Use DONTNEED instead which
	 * plays a similar role of zapping (rather than freeing the page)
	 * to expose pte markers.
	 */
	if (args->mem_type->shared) {
		if (madvise(area_dst, page_size, MADV_DONTNEED))
			err("MADV_DONTNEED");
	} else {
		/*
		 * NOTE: ignore retval because private-hugetlb doesn't yet
		 * support swapping, so it could fail.
		 */
		madvise(area_dst, page_size, MADV_PAGEOUT);
	}

	/* Uffd-wp should persist even swapped out */
	value = pagemap_get_entry(pagemap_fd, area_dst);
	pagemap_check_wp(value, true);
	if (pagemap_test_fork(uffd, with_event, false)) {
		uffd_test_fail("Detected %s uffd-wp bit in child in zapped pte",
			       with_event ? "missing" : "stall");
		goto out;
	}

	/* Unprotect; this tests swap pte modifications */
	wp_range(uffd, (uint64_t)area_dst, page_size, false);
	value = pagemap_get_entry(pagemap_fd, area_dst);
	pagemap_check_wp(value, false);

	/* Fault in the page from disk */
	*area_dst = 2;
	value = pagemap_get_entry(pagemap_fd, area_dst);
	pagemap_check_wp(value, false);
	uffd_test_pass();
out:
	if (uffd_unregister(uffd, area_dst, nr_pages * page_size))
		err("unregister failed");
	close(pagemap_fd);
}

static void uffd_wp_fork_test(uffd_test_args_t *args)
{
	uffd_wp_fork_test_common(args, false);
}

static void uffd_wp_fork_with_event_test(uffd_test_args_t *args)
{
	uffd_wp_fork_test_common(args, true);
}

static void uffd_wp_fork_pin_test_common(uffd_test_args_t *args,
					 bool with_event)
{
	int pagemap_fd;
	pin_args pin_args = {};

	if (uffd_register(uffd, area_dst, page_size, false, true, false))
		err("register failed");

	pagemap_fd = pagemap_open();

	/* Touch the page */
	*area_dst = 1;
	wp_range(uffd, (uint64_t)area_dst, page_size, true);

	/*
	 * 1. First pin, then fork().  This tests fork() special path when
	 * doing early CoW if the page is private.
	 */
	if (pin_pages(&pin_args, area_dst, page_size)) {
		uffd_test_skip("Possibly CONFIG_GUP_TEST missing "
			       "or unprivileged");
		close(pagemap_fd);
		uffd_unregister(uffd, area_dst, page_size);
		return;
	}

	if (pagemap_test_fork(uffd, with_event, false)) {
		uffd_test_fail("Detected %s uffd-wp bit in early CoW of fork()",
			       with_event ? "missing" : "stall");
		unpin_pages(&pin_args);
		goto out;
	}

	unpin_pages(&pin_args);

	/*
	 * 2. First fork(), then pin (in the child, where test_pin==true).
	 * This tests COR, aka, page unsharing on private memories.
	 */
	if (pagemap_test_fork(uffd, with_event, true)) {
		uffd_test_fail("Detected %s uffd-wp bit when RO pin",
			       with_event ? "missing" : "stall");
		goto out;
	}
	uffd_test_pass();
out:
	if (uffd_unregister(uffd, area_dst, page_size))
		err("register failed");
	close(pagemap_fd);
}

static void uffd_wp_fork_pin_test(uffd_test_args_t *args)
{
	uffd_wp_fork_pin_test_common(args, false);
}

static void uffd_wp_fork_pin_with_event_test(uffd_test_args_t *args)
{
	uffd_wp_fork_pin_test_common(args, true);
}

static void check_memory_contents(char *p)
{
	unsigned long i, j;
	uint8_t expected_byte;

	for (i = 0; i < nr_pages; ++i) {
		expected_byte = ~((uint8_t)(i % ((uint8_t)-1)));
		for (j = 0; j < page_size; j++) {
			uint8_t v = *(uint8_t *)(p + (i * page_size) + j);
			if (v != expected_byte)
				err("unexpected page contents");
		}
	}
}

static void uffd_minor_test_common(bool test_collapse, bool test_wp)
{
	unsigned long p;
	pthread_t uffd_mon;
	char c;
	struct uffd_args args = { 0 };

	/*
	 * NOTE: MADV_COLLAPSE is not yet compatible with WP, so testing
	 * both do not make much sense.
	 */
	assert(!(test_collapse && test_wp));

	if (uffd_register(uffd, area_dst_alias, nr_pages * page_size,
			  /* NOTE! MADV_COLLAPSE may not work with uffd-wp */
			  false, test_wp, true))
		err("register failure");

	/*
	 * After registering with UFFD, populate the non-UFFD-registered side of
	 * the shared mapping. This should *not* trigger any UFFD minor faults.
	 */
	for (p = 0; p < nr_pages; ++p)
		memset(area_dst + (p * page_size), p % ((uint8_t)-1),
		       page_size);

	args.apply_wp = test_wp;
	if (pthread_create(&uffd_mon, NULL, uffd_poll_thread, &args))
		err("uffd_poll_thread create");

	/*
	 * Read each of the pages back using the UFFD-registered mapping. We
	 * expect that the first time we touch a page, it will result in a minor
	 * fault. uffd_poll_thread will resolve the fault by bit-flipping the
	 * page's contents, and then issuing a CONTINUE ioctl.
	 */
	check_memory_contents(area_dst_alias);

	if (write(pipefd[1], &c, sizeof(c)) != sizeof(c))
		err("pipe write");
	if (pthread_join(uffd_mon, NULL))
		err("join() failed");

	if (test_collapse) {
		if (madvise(area_dst_alias, nr_pages * page_size,
			    MADV_COLLAPSE)) {
			/* It's fine to fail for this one... */
			uffd_test_skip("MADV_COLLAPSE failed");
			return;
		}

		uffd_test_ops->check_pmd_mapping(area_dst,
						 nr_pages * page_size /
						 read_pmd_pagesize());
		/*
		 * This won't cause uffd-fault - it purely just makes sure there
		 * was no corruption.
		 */
		check_memory_contents(area_dst_alias);
	}

	if (args.missing_faults != 0 || args.minor_faults != nr_pages)
		uffd_test_fail("stats check error");
	else
		uffd_test_pass();
}

void uffd_minor_test(uffd_test_args_t *args)
{
	uffd_minor_test_common(false, false);
}

void uffd_minor_wp_test(uffd_test_args_t *args)
{
	uffd_minor_test_common(false, true);
}

void uffd_minor_collapse_test(uffd_test_args_t *args)
{
	uffd_minor_test_common(true, false);
}

static sigjmp_buf jbuf, *sigbuf;

static void sighndl(int sig, siginfo_t *siginfo, void *ptr)
{
	if (sig == SIGBUS) {
		if (sigbuf)
			siglongjmp(*sigbuf, 1);
		abort();
	}
}

/*
 * For non-cooperative userfaultfd test we fork() a process that will
 * generate pagefaults, will mremap the area monitored by the
 * userfaultfd and at last this process will release the monitored
 * area.
 * For the anonymous and shared memory the area is divided into two
 * parts, the first part is accessed before mremap, and the second
 * part is accessed after mremap. Since hugetlbfs does not support
 * mremap, the entire monitored area is accessed in a single pass for
 * HUGETLB_TEST.
 * The release of the pages currently generates event for shmem and
 * anonymous memory (UFFD_EVENT_REMOVE), hence it is not checked
 * for hugetlb.
 * For signal test(UFFD_FEATURE_SIGBUS), signal_test = 1, we register
 * monitored area, generate pagefaults and test that signal is delivered.
 * Use UFFDIO_COPY to allocate missing page and retry. For signal_test = 2
 * test robustness use case - we release monitored area, fork a process
 * that will generate pagefaults and verify signal is generated.
 * This also tests UFFD_FEATURE_EVENT_FORK event along with the signal
 * feature. Using monitor thread, verify no userfault events are generated.
 */
static int faulting_process(int signal_test, bool wp)
{
	unsigned long nr, i;
	unsigned long long count;
	unsigned long split_nr_pages;
	unsigned long lastnr;
	struct sigaction act;
	volatile unsigned long signalled = 0;

	split_nr_pages = (nr_pages + 1) / 2;

	if (signal_test) {
		sigbuf = &jbuf;
		memset(&act, 0, sizeof(act));
		act.sa_sigaction = sighndl;
		act.sa_flags = SA_SIGINFO;
		if (sigaction(SIGBUS, &act, 0))
			err("sigaction");
		lastnr = (unsigned long)-1;
	}

	for (nr = 0; nr < split_nr_pages; nr++) {
		volatile int steps = 1;
		unsigned long offset = nr * page_size;

		if (signal_test) {
			if (sigsetjmp(*sigbuf, 1) != 0) {
				if (steps == 1 && nr == lastnr)
					err("Signal repeated");

				lastnr = nr;
				if (signal_test == 1) {
					if (steps == 1) {
						/* This is a MISSING request */
						steps++;
						if (copy_page(uffd, offset, wp))
							signalled++;
					} else {
						/* This is a WP request */
						assert(steps == 2);
						wp_range(uffd,
							 (__u64)area_dst +
							 offset,
							 page_size, false);
					}
				} else {
					signalled++;
					continue;
				}
			}
		}

		count = *area_count(area_dst, nr);
		if (count != count_verify[nr])
			err("nr %lu memory corruption %llu %llu\n",
			    nr, count, count_verify[nr]);
		/*
		 * Trigger write protection if there is by writing
		 * the same value back.
		 */
		*area_count(area_dst, nr) = count;
	}

	if (signal_test)
		return signalled != split_nr_pages;

	area_dst = mremap(area_dst, nr_pages * page_size,  nr_pages * page_size,
			  MREMAP_MAYMOVE | MREMAP_FIXED, area_src);
	if (area_dst == MAP_FAILED)
		err("mremap");
	/* Reset area_src since we just clobbered it */
	area_src = NULL;

	for (; nr < nr_pages; nr++) {
		count = *area_count(area_dst, nr);
		if (count != count_verify[nr]) {
			err("nr %lu memory corruption %llu %llu\n",
			    nr, count, count_verify[nr]);
		}
		/*
		 * Trigger write protection if there is by writing
		 * the same value back.
		 */
		*area_count(area_dst, nr) = count;
	}

	uffd_test_ops->release_pages(area_dst);

	for (nr = 0; nr < nr_pages; nr++)
		for (i = 0; i < page_size; i++)
			if (*(area_dst + nr * page_size + i) != 0)
				err("page %lu offset %lu is not zero", nr, i);

	return 0;
}

static void uffd_sigbus_test_common(bool wp)
{
	unsigned long userfaults;
	pthread_t uffd_mon;
	pid_t pid;
	int err;
	char c;
	struct uffd_args args = { 0 };

	ready_for_fork = false;

	fcntl(uffd, F_SETFL, uffd_flags | O_NONBLOCK);

	if (uffd_register(uffd, area_dst, nr_pages * page_size,
			  true, wp, false))
		err("register failure");

	if (faulting_process(1, wp))
		err("faulting process failed");

	uffd_test_ops->release_pages(area_dst);

	args.apply_wp = wp;
	if (pthread_create(&uffd_mon, NULL, uffd_poll_thread, &args))
		err("uffd_poll_thread create");

	while (!ready_for_fork)
		; /* Wait for the poll_thread to start executing before forking */

	pid = fork();
	if (pid < 0)
		err("fork");

	if (!pid)
		exit(faulting_process(2, wp));

	waitpid(pid, &err, 0);
	if (err)
		err("faulting process failed");
	if (write(pipefd[1], &c, sizeof(c)) != sizeof(c))
		err("pipe write");
	if (pthread_join(uffd_mon, (void **)&userfaults))
		err("pthread_join()");

	if (userfaults)
		uffd_test_fail("Signal test failed, userfaults: %ld", userfaults);
	else
		uffd_test_pass();
}

static void uffd_sigbus_test(uffd_test_args_t *args)
{
	uffd_sigbus_test_common(false);
}

static void uffd_sigbus_wp_test(uffd_test_args_t *args)
{
	uffd_sigbus_test_common(true);
}

static void uffd_events_test_common(bool wp)
{
	pthread_t uffd_mon;
	pid_t pid;
	int err;
	char c;
	struct uffd_args args = { 0 };

	ready_for_fork = false;

	fcntl(uffd, F_SETFL, uffd_flags | O_NONBLOCK);
	if (uffd_register(uffd, area_dst, nr_pages * page_size,
			  true, wp, false))
		err("register failure");

	args.apply_wp = wp;
	if (pthread_create(&uffd_mon, NULL, uffd_poll_thread, &args))
		err("uffd_poll_thread create");

	while (!ready_for_fork)
		; /* Wait for the poll_thread to start executing before forking */

	pid = fork();
	if (pid < 0)
		err("fork");

	if (!pid)
		exit(faulting_process(0, wp));

	waitpid(pid, &err, 0);
	if (err)
		err("faulting process failed");
	if (write(pipefd[1], &c, sizeof(c)) != sizeof(c))
		err("pipe write");
	if (pthread_join(uffd_mon, NULL))
		err("pthread_join()");

	if (args.missing_faults != nr_pages)
		uffd_test_fail("Fault counts wrong");
	else
		uffd_test_pass();
}

static void uffd_events_test(uffd_test_args_t *args)
{
	uffd_events_test_common(false);
}

static void uffd_events_wp_test(uffd_test_args_t *args)
{
	uffd_events_test_common(true);
}

static void retry_uffdio_zeropage(int ufd,
				  struct uffdio_zeropage *uffdio_zeropage)
{
	uffd_test_ops->alias_mapping(&uffdio_zeropage->range.start,
				     uffdio_zeropage->range.len,
				     0);
	if (ioctl(ufd, UFFDIO_ZEROPAGE, uffdio_zeropage)) {
		if (uffdio_zeropage->zeropage != -EEXIST)
			err("UFFDIO_ZEROPAGE error: %"PRId64,
			    (int64_t)uffdio_zeropage->zeropage);
	} else {
		err("UFFDIO_ZEROPAGE error: %"PRId64,
		    (int64_t)uffdio_zeropage->zeropage);
	}
}

static bool do_uffdio_zeropage(int ufd, bool has_zeropage)
{
	struct uffdio_zeropage uffdio_zeropage = { 0 };
	int ret;
	__s64 res;

	uffdio_zeropage.range.start = (unsigned long) area_dst;
	uffdio_zeropage.range.len = page_size;
	uffdio_zeropage.mode = 0;
	ret = ioctl(ufd, UFFDIO_ZEROPAGE, &uffdio_zeropage);
	res = uffdio_zeropage.zeropage;
	if (ret) {
		/* real retval in ufdio_zeropage.zeropage */
		if (has_zeropage)
			err("UFFDIO_ZEROPAGE error: %"PRId64, (int64_t)res);
		else if (res != -EINVAL)
			err("UFFDIO_ZEROPAGE not -EINVAL");
	} else if (has_zeropage) {
		if (res != page_size)
			err("UFFDIO_ZEROPAGE unexpected size");
		else
			retry_uffdio_zeropage(ufd, &uffdio_zeropage);
		return true;
	} else
		err("UFFDIO_ZEROPAGE succeeded");

	return false;
}

/*
 * Registers a range with MISSING mode only for zeropage test.  Return true
 * if UFFDIO_ZEROPAGE supported, false otherwise. Can't use uffd_register()
 * because we want to detect .ioctls along the way.
 */
static bool
uffd_register_detect_zeropage(int uffd, void *addr, uint64_t len)
{
	uint64_t ioctls = 0;

	if (uffd_register_with_ioctls(uffd, addr, len, true,
				      false, false, &ioctls))
		err("zeropage register fail");

	return ioctls & (1 << _UFFDIO_ZEROPAGE);
}

/* exercise UFFDIO_ZEROPAGE */
static void uffd_zeropage_test(uffd_test_args_t *args)
{
	bool has_zeropage;
	int i;

	has_zeropage = uffd_register_detect_zeropage(uffd, area_dst, page_size);
	if (area_dst_alias)
		/* Ignore the retval; we already have it */
		uffd_register_detect_zeropage(uffd, area_dst_alias, page_size);

	if (do_uffdio_zeropage(uffd, has_zeropage))
		for (i = 0; i < page_size; i++)
			if (area_dst[i] != 0)
				err("data non-zero at offset %d\n", i);

	if (uffd_unregister(uffd, area_dst, page_size))
		err("unregister");

	if (area_dst_alias && uffd_unregister(uffd, area_dst_alias, page_size))
		err("unregister");

	uffd_test_pass();
}

static void uffd_register_poison(int uffd, void *addr, uint64_t len)
{
	uint64_t ioctls = 0;
	uint64_t expected = (1 << _UFFDIO_COPY) | (1 << _UFFDIO_POISON);

	if (uffd_register_with_ioctls(uffd, addr, len, true,
				      false, false, &ioctls))
		err("poison register fail");

	if ((ioctls & expected) != expected)
		err("registered area doesn't support COPY and POISON ioctls");
}

static void do_uffdio_poison(int uffd, unsigned long offset)
{
	struct uffdio_poison uffdio_poison = { 0 };
	int ret;
	__s64 res;

	uffdio_poison.range.start = (unsigned long) area_dst + offset;
	uffdio_poison.range.len = page_size;
	uffdio_poison.mode = 0;
	ret = ioctl(uffd, UFFDIO_POISON, &uffdio_poison);
	res = uffdio_poison.updated;

	if (ret)
		err("UFFDIO_POISON error: %"PRId64, (int64_t)res);
	else if (res != page_size)
		err("UFFDIO_POISON unexpected size: %"PRId64, (int64_t)res);
}

static void uffd_poison_handle_fault(
	struct uffd_msg *msg, struct uffd_args *args)
{
	unsigned long offset;

	if (msg->event != UFFD_EVENT_PAGEFAULT)
		err("unexpected msg event %u", msg->event);

	if (msg->arg.pagefault.flags &
	    (UFFD_PAGEFAULT_FLAG_WP | UFFD_PAGEFAULT_FLAG_MINOR))
		err("unexpected fault type %llu", msg->arg.pagefault.flags);

	offset = (char *)(unsigned long)msg->arg.pagefault.address - area_dst;
	offset &= ~(page_size-1);

	/* Odd pages -> copy zeroed page; even pages -> poison. */
	if (offset & page_size)
		copy_page(uffd, offset, false);
	else
		do_uffdio_poison(uffd, offset);
}

/* Make sure to cover odd/even, and minimum duplications */
#define  UFFD_POISON_TEST_NPAGES  4

static void uffd_poison_test(uffd_test_args_t *targs)
{
	pthread_t uffd_mon;
	char c;
	struct uffd_args args = { 0 };
	struct sigaction act = { 0 };
	unsigned long nr_sigbus = 0;
	unsigned long nr, poison_pages = UFFD_POISON_TEST_NPAGES;

	if (nr_pages < poison_pages) {
		uffd_test_skip("Too few pages for POISON test");
		return;
	}

	fcntl(uffd, F_SETFL, uffd_flags | O_NONBLOCK);

	uffd_register_poison(uffd, area_dst, poison_pages * page_size);
	memset(area_src, 0, poison_pages * page_size);

	args.handle_fault = uffd_poison_handle_fault;
	if (pthread_create(&uffd_mon, NULL, uffd_poll_thread, &args))
		err("uffd_poll_thread create");

	sigbuf = &jbuf;
	act.sa_sigaction = sighndl;
	act.sa_flags = SA_SIGINFO;
	if (sigaction(SIGBUS, &act, 0))
		err("sigaction");

	for (nr = 0; nr < poison_pages; ++nr) {
		unsigned long offset = nr * page_size;
		const char *bytes = (const char *) area_dst + offset;
		const char *i;

		if (sigsetjmp(*sigbuf, 1)) {
			/*
			 * Access below triggered a SIGBUS, which was caught by
			 * sighndl, which then jumped here. Count this SIGBUS,
			 * and move on to next page.
			 */
			++nr_sigbus;
			continue;
		}

		for (i = bytes; i < bytes + page_size; ++i) {
			if (*i)
				err("nonzero byte in area_dst (%p) at %p: %u",
				    area_dst, i, *i);
		}
	}

	if (write(pipefd[1], &c, sizeof(c)) != sizeof(c))
		err("pipe write");
	if (pthread_join(uffd_mon, NULL))
		err("pthread_join()");

	if (nr_sigbus != poison_pages / 2)
		err("expected to receive %lu SIGBUS, actually received %lu",
		    poison_pages / 2, nr_sigbus);

	uffd_test_pass();
}

static void
uffd_move_handle_fault_common(struct uffd_msg *msg, struct uffd_args *args,
			      unsigned long len)
{
	unsigned long offset;

	if (msg->event != UFFD_EVENT_PAGEFAULT)
		err("unexpected msg event %u", msg->event);

	if (msg->arg.pagefault.flags &
	    (UFFD_PAGEFAULT_FLAG_WP | UFFD_PAGEFAULT_FLAG_MINOR | UFFD_PAGEFAULT_FLAG_WRITE))
		err("unexpected fault type %llu", msg->arg.pagefault.flags);

	offset = (char *)(unsigned long)msg->arg.pagefault.address - area_dst;
	offset &= ~(len-1);

	if (move_page(uffd, offset, len))
		args->missing_faults++;
}

static void uffd_move_handle_fault(struct uffd_msg *msg,
				   struct uffd_args *args)
{
	uffd_move_handle_fault_common(msg, args, page_size);
}

static void uffd_move_pmd_handle_fault(struct uffd_msg *msg,
				       struct uffd_args *args)
{
	uffd_move_handle_fault_common(msg, args, read_pmd_pagesize());
}

static void
uffd_move_test_common(uffd_test_args_t *targs, unsigned long chunk_size,
		      void (*handle_fault)(struct uffd_msg *msg, struct uffd_args *args))
{
	unsigned long nr;
	pthread_t uffd_mon;
	char c;
	unsigned long long count;
	struct uffd_args args = { 0 };
	char *orig_area_src = NULL, *orig_area_dst = NULL;
	unsigned long step_size, step_count;
	unsigned long src_offs = 0;
	unsigned long dst_offs = 0;

	/* Prevent source pages from being mapped more than once */
	if (madvise(area_src, nr_pages * page_size, MADV_DONTFORK))
		err("madvise(MADV_DONTFORK) failure");

	if (uffd_register(uffd, area_dst, nr_pages * page_size,
			  true, false, false))
		err("register failure");

	args.handle_fault = handle_fault;
	if (pthread_create(&uffd_mon, NULL, uffd_poll_thread, &args))
		err("uffd_poll_thread create");

	step_size = chunk_size / page_size;
	step_count = nr_pages / step_size;

	if (chunk_size > page_size) {
		char *aligned_src = ALIGN_UP(area_src, chunk_size);
		char *aligned_dst = ALIGN_UP(area_dst, chunk_size);

		if (aligned_src != area_src || aligned_dst != area_dst) {
			src_offs = (aligned_src - area_src) / page_size;
			dst_offs = (aligned_dst - area_dst) / page_size;
			step_count--;
		}
		orig_area_src = area_src;
		orig_area_dst = area_dst;
		area_src = aligned_src;
		area_dst = aligned_dst;
	}

	/*
	 * Read each of the pages back using the UFFD-registered mapping. We
	 * expect that the first time we touch a page, it will result in a missing
	 * fault. uffd_poll_thread will resolve the fault by moving source
	 * page to destination.
	 */
	for (nr = 0; nr < step_count * step_size; nr += step_size) {
		unsigned long i;

		/* Check area_src content */
		for (i = 0; i < step_size; i++) {
			count = *area_count(area_src, nr + i);
			if (count != count_verify[src_offs + nr + i])
				err("nr %lu source memory invalid %llu %llu\n",
				    nr + i, count, count_verify[src_offs + nr + i]);
		}

		/* Faulting into area_dst should move the page or the huge page */
		for (i = 0; i < step_size; i++) {
			count = *area_count(area_dst, nr + i);
			if (count != count_verify[dst_offs + nr + i])
				err("nr %lu memory corruption %llu %llu\n",
				    nr, count, count_verify[dst_offs + nr + i]);
		}

		/* Re-check area_src content which should be empty */
		for (i = 0; i < step_size; i++) {
			count = *area_count(area_src, nr + i);
			if (count != 0)
				err("nr %lu move failed %llu %llu\n",
				    nr, count, count_verify[src_offs + nr + i]);
		}
	}
	if (chunk_size > page_size) {
		area_src = orig_area_src;
		area_dst = orig_area_dst;
	}

	if (write(pipefd[1], &c, sizeof(c)) != sizeof(c))
		err("pipe write");
	if (pthread_join(uffd_mon, NULL))
		err("join() failed");

	if (args.missing_faults != step_count || args.minor_faults != 0)
		uffd_test_fail("stats check error");
	else
		uffd_test_pass();
}

static void uffd_move_test(uffd_test_args_t *targs)
{
	uffd_move_test_common(targs, page_size, uffd_move_handle_fault);
}

static void uffd_move_pmd_test(uffd_test_args_t *targs)
{
	if (madvise(area_dst, nr_pages * page_size, MADV_HUGEPAGE))
		err("madvise(MADV_HUGEPAGE) failure");
	uffd_move_test_common(targs, read_pmd_pagesize(),
			      uffd_move_pmd_handle_fault);
}

static void uffd_move_pmd_split_test(uffd_test_args_t *targs)
{
	if (madvise(area_dst, nr_pages * page_size, MADV_NOHUGEPAGE))
		err("madvise(MADV_NOHUGEPAGE) failure");
	uffd_move_test_common(targs, read_pmd_pagesize(),
			      uffd_move_pmd_handle_fault);
}

static bool
uffdio_verify_results(const char *name, int ret, int error, long result)
{
	/*
	 * Should always return -1 with errno=EAGAIN, with corresponding
	 * result field updated in ioctl() args to be -EAGAIN too
	 * (e.g. copy.copy field for UFFDIO_COPY).
	 */
	if (ret != -1) {
		uffd_test_fail("%s should have returned -1", name);
		return false;
	}

	if (error != EAGAIN) {
		uffd_test_fail("%s should have errno==EAGAIN", name);
		return false;
	}

	if (result != -EAGAIN) {
		uffd_test_fail("%s should have been updated for -EAGAIN",
			       name);
		return false;
	}

	return true;
}

/*
 * This defines a function to test one ioctl.  Note that here "field" can
 * be 1 or anything not -EAGAIN.  With that initial value set, we can
 * verify later that it should be updated by kernel (when -EAGAIN
 * returned), by checking whether it is also updated to -EAGAIN.
 */
#define DEFINE_MMAP_CHANGING_TEST(name, ioctl_name, field)		\
	static bool uffdio_mmap_changing_test_##name(int fd)		\
	{								\
		int ret;						\
		struct uffdio_##name args = {				\
			.field = 1,					\
		};							\
		ret = ioctl(fd, ioctl_name, &args);			\
		return uffdio_verify_results(#ioctl_name, ret, errno, args.field); \
	}

DEFINE_MMAP_CHANGING_TEST(zeropage, UFFDIO_ZEROPAGE, zeropage)
DEFINE_MMAP_CHANGING_TEST(copy, UFFDIO_COPY, copy)
DEFINE_MMAP_CHANGING_TEST(move, UFFDIO_MOVE, move)
DEFINE_MMAP_CHANGING_TEST(poison, UFFDIO_POISON, updated)
DEFINE_MMAP_CHANGING_TEST(continue, UFFDIO_CONTINUE, mapped)

typedef enum {
	/* We actually do not care about any state except UNINTERRUPTIBLE.. */
	THR_STATE_UNKNOWN = 0,
	THR_STATE_UNINTERRUPTIBLE,
} thread_state;

static void sleep_short(void)
{
	usleep(1000);
}

static thread_state thread_state_get(pid_t tid)
{
	const char *header = "State:\t";
	char tmp[256], *p, c;
	FILE *fp;

	snprintf(tmp, sizeof(tmp), "/proc/%d/status", tid);
	fp = fopen(tmp, "r");

	if (!fp)
		return THR_STATE_UNKNOWN;

	while (fgets(tmp, sizeof(tmp), fp)) {
		p = strstr(tmp, header);
		if (p) {
			/* For example, "State:\tD (disk sleep)" */
			c = *(p + sizeof(header) - 1);
			return c == 'D' ?
			    THR_STATE_UNINTERRUPTIBLE : THR_STATE_UNKNOWN;
		}
	}

	return THR_STATE_UNKNOWN;
}

static void thread_state_until(pid_t tid, thread_state state)
{
	thread_state s;

	do {
		s = thread_state_get(tid);
		sleep_short();
	} while (s != state);
}

static void *uffd_mmap_changing_thread(void *opaque)
{
	volatile pid_t *pid = opaque;
	int ret;

	/* Unfortunately, it's only fetch-able from the thread itself.. */
	assert(*pid == 0);
	*pid = syscall(SYS_gettid);

	/* Inject an event, this will hang solid until the event read */
	ret = madvise(area_dst, page_size, MADV_REMOVE);
	if (ret)
		err("madvise(MADV_REMOVE) failed");

	return NULL;
}

static void uffd_consume_message(int fd)
{
	struct uffd_msg msg = { 0 };

	while (uffd_read_msg(fd, &msg));
}

static void uffd_mmap_changing_test(uffd_test_args_t *targs)
{
	/*
	 * This stores the real PID (which can be different from how tid is
	 * defined..) for the child thread, 0 means not initialized.
	 */
	pid_t pid = 0;
	pthread_t tid;
	int ret;

	if (uffd_register(uffd, area_dst, nr_pages * page_size,
			  true, false, false))
		err("uffd_register() failed");

	/* Create a thread to generate the racy event */
	ret = pthread_create(&tid, NULL, uffd_mmap_changing_thread, &pid);
	if (ret)
		err("pthread_create() failed");

	/*
	 * Wait until the thread setup the pid.  Use volatile to make sure
	 * it reads from RAM not regs.
	 */
	while (!(volatile pid_t)pid)
		sleep_short();

	/* Wait until the thread hangs at REMOVE event */
	thread_state_until(pid, THR_STATE_UNINTERRUPTIBLE);

	if (!uffdio_mmap_changing_test_copy(uffd))
		return;

	if (!uffdio_mmap_changing_test_zeropage(uffd))
		return;

	if (!uffdio_mmap_changing_test_move(uffd))
		return;

	if (!uffdio_mmap_changing_test_poison(uffd))
		return;

	if (!uffdio_mmap_changing_test_continue(uffd))
		return;

	/*
	 * All succeeded above!  Recycle everything.  Start by reading the
	 * event so as to kick the thread roll again..
	 */
	uffd_consume_message(uffd);

	ret = pthread_join(tid, NULL);
	assert(ret == 0);

	uffd_test_pass();
}

static int prevent_hugepages(const char **errmsg)
{
	/* This should be done before source area is populated */
	if (madvise(area_src, nr_pages * page_size, MADV_NOHUGEPAGE)) {
		/* Ignore only if CONFIG_TRANSPARENT_HUGEPAGE=n */
		if (errno != EINVAL) {
			if (errmsg)
				*errmsg = "madvise(MADV_NOHUGEPAGE) failed";
			return -errno;
		}
	}
	return 0;
}

static int request_hugepages(const char **errmsg)
{
	/* This should be done before source area is populated */
	if (madvise(area_src, nr_pages * page_size, MADV_HUGEPAGE)) {
		if (errmsg) {
			*errmsg = (errno == EINVAL) ?
				"CONFIG_TRANSPARENT_HUGEPAGE is not set" :
				"madvise(MADV_HUGEPAGE) failed";
		}
		return -errno;
	}
	return 0;
}

struct uffd_test_case_ops uffd_move_test_case_ops = {
	.post_alloc = prevent_hugepages,
};

struct uffd_test_case_ops uffd_move_test_pmd_case_ops = {
	.post_alloc = request_hugepages,
};

/*
 * Test the returned uffdio_register.ioctls with different register modes.
 * Note that _UFFDIO_ZEROPAGE is tested separately in the zeropage test.
 */
static void
do_register_ioctls_test(uffd_test_args_t *args, bool miss, bool wp, bool minor)
{
	uint64_t ioctls = 0, expected = BIT_ULL(_UFFDIO_WAKE);
	mem_type_t *mem_type = args->mem_type;
	int ret;

	ret = uffd_register_with_ioctls(uffd, area_dst, page_size,
					miss, wp, minor, &ioctls);

	/*
	 * Handle special cases of UFFDIO_REGISTER here where it should
	 * just fail with -EINVAL first..
	 *
	 * Case 1: register MINOR on anon
	 * Case 2: register with no mode selected
	 */
	if ((minor && (mem_type->mem_flag == MEM_ANON)) ||
	    (!miss && !wp && !minor)) {
		if (ret != -EINVAL)
			err("register (miss=%d, wp=%d, minor=%d) failed "
			    "with wrong errno=%d", miss, wp, minor, ret);
		return;
	}

	/* UFFDIO_REGISTER should succeed, then check ioctls returned */
	if (miss)
		expected |= BIT_ULL(_UFFDIO_COPY);
	if (wp)
		expected |= BIT_ULL(_UFFDIO_WRITEPROTECT);
	if (minor)
		expected |= BIT_ULL(_UFFDIO_CONTINUE);

	if ((ioctls & expected) != expected)
		err("unexpected uffdio_register.ioctls "
		    "(miss=%d, wp=%d, minor=%d): expected=0x%"PRIx64", "
		    "returned=0x%"PRIx64, miss, wp, minor, expected, ioctls);

	if (uffd_unregister(uffd, area_dst, page_size))
		err("unregister");
}

static void uffd_register_ioctls_test(uffd_test_args_t *args)
{
	int miss, wp, minor;

	for (miss = 0; miss <= 1; miss++)
		for (wp = 0; wp <= 1; wp++)
			for (minor = 0; minor <= 1; minor++)
				do_register_ioctls_test(args, miss, wp, minor);

	uffd_test_pass();
}

uffd_test_case_t uffd_tests[] = {
	{
		/* Test returned uffdio_register.ioctls. */
		.name = "register-ioctls",
		.uffd_fn = uffd_register_ioctls_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_MISSING_HUGETLBFS |
		UFFD_FEATURE_MISSING_SHMEM |
		UFFD_FEATURE_PAGEFAULT_FLAG_WP |
		UFFD_FEATURE_WP_HUGETLBFS_SHMEM |
		UFFD_FEATURE_MINOR_HUGETLBFS |
		UFFD_FEATURE_MINOR_SHMEM,
	},
	{
		.name = "zeropage",
		.uffd_fn = uffd_zeropage_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = 0,
	},
	{
		.name = "move",
		.uffd_fn = uffd_move_test,
		.mem_targets = MEM_ANON,
		.uffd_feature_required = UFFD_FEATURE_MOVE,
		.test_case_ops = &uffd_move_test_case_ops,
	},
	{
		.name = "move-pmd",
		.uffd_fn = uffd_move_pmd_test,
		.mem_targets = MEM_ANON,
		.uffd_feature_required = UFFD_FEATURE_MOVE,
		.test_case_ops = &uffd_move_test_pmd_case_ops,
	},
	{
		.name = "move-pmd-split",
		.uffd_fn = uffd_move_pmd_split_test,
		.mem_targets = MEM_ANON,
		.uffd_feature_required = UFFD_FEATURE_MOVE,
		.test_case_ops = &uffd_move_test_pmd_case_ops,
	},
	{
		.name = "wp-fork",
		.uffd_fn = uffd_wp_fork_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_PAGEFAULT_FLAG_WP |
		UFFD_FEATURE_WP_HUGETLBFS_SHMEM,
	},
	{
		.name = "wp-fork-with-event",
		.uffd_fn = uffd_wp_fork_with_event_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_PAGEFAULT_FLAG_WP |
		UFFD_FEATURE_WP_HUGETLBFS_SHMEM |
		/* when set, child process should inherit uffd-wp bits */
		UFFD_FEATURE_EVENT_FORK,
	},
	{
		.name = "wp-fork-pin",
		.uffd_fn = uffd_wp_fork_pin_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_PAGEFAULT_FLAG_WP |
		UFFD_FEATURE_WP_HUGETLBFS_SHMEM,
	},
	{
		.name = "wp-fork-pin-with-event",
		.uffd_fn = uffd_wp_fork_pin_with_event_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_PAGEFAULT_FLAG_WP |
		UFFD_FEATURE_WP_HUGETLBFS_SHMEM |
		/* when set, child process should inherit uffd-wp bits */
		UFFD_FEATURE_EVENT_FORK,
	},
	{
		.name = "wp-unpopulated",
		.uffd_fn = uffd_wp_unpopulated_test,
		.mem_targets = MEM_ANON,
		.uffd_feature_required =
		UFFD_FEATURE_PAGEFAULT_FLAG_WP | UFFD_FEATURE_WP_UNPOPULATED,
	},
	{
		.name = "minor",
		.uffd_fn = uffd_minor_test,
		.mem_targets = MEM_SHMEM | MEM_HUGETLB,
		.uffd_feature_required =
		UFFD_FEATURE_MINOR_HUGETLBFS | UFFD_FEATURE_MINOR_SHMEM,
	},
	{
		.name = "minor-wp",
		.uffd_fn = uffd_minor_wp_test,
		.mem_targets = MEM_SHMEM | MEM_HUGETLB,
		.uffd_feature_required =
		UFFD_FEATURE_MINOR_HUGETLBFS | UFFD_FEATURE_MINOR_SHMEM |
		UFFD_FEATURE_PAGEFAULT_FLAG_WP |
		/*
		 * HACK: here we leveraged WP_UNPOPULATED to detect whether
		 * minor mode supports wr-protect.  There's no feature flag
		 * for it so this is the best we can test against.
		 */
		UFFD_FEATURE_WP_UNPOPULATED,
	},
	{
		.name = "minor-collapse",
		.uffd_fn = uffd_minor_collapse_test,
		/* MADV_COLLAPSE only works with shmem */
		.mem_targets = MEM_SHMEM,
		/* We can't test MADV_COLLAPSE, so try our luck */
		.uffd_feature_required = UFFD_FEATURE_MINOR_SHMEM,
	},
	{
		.name = "sigbus",
		.uffd_fn = uffd_sigbus_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_SIGBUS |
		UFFD_FEATURE_EVENT_FORK,
	},
	{
		.name = "sigbus-wp",
		.uffd_fn = uffd_sigbus_wp_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_SIGBUS |
		UFFD_FEATURE_EVENT_FORK | UFFD_FEATURE_PAGEFAULT_FLAG_WP |
		UFFD_FEATURE_WP_HUGETLBFS_SHMEM,
	},
	{
		.name = "events",
		.uffd_fn = uffd_events_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_EVENT_FORK |
		UFFD_FEATURE_EVENT_REMAP | UFFD_FEATURE_EVENT_REMOVE,
	},
	{
		.name = "events-wp",
		.uffd_fn = uffd_events_wp_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_EVENT_FORK |
		UFFD_FEATURE_EVENT_REMAP | UFFD_FEATURE_EVENT_REMOVE |
		UFFD_FEATURE_PAGEFAULT_FLAG_WP |
		UFFD_FEATURE_WP_HUGETLBFS_SHMEM,
	},
	{
		.name = "poison",
		.uffd_fn = uffd_poison_test,
		.mem_targets = MEM_ALL,
		.uffd_feature_required = UFFD_FEATURE_POISON,
	},
	{
		.name = "mmap-changing",
		.uffd_fn = uffd_mmap_changing_test,
		/*
		 * There's no point running this test over all mem types as
		 * they share the same code paths.
		 *
		 * Choose shmem for simplicity, because (1) shmem supports
		 * MINOR mode to cover UFFDIO_CONTINUE, and (2) shmem is
		 * almost always available (unlike hugetlb).  Here we
		 * abused SHMEM for UFFDIO_MOVE, but the test we want to
		 * cover doesn't yet need the correct memory type..
		 */
		.mem_targets = MEM_SHMEM,
		/*
		 * Any UFFD_FEATURE_EVENT_* should work to trigger the
		 * race logically, but choose the simplest (REMOVE).
		 *
		 * Meanwhile, since we'll cover quite a few new ioctl()s
		 * (CONTINUE, POISON, MOVE), skip this test for old kernels
		 * by choosing all of them.
		 */
		.uffd_feature_required = UFFD_FEATURE_EVENT_REMOVE |
		UFFD_FEATURE_MOVE | UFFD_FEATURE_POISON |
		UFFD_FEATURE_MINOR_SHMEM,
	},
};

static void usage(const char *prog)
{
	printf("usage: %s [-f TESTNAME]\n", prog);
	puts("");
	puts(" -f: test name to filter (e.g., event)");
	puts(" -h: show the help msg");
	puts(" -l: list tests only");
	puts("");
	exit(KSFT_FAIL);
}

int main(int argc, char *argv[])
{
	int n_tests = sizeof(uffd_tests) / sizeof(uffd_test_case_t);
	int n_mems = sizeof(mem_types) / sizeof(mem_type_t);
	const char *test_filter = NULL;
	bool list_only = false;
	uffd_test_case_t *test;
	mem_type_t *mem_type;
	uffd_test_args_t args;
	const char *errmsg;
	int has_uffd, opt;
	int i, j;

	while ((opt = getopt(argc, argv, "f:hl")) != -1) {
		switch (opt) {
		case 'f':
			test_filter = optarg;
			break;
		case 'l':
			list_only = true;
			break;
		case 'h':
		default:
			/* Unknown */
			usage(argv[0]);
			break;
		}
	}

	if (!test_filter && !list_only) {
		has_uffd = test_uffd_api(false);
		has_uffd |= test_uffd_api(true);

		if (!has_uffd) {
			printf("Userfaultfd not supported or unprivileged, skip all tests\n");
			exit(KSFT_SKIP);
		}
	}

	for (i = 0; i < n_tests; i++) {
		test = &uffd_tests[i];
		if (test_filter && !strstr(test->name, test_filter))
			continue;
		if (list_only) {
			printf("%s\n", test->name);
			continue;
		}
		for (j = 0; j < n_mems; j++) {
			mem_type = &mem_types[j];
			if (!(test->mem_targets & mem_type->mem_flag))
				continue;

			uffd_test_start("%s on %s", test->name, mem_type->name);
			if ((mem_type->mem_flag == MEM_HUGETLB ||
			    mem_type->mem_flag == MEM_HUGETLB_PRIVATE) &&
			    (default_huge_page_size() == 0)) {
				uffd_test_skip("huge page size is 0, feature missing?");
				continue;
			}
			if (!uffd_feature_supported(test)) {
				uffd_test_skip("feature missing");
				continue;
			}
			if (uffd_setup_environment(&args, test, mem_type,
						   &errmsg)) {
				uffd_test_skip(errmsg);
				continue;
			}
			test->uffd_fn(&args);
			uffd_test_ctx_clear();
		}
	}

	if (!list_only)
		uffd_test_report();

	return ksft_get_fail_cnt() ? KSFT_FAIL : KSFT_PASS;
}

#else /* __NR_userfaultfd */

#warning "missing __NR_userfaultfd definition"

int main(void)
{
	printf("Skipping %s (missing __NR_userfaultfd)\n", __file__);
	return KSFT_SKIP;
}

#endif /* __NR_userfaultfd */
