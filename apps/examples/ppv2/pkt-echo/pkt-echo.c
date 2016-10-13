/*
 * (C) 2011-2014 Luigi Rizzo, Matteo Landi
 *
 * BSD license
 *
 * A netmap client to bridge two network interfaces
 * (or one interface and the host stack).
 *
 * $FreeBSD: head/tools/tools/netmap/bridge.c 228975 2011-12-30 00:04:11Z uqs $
 */

#define _GNU_SOURCE	/* for CPU_SET() */
#include <stdio.h>
#define NETMAP_WITH_LIBS
#include <net/netmap_user.h>
#include <ctype.h>	// isprint()
#include <sys/poll.h>
#include <pthread.h>


#ifdef linux
#define cpuset_t	cpu_set_t
#endif  /* linux */

#define BUSY_WAIT
//#define VERBOSE_SUPPORT
//#define ADDR_TBL_EMUL

#define USE_APP_PREFETCH
#define PREFETCH_SHIFT 4


#ifdef ADDR_TBL_EMUL
struct tbl_entry {
	int			valid;
	uint8_t		src_addr[6];
	int 		port;
};

struct addr_tbl {
	int 				num_ents;
	struct tbl_entry	entries[10];
};
#endif /* ADDR_TBL_EMUL */

/**
 * Ethernet MAC address
 */
typedef struct {
	uint8_t addr[6];
} ethaddr_t;


struct glob_arg {
	int verbose;

	int nthreads;
	int cpus;	/* cpus used for running */
	int system_cpus;	/* cpus on the system */

	int do_abort;
	int zerocopy; /* enable zerocopy if possible */
	u_int burst;
	int affinity;
	int loopback;
	int echo;
	int dump_payload;
	uint64_t	qs_map;
	int		qs_map_shift;

	int main_fds[2];
	char *ifnames[2];
	struct nm_desc *nmds[2];
#ifdef ADDR_TBL_EMUL
	struct addr_tbl	addr_tbl;
#endif /* ADDR_TBL_EMUL */
};

/*
 * Arguments for a new thread. The same structure is used by
 * the source and the sink
 */
struct targ {
	int			 zerocopy; /* enable zerocopy if possible */
	u_int			 burst;
	int			 echo;
	uint64_t		 qs_map;
	struct nm_desc		*nmds[2];
	struct pollfd		 pollfds[2];
#ifdef ADDR_TBL_EMUL
	struct addr_tbl		*addr_tbl;
#endif /* ADDR_TBL_EMUL */

	int			 used;
	int			 completed;
	int			 cancel;

	int			 me;
	pthread_t		 thread;
	int 			 affinity;
	int 			 fds[2];

	struct glob_arg	*g;
};


static struct targ *targs;
static int global_nthreads;


static void sigint_h(int sig)
{
	int i;

	(void)sig;	/* UNUSED */
	D("received control-C on thread %p", (void *)pthread_self());
	for (i = 0; i < global_nthreads; i++) {
		targs[i].cancel = 1;
	}
	signal(SIGINT, SIG_DFL);
}

#ifdef USE_APP_PREFETCH
static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}
#endif /* USE_APP_PREFETCH */

/* sysctl wrapper to return the number of active CPUs */
static int system_ncpus(void)
{
	int ncpus;
#if defined (__FreeBSD__)
	int mib[2] = { CTL_HW, HW_NCPU };
	size_t len = sizeof(mib);
	sysctl(mib, 2, &ncpus, &len, NULL, 0);
#elif defined(linux)
	ncpus = sysconf(_SC_NPROCESSORS_ONLN);
#elif defined(_WIN32)
	{
		SYSTEM_INFO sysinfo;
		GetSystemInfo(&sysinfo);
		ncpus = sysinfo.dwNumberOfProcessors;
	}
#else /* others */
	ncpus = 1;
#endif /* others */
	return (ncpus);
}

/* set the thread affinity. */
static int setaffinity(pthread_t me, int i)
{
	cpuset_t cpumask;

	if (i == -1)
		return 0;

	/* Set thread affinity.*/
	CPU_ZERO(&cpumask);
	CPU_SET(i, &cpumask);

	if (pthread_setaffinity_np(me, sizeof(cpuset_t), &cpumask) != 0) {
		D("Unable to set affinity: %s", strerror(errno));
		return 1;
	}
	return 0;
}

#ifdef VERBOSE_SUPPORT
/* Check the payload of the packet for errors (use it for debug).
 * Look for consecutive ascii representations of the size of the packet.
 */
static void dump_payload(const char *_p, int len, struct netmap_ring *ring, int cur)
{
	char buf[128];
	int i, j, i0;
	const unsigned char *p = (const unsigned char *)_p;

	/* get the length in ASCII of the length of the packet. */

	printf("ring %p cur %5d [buf %6d flags 0x%04x len %5d]\n",
		ring, cur, ring->slot[cur].buf_idx,
		ring->slot[cur].flags, len);
	/* hexdump routine */
	for (i = 0; i < len; ) {
		memset(buf, sizeof(buf), ' ');
		sprintf(buf, "%5d: ", i);
		i0 = i;
		for (j=0; j < 16 && i < len; i++, j++)
			sprintf(buf+7+j*3, "%02x ", (uint8_t)(p[i]));
		i = i0;
		for (j=0; j < 16 && i < len; i++, j++)
			sprintf(buf+7+j + 48, "%c",
				isprint(p[i]) ? p[i] : '.');
		printf("%s\n", buf);
	}
}
#endif /* VERBOSE_SUPPORT */

static inline char * get_l2_hdr(char *buf, int offs)
{
	return (buf + offs);
}

static inline void swap_l2(char *buf, int offs)
{
	uint16_t *eth_hdr;
	register uint16_t tmp;

	eth_hdr = (uint16_t *)get_l2_hdr(buf, offs);
	tmp = eth_hdr[0];
	eth_hdr[0] = eth_hdr[3];
	eth_hdr[3] = tmp;
	tmp = eth_hdr[1];
	eth_hdr[1] = eth_hdr[4];
	eth_hdr[4] = tmp;
	tmp = eth_hdr[2];
	eth_hdr[2] = eth_hdr[5];
	eth_hdr[5] = tmp;
}

static inline void swap_l3(char *buf, int offs)
{
	register uint32_t tmp32;

	buf += offs + 14 + 12;
	tmp32 = ((uint32_t *)buf)[0];
	((uint32_t *)buf)[0] = ((uint32_t *)buf)[1];
	((uint32_t *)buf)[1] = tmp32;
}

#ifdef ADDR_TBL_EMUL
static int get_out_port(struct addr_tbl	*addr_tbl, uint8_t *src_addr)
{
	int i;
	for (i=0; i<addr_tbl->num_ents; i++) {
		uint8_t	*tmp = (uint8_t	*)addr_tbl->entries[i].src_addr;
		if (*(uint32_t *)(tmp) == *(uint32_t *)(src_addr))
			return i;
	}
	return -1;
}
#endif /* ADDR_TBL_EMUL */

/*
 * how many packets on this set of queues ?
 */
int pkt_queued(struct nm_desc *d, int tx)
{
	u_int i, tot = 0;

	if (tx) {
		for (i = d->first_tx_ring; i <= d->last_tx_ring; i++)
			tot += nm_ring_space(NETMAP_TXRING(d->nifp, i));
	} else {
		for (i = d->first_rx_ring; i <= d->last_rx_ring; i++)
			tot += nm_ring_space(NETMAP_RXRING(d->nifp, i));
	}
	return tot;
}

/*
 * move up to 'limit' pkts from rxring to txring swapping buffers.
 */
static int process_rings(
	struct netmap_ring *rxring,
	struct netmap_ring *txring,
	const char *msg,
	struct targ *targ
	)
{
	u_int j, k, m = 0;

	/* print a warning if any of the ring flags is set (e.g. NM_REINIT) */
	if (rxring->flags || txring->flags)
		D("%s rxflags %x txflags %x",
			msg, rxring->flags, txring->flags);
	j = rxring->cur; /* RX */
	k = txring->cur; /* TX */
	m = nm_ring_space(rxring);
	if (m < targ->burst)
		targ->burst = m;
	m = nm_ring_space(txring);
	if (m < targ->burst)
		targ->burst = m;
	m = targ->burst;
	while (targ->burst-- > 0) {
		struct netmap_slot *rs = &rxring->slot[j];
		struct netmap_slot *ts = &txring->slot[k];

		/* swap packets */
		if (ts->buf_idx < 2 || rs->buf_idx < 2) {
			D("wrong index rx[%d] = %d  -> tx[%d] = %d",
				j, rs->buf_idx, k, ts->buf_idx);
			sleep(2);
		}
		/* copy the packet length. */
		if (rs->len > 2048) {
			D("wrong len %d rx[%d] -> tx[%d]", rs->len, j, k);
			rs->len = 0;
		}
#ifdef VERBOSE_SUPPORT
		else if (targ->g->verbose > 1) {
			D("%s send len %d rx[%d] -> tx[%d]", msg, rs->len, j, k);
		}
#endif /* VERBOSE_SUPPORT */
		ts->len = rs->len;
		ts->data_offs = rs->data_offs;

#ifdef ADDR_TBL_EMUL
		if (get_out_port(targ->addr_tbl,
			(uint8_t *)get_l2_hdr(NETMAP_BUF(rxring, rs->buf_idx), rs->data_offs)) != -1)
			printf("found!\n");
#endif /* ADDR_TBL_EMUL */

		if (targ->echo) {
#ifdef USE_APP_PREFETCH
			if (targ->burst >= PREFETCH_SHIFT)
				prefetch(NETMAP_BUF(rxring,
						    rxring->slot[j+PREFETCH_SHIFT].buf_idx) +
					 rxring->slot[j+PREFETCH_SHIFT].data_offs);
#endif /* USE_APP_PREFETCH */
			swap_l2(NETMAP_BUF(rxring, rs->buf_idx), rs->data_offs);
			swap_l3(NETMAP_BUF(rxring, rs->buf_idx), rs->data_offs);
		}

		if (targ->zerocopy) {
			uint32_t pkt = ts->buf_idx;
			ts->buf_idx = rs->buf_idx;
			rs->buf_idx = pkt;
			/* report the buffer change. */
			ts->flags |= NS_BUF_CHANGED;
			rs->flags |= NS_BUF_CHANGED;
		} else {
			char *rxbuf = NETMAP_BUF(rxring, rs->buf_idx);
			char *txbuf = NETMAP_BUF(txring, ts->buf_idx);
			nm_pkt_copy(rxbuf, txbuf, ts->len);
		}
		j = nm_ring_next(rxring, j);
		k = nm_ring_next(txring, k);

#ifdef VERBOSE_SUPPORT
		if ((targ->g->verbose > 1) && targ->g->dump_payload)
			dump_payload((NETMAP_BUF(txring, ts->buf_idx) + ts->data_offs), ts->len , txring, k);
#endif /* VERBOSE_SUPPORT */
	}
	rxring->head = rxring->cur = j;
	txring->head = txring->cur = k;

#ifdef VERBOSE_SUPPORT
	if (targ->g->verbose && m > 0)
		D("%s sent %d packets to %p", msg, m, txring);
#endif /* VERBOSE_SUPPORT */

	return (m);
}

/* move packts from src to destination */
static int move(struct nm_desc *src, struct nm_desc *dst, struct targ *targ)
{
	struct netmap_ring *txring, *rxring;
	u_int m = 0, si = src->first_rx_ring, di = dst->first_tx_ring;
	const char *msg = (src->req.nr_ringid & NETMAP_SW_RING) ?
		"host->net" : "net->host";

	while (si <= src->last_rx_ring && di <= dst->last_tx_ring) {
		if (!((1 << si) & targ->qs_map)) continue;
		rxring = NETMAP_RXRING(src->nifp, si);
		txring = NETMAP_TXRING(dst->nifp, di);
		ND("txring %p rxring %p", txring, rxring);
		if (nm_ring_empty(rxring)) {
			si++;
			continue;
		}
		if (nm_ring_empty(txring)) {
			di++;
			continue;
		}
//printf("th %d, rr %d, tr %d\n",targ->me, rxring->ringid, txring->ringid);
		m += process_rings(rxring, txring, msg, targ);
	}

	return (m);
}

static void * main_loop_lb_thread(void *data)
{
	struct targ	*targ = (struct targ *) data;
	int			tmp;

	if (setaffinity(targ->thread, targ->affinity))
		goto quit;

	while (!targ->cancel) {
		targ->pollfds[0].events = 0;
		targ->pollfds[0].revents = 0;
		tmp = pkt_queued(targ->nmds[0], 0);
#if defined(_WIN32) || defined(BUSY_WAIT)
		if (tmp){
			ioctl(targ->pollfds[0].fd, NIOCTXSYNC, NULL);
			targ->pollfds[0].revents = POLLOUT;
		}
		else {
			ioctl(targ->pollfds[0].fd, NIOCRXSYNC, NULL);
		}
#else
		if (tmp)
			targ->pollfds[0].events |= POLLOUT;
		else
			targ->pollfds[0].events |= POLLIN;
		tmp = poll(targ->pollfds, 1, 2500);
		if ((tmp <= 0)
#ifdef VERBOSE_SUPPORT
			|| targ->g->verbose
#endif /* VERBOSE_SUPPORT */
			)
			D("poll %s ev %x %x rx %d@%d tx %d,",
				tmp <= 0 ? "timeout" : "ok",
				targ->pollfds[0].events,
				targ->pollfds[0].revents,
				pkt_queued(targ->nmds[0], 0),
				NETMAP_RXRING(targ->nmds[0]->nifp, targ->nmds[0]->cur_rx_ring)->cur,
				pkt_queued(targ->nmds[0], 1)
			);
		if (tmp < 0)
			continue;
#endif //defined(_WIN32) || defined(BUSY_WAIT)
		if (targ->pollfds[0].revents & POLLERR) {
			struct netmap_ring *rx = NETMAP_RXRING(targ->nmds[0]->nifp, targ->nmds[0]->cur_rx_ring);
			D("error on fd0, rx [%d,%d,%d)",
				rx->head, rx->cur, rx->tail);
		}
		if (targ->pollfds[0].revents & POLLOUT)
			move(targ->nmds[0], targ->nmds[0], targ);
	}

	targ->completed = 1;

quit:
	/* reset the ``used`` flag. */
	targ->used = 0;

	return (NULL);
}

static void * main_loop_thread(void *data)
{
	struct targ *targ = (struct targ *) data;

	if (setaffinity(targ->thread, targ->affinity))
		goto quit;

	while (!targ->cancel) {
		int n0, n1 = 0, ret;
		targ->pollfds[0].events = targ->pollfds[1].events = 0;
		targ->pollfds[0].revents = targ->pollfds[1].revents = 0;
		n0 = pkt_queued(targ->nmds[0], 0);
		n1 = pkt_queued(targ->nmds[1], 0);
#if defined(_WIN32) || defined(BUSY_WAIT)
		if (n0){
			ioctl(targ->pollfds[1].fd, NIOCTXSYNC, NULL);
			targ->pollfds[1].revents = POLLOUT;
		}
		else {
			ioctl(targ->pollfds[0].fd, NIOCRXSYNC, NULL);
		}
		if (n1){
			ioctl(targ->pollfds[0].fd, NIOCTXSYNC, NULL);
			targ->pollfds[0].revents = POLLOUT;
		}
		else {
			ioctl(targ->pollfds[1].fd, NIOCRXSYNC, NULL);
		}
		ret = 1;
#else
		if (n0)
			targ->pollfds[1].events |= POLLOUT;
		else
			targ->pollfds[0].events |= POLLIN;
		if (n1)
			targ->pollfds[0].events |= POLLOUT;
		else
			targ->pollfds[1].events |= POLLIN;
		ret = poll(targ->pollfds, 2, 2500);
#endif //defined(_WIN32) || defined(BUSY_WAIT)
		if ((ret <= 0)
#ifdef VERBOSE_SUPPORT
			|| targ->g->verbose
#endif /* VERBOSE_SUPPORT */
			)
			D("poll %s [0] ev %x %x rx %d@%d tx %d,"
				" [1] ev %x %x rx %d@%d tx %d",
				ret <= 0 ? "timeout" : "ok",
				targ->pollfds[0].events,
				targ->pollfds[0].revents,
				pkt_queued(targ->nmds[0], 0),
				NETMAP_RXRING(targ->nmds[0]->nifp, targ->nmds[0]->cur_rx_ring)->cur,
				pkt_queued(targ->nmds[0], 1),
				targ->pollfds[1].events,
				targ->pollfds[1].revents,
				pkt_queued(targ->nmds[1], 0),
				NETMAP_RXRING(targ->nmds[1]->nifp, targ->nmds[1]->cur_rx_ring)->cur,
				pkt_queued(targ->nmds[1], 1)
			);
		if (ret < 0)
			continue;
		if (targ->pollfds[0].revents & POLLERR) {
			struct netmap_ring *rx = NETMAP_RXRING(targ->nmds[0]->nifp, targ->nmds[0]->cur_rx_ring);
			D("error on fd0, rx [%d,%d,%d)",
				rx->head, rx->cur, rx->tail);
		}
		if (targ->pollfds[1].revents & POLLERR) {
			struct netmap_ring *rx = NETMAP_RXRING(targ->nmds[1]->nifp, targ->nmds[1]->cur_rx_ring);
			D("error on fd1, rx [%d,%d,%d)",
				rx->head, rx->cur, rx->tail);
		}
		if (targ->pollfds[0].revents & POLLOUT)
			move(targ->nmds[1], targ->nmds[0], targ);
		if (targ->pollfds[1].revents & POLLOUT)
			move(targ->nmds[0], targ->nmds[1], targ);
	}

	targ->completed = 1;

quit:
	/* reset the ``used`` flag. */
	targ->used = 0;

	return (NULL);
}

static void start_threads(struct glob_arg *g)
{
	int i;

	targs = calloc(g->nthreads, sizeof(*targs));
	/*
	 * Now create the desired number of threads, each one
	 * using a single descriptor.
 	 */
	for (i = 0; i < g->nthreads; i++) {
		struct targ *t = &targs[i];

		bzero(t, sizeof(*t));
		t->g = g;
		t->zerocopy = g->zerocopy;
		t->burst = g->burst;
		t->echo = g->echo;
#ifdef ADDR_TBL_EMUL
		t->addr_tbl = &g->addr_tbl;
#endif /* ADDR_TBL_EMUL */

		t->qs_map = (((uint64_t)g->qs_map << 48) |
			     ((uint64_t)g->qs_map << 32) |
			     ((uint64_t)g->qs_map << 16) |
			     ((uint64_t)g->qs_map << 0));
		t->qs_map <<= (g->qs_map_shift * i);

		if (i > 0) {
			struct nm_desc nmd;

			memcpy(&nmd, g->nmds[0], sizeof(nmd)); /* copy, we overwrite ringid */
			nmd.self = &nmd;

			/* the first thread uses the fd opened by the main
			 * thread, the other threads re-open /dev/netmap
			 */
			if (g->nthreads > 1) {
				int j;
				for (j=0; j<64; j++)
					if (t->qs_map & (1<<j))
						break;
				nmd.req.nr_flags =
					g->nmds[0]->req.nr_flags & ~NR_REG_MASK;
				nmd.req.nr_flags |= NR_REG_ONE_NIC;
				nmd.req.nr_ringid = j;
			}

			/* register interface. Override ifname and ringid etc. */
			t->nmds[0] = nm_open(t->g->ifnames[0], NULL,
				NM_OPEN_IFNAME | NM_OPEN_NO_MMAP, &nmd);
			if (t->nmds[0] == NULL) {
				D("Unable to open %s: %s",
					t->g->ifnames[0], strerror(errno));
				continue;
			}

			memcpy(&nmd, g->nmds[1], sizeof(nmd)); /* copy, we overwrite ringid */
			nmd.self = &nmd;

			if (g->loopback)
				t->nmds[1] = t->nmds[0];
			else {
			if (g->nthreads > 1) {
				int j;
				for (j=0; j<64; j++)
					if (t->qs_map & (1<<j))
						break;
				nmd.req.nr_flags =
					g->nmds[1]->req.nr_flags & ~NR_REG_MASK;
				nmd.req.nr_flags |= NR_REG_ONE_NIC;
				nmd.req.nr_ringid = j;
			}

			/* register interface. Override ifname and ringid etc. */
			t->nmds[1] = nm_open(t->g->ifnames[1], NULL,
				NM_OPEN_IFNAME | NM_OPEN_NO_MMAP, &nmd);
			}
			if (t->nmds[1] == NULL) {
				D("Unable to open %s: %s",
					t->g->ifnames[1], strerror(errno));
				continue;
			}
		} else {
			t->nmds[0] = g->nmds[0];
			t->nmds[1] = g->nmds[1];
		}
		t->fds[0] = t->nmds[0]->fd;
		t->fds[1] = t->nmds[1]->fd;
		/* setup poll(2) variables. */
		t->pollfds[0].fd = t->fds[0];
		t->pollfds[1].fd = t->fds[1];

		t->used = 1;
		t->me = i;
		if (g->affinity >= 0) {
			t->affinity = (g->affinity + i) % g->system_cpus;
		} else
			t->affinity = -1;

		if (pthread_create(&t->thread,
				   NULL,
				   (g->loopback ? main_loop_lb_thread : main_loop_thread),
				   t) == -1) {
			D("Unable to create thread %d: %s", i, strerror(errno));
			t->used = 0;
		}
	}
}

static void main_thread(struct glob_arg *g)
{
	int i;

	/* final round */
	for (i = 0; i < g->nthreads; i++) {
		/*
		 * Join active threads, unregister interfaces and close
		 * file descriptors.
		 */
		if (targs[i].used)
			pthread_join(targs[i].thread, NULL); /* blocking */
		close(targs[i].fds[0]);
		close(targs[i].fds[1]);

		if (targs[i].completed == 0)
			D("ouch, thread %d exited with error", i);
	}

	munmap(g->nmds[0]->mem, g->nmds[0]->req.nr_memsize);
	munmap(g->nmds[1]->mem, g->nmds[1]->req.nr_memsize);
	close(g->main_fds[0]);
	close(g->main_fds[1]);
}

static void usage(void)
{
	const char *cmd = "bridge";
	fprintf(stderr,
		"Usage:\n"
		"%s arguments\n"
		"\t-i interface		interface name\n"
		"\t-a cpu_id		use setaffinity\n"
		"\t-b burst size		testing, mostly\n"
		"\t-c cores		cores to use\n"
		"\t-C 			Copy payload\n"
		"\t-m mask:shift	Queues-CPUs mapping mask & shift\n"
		"\t-p threads		processes/threads to use\n"
		"\t-w wait_for_link_time	in seconds\n"
		"\t-X			dump payload\n"
		"",
		cmd);

	exit(1);
}


/*
 * bridge [-v] if1 [if2]
 *
 * If only one name, or the two interfaces are the same,
 * bridges userland and the adapter. Otherwise bridge
 * two intefaces.
 */
int main(int argc, char **argv)
{
	int ch;
	u_int wait_link = 2;
	int i;
	struct glob_arg g;

	fprintf(stderr, "%s built %s %s\n",
		argv[0], __DATE__, __TIME__);

	bzero(&g, sizeof(g));

	g.affinity = -1;
	g.burst = 512;		// default
	g.nthreads = 1;
	g.cpus = 1;		// default
	g.dump_payload = 0;
	g.loopback = 0;
	g.echo = 0;
	g.zerocopy = 1;
	g.qs_map = 0;
	g.qs_map_shift = 0;

#ifdef ADDR_TBL_EMUL
	g.addr_tbl.num_ents = 10;
	{
		int j;
		for (j=0; j<g.addr_tbl.num_ents; j++){
			uint16_t	*tmp = (uint16_t *)g.addr_tbl.entries[j].src_addr;
			tmp[0] = j+1;
			tmp[1] = j+2;
			tmp[2] = j+3;
		}
	}
#endif /* ADDR_TBL_EMUL */

	while ( (ch = getopt(argc, argv,
			"a:b:c:i:m:p:w:eCvX")) != -1) {
		switch (ch) {
		default:
			D("bad option %c %s", ch, optarg);
			usage();
			break;

		case 'a':       /* force affinity */
			g.affinity = atoi(optarg);
			break;
		case 'b':	/* burst */
			g.burst = atoi(optarg);
			break;
		case 'c':
			g.cpus = atoi(optarg);
			break;
		case 'C':
			g.zerocopy = 0; /* do not zerocopy */
			break;
		case 'e':
			g.echo = 1;
			break;
		case 'i':	/* interface */
			if (g.ifnames[0] == NULL)
				g.ifnames[0] = optarg;
			else if (g.ifnames[1] == NULL)
				g.ifnames[1] = optarg;
			else
				D("%s ignored, already have 2 interfaces", optarg);
			break;
		case 'm':
		{
			char *token = strtok(optarg, ":");
			sscanf(token,"%x", (unsigned int *)&g.qs_map); 
			token = strtok(NULL, "");
			g.qs_map_shift = atoi(token);
			break;
		}
		case 'p':
			g.nthreads = atoi(optarg);
			if ((g.nthreads > 1) && !g.qs_map) {
				g.qs_map = 0x01;
				g.qs_map_shift = 1;
			}
			break;
		case 'v':
			g.verbose++;
			break;
		case 'w':
			wait_link = atoi(optarg);
			break;
		case 'X':
			g.dump_payload = 1;
			break;
		}
	}

#ifndef VERBOSE_SUPPORT
	if (g.verbose) {
		D("application was built without debug support; please rebuild.");
		return (1);
	}
#endif /* VERBOSE_SUPPORT */
	g.system_cpus = i = system_ncpus();
	if (g.cpus < 0 || g.cpus > i) {
		D("%d cpus is too high, have only %d cpus", g.cpus, i);
		usage();
		return (1);
	}
	D("running on %d cpus (have %d)", g.cpus, i);
	if (g.cpus == 0)
		g.cpus = i;

	argc -= optind;
	argv += optind;

	if (argc > 1)
		g.ifnames[0] = argv[1];
	if (argc > 2)
		g.ifnames[1] = argv[2];
	if (argc > 3)
		g.burst = atoi(argv[3]);
	if (!g.ifnames[1])
		g.ifnames[1] = g.ifnames[0];
	if (!g.ifnames[0]) {
		D("missing interface");
		usage();
	}
	if (g.burst < 1 || g.burst > 8192) {
		D("invalid burst %d, set to 1024", g.burst);
		g.burst = 1024;
	}
	if (wait_link > 100) {
		D("invalid wait_link %d, set to 4", wait_link);
		wait_link = 4;
	}
	if (!strcmp(g.ifnames[0], g.ifnames[1])) {
		g.loopback = 1;
		D("same interface, send loopback");
	} else {
		/* two different interfaces. Take all rings on if1 */
	}

	g.nmds[0] = nm_open(g.ifnames[0], NULL, 0, NULL);
	if (g.nmds[0] == NULL) {
		D("cannot open %s", g.ifnames[0]);
		return (1);
	}
	if (g.nthreads > 1) {
		struct nm_desc saved_desc = *g.nmds[0];
		uint64_t local_mask = g.qs_map << (g.qs_map_shift * 0);
		int j;
		for (j=0; j<64; j++)
			if (local_mask & (1<<j))
				break;
		saved_desc.self = &saved_desc;
		saved_desc.mem = NULL;
		nm_close(g.nmds[0]);
		saved_desc.req.nr_flags &= ~NR_REG_MASK;
		saved_desc.req.nr_flags |= NR_REG_ONE_NIC;
		saved_desc.req.nr_ringid = j;
		g.nmds[0] = nm_open(g.ifnames[0], NULL, NM_OPEN_IFNAME, &saved_desc);
		if (g.nmds[0] == NULL) {
			D("Unable to open %s: %s", g.ifnames[0], strerror(errno));
			return (1);
		}
	}
	g.main_fds[0] = g.nmds[0]->fd;

	if (g.loopback)
		g.nmds[1] = g.nmds[0];
	else {
		/* try to reuse the mmap() of the first interface, if possible */
		g.nmds[1] = nm_open(g.ifnames[1], NULL, NM_OPEN_NO_MMAP, g.nmds[0]);
		if (g.nmds[1] == NULL) {
			D("cannot open %s", g.ifnames[1]);
			nm_close(g.nmds[0]);
			return (1);
		}

		if (g.nthreads > 1) {
			struct nm_desc saved_desc = *g.nmds[1];
			uint64_t local_mask = g.qs_map << (g.qs_map_shift * 0);
			int j;
			for (j=0; j<64; j++)
				if (local_mask & (1<<j))
					break;
			saved_desc.self = &saved_desc;
			nm_close(g.nmds[1]);
			saved_desc.req.nr_flags &= ~NR_REG_MASK;
			saved_desc.req.nr_flags |= NR_REG_ONE_NIC;
			saved_desc.req.nr_ringid = j;
			g.nmds[1] = nm_open(g.ifnames[1], NULL, NM_OPEN_IFNAME | NM_OPEN_NO_MMAP, &saved_desc);
			if (g.nmds[1] == NULL) {
				D("Unable to open %s: %s", g.ifnames[1], strerror(errno));
				return (1);
			}
		}
	}
	g.main_fds[1] = g.nmds[1]->fd;

	if (!g.qs_map) {
		g.qs_map = 0xf;
		g.qs_map_shift = 4;
	}

	g.zerocopy = g.zerocopy && (g.nmds[0]->mem == g.nmds[1]->mem);
	D("------- zerocopy %ssupported", g.zerocopy ? "" : "NOT ");

	/* validate provided nthreads. */
	if (g.nthreads < 1 || g.nthreads > g.nmds[0]->req.nr_rx_rings) {
		D("bad nthreads %d, have %d queues", g.nthreads, g.nmds[0]->req.nr_rx_rings);
		return (1);
	}

	global_nthreads = g.nthreads;
	signal(SIGINT, sigint_h);

	start_threads(&g);

	D("Wait %d secs for link to come up...", wait_link);
	sleep(wait_link);
	D("Ready to go, %s 0x%x/%d <-> %s 0x%x/%d.",
		g.nmds[0]->req.nr_name, g.nmds[0]->first_rx_ring, g.nmds[0]->req.nr_rx_rings,
		g.nmds[1]->req.nr_name, g.nmds[1]->first_rx_ring, g.nmds[1]->req.nr_rx_rings);

	main_thread(&g);

	return (0);
}
