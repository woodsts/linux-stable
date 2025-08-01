/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __NETNS_CONNTRACK_H
#define __NETNS_CONNTRACK_H

#include <linux/list.h>
#include <linux/list_nulls.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/netfilter/nf_conntrack_tcp.h>
#ifdef CONFIG_NF_CT_PROTO_SCTP
#include <linux/netfilter/nf_conntrack_sctp.h>
#endif
#include <linux/seqlock.h>

struct ctl_table_header;
struct nf_conntrack_ecache;

struct nf_generic_net {
	unsigned int timeout;
};

struct nf_tcp_net {
	unsigned int timeouts[TCP_CONNTRACK_TIMEOUT_MAX];
	u8 tcp_loose;
	u8 tcp_be_liberal;
	u8 tcp_max_retrans;
	u8 tcp_ignore_invalid_rst;
#if IS_ENABLED(CONFIG_NF_FLOW_TABLE)
	unsigned int offload_timeout;
#endif
};

enum udp_conntrack {
	UDP_CT_UNREPLIED,
	UDP_CT_REPLIED,
	UDP_CT_MAX
};

struct nf_udp_net {
	unsigned int timeouts[UDP_CT_MAX];
#if IS_ENABLED(CONFIG_NF_FLOW_TABLE)
	unsigned int offload_timeout;
#endif
};

struct nf_icmp_net {
	unsigned int timeout;
};

#ifdef CONFIG_NF_CT_PROTO_SCTP
struct nf_sctp_net {
	unsigned int timeouts[SCTP_CONNTRACK_MAX];
};
#endif

#ifdef CONFIG_NF_CT_PROTO_GRE
enum gre_conntrack {
	GRE_CT_UNREPLIED,
	GRE_CT_REPLIED,
	GRE_CT_MAX
};

struct nf_gre_net {
	struct list_head	keymap_list;
	unsigned int		timeouts[GRE_CT_MAX];
};
#endif

struct nf_ip_net {
	struct nf_generic_net   generic;
	struct nf_tcp_net	tcp;
	struct nf_udp_net	udp;
	struct nf_icmp_net	icmp;
	struct nf_icmp_net	icmpv6;
#ifdef CONFIG_NF_CT_PROTO_SCTP
	struct nf_sctp_net	sctp;
#endif
#ifdef CONFIG_NF_CT_PROTO_GRE
	struct nf_gre_net	gre;
#endif
};

struct netns_ct {
#ifdef CONFIG_NF_CONNTRACK_EVENTS
	bool ecache_dwork_pending;
#endif
	u8			sysctl_log_invalid; /* Log invalid packets */
	u8			sysctl_events;
	u8			sysctl_acct;
	u8			sysctl_tstamp;
	u8			sysctl_checksum;

	struct ip_conntrack_stat __percpu *stat;
	struct nf_ct_event_notifier __rcu *nf_conntrack_event_cb;
	struct nf_ip_net	nf_ct_proto;
#if defined(CONFIG_NF_CONNTRACK_LABELS)
	atomic_t		labels_used;
#endif
};
#endif
