// SPDX-License-Identifier: GPL-2.0-only
/* TCP LARP research prototype. Load only in a disposable Linux test guest. */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <net/tcp.h>
#include "larp_core.h"

static unsigned int initial_mbps = 1000;
module_param(initial_mbps, uint, 0444);
MODULE_PARM_DESC(initial_mbps, "Speculative initial pacing rate, Mbps (1..100000)");
static unsigned int flight_cap = 512;
module_param(flight_cap, uint, 0444);
MODULE_PARM_DESC(flight_cap, "Initial flight limit in segments (4..65535), released after qualified feedback");
static unsigned int delay_threshold_us = 500;
module_param(delay_threshold_us, uint, 0444);
MODULE_PARM_DESC(delay_threshold_us, "Minimum RTT growth threshold, microseconds");
static unsigned int rtt_window_ms = 10000;
module_param(rtt_window_ms, uint, 0444);
MODULE_PARM_DESC(rtt_window_ms, "Baseline RTT revalidation interval (1000..600000 ms)");
static bool trace_events;
module_param(trace_events, bool, 0444);
MODULE_PARM_DESC(trace_events, "Record RTT revalidation transitions in guest kernel log");
static atomic_t next_connection = ATOMIC_INIT(0);

struct larp {
	struct larp_model *model;
	u64 recovery_start_us;
	u32 prior_cwnd;
	u32 connection_id;
	u8 ecn_pending, loss_pending;

};

static void larp_apply(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct larp *ca = inet_csk_ca(sk);
	u32 cap = ca->model && ca->model->learned ?
		tp->snd_cwnd_clamp : flight_cap;
	u32 target = larp_window(ca->model, max(tp->mss_cache, 1U), cap);
	u8 state = inet_csk(sk)->icsk_ca_state;

	target = min(target, tp->snd_cwnd_clamp);
	/* Only initialization may jump directly to the flight cap. */
	if (rs)
		target = larp_ack_window(tp->snd_cwnd, target,
			tcp_packets_in_flight(tp), rs->acked_sacked, tp->snd_ssthresh,
			state == TCP_CA_Loss ? LARP_TIMEOUT :
			state == TCP_CA_Recovery ?
				(tcp_clock_us() - ca->recovery_start_us >=
				 max(ca->model->min_rtt_us, 1000U) ?
				 LARP_RECOVERY_GROW : LARP_RECOVERY) :
			state == TCP_CA_CWR ? LARP_CWR : LARP_OPEN);
	tp->snd_cwnd = max(target, 1U);
	WRITE_ONCE(sk->sk_pacing_rate,
		min_t(u64, ca->model->rate, sk->sk_max_pacing_rate));
}

static void larp_init(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct larp *ca = inet_csk_ca(sk);

	memset(ca, 0, sizeof(*ca));
	ca->model = kzalloc(sizeof(*ca->model), GFP_ATOMIC);
    if (!ca->model) {
        pr_warn_once("LARP: model allocation failed; conservative two-packet fallback\n");
        tp->snd_cwnd = 2;
        return;
    }
	larp_start(ca->model, (u64)initial_mbps * 125000, 125000,
		   tp->srtt_us >> 3, delay_threshold_us, tcp_clock_us());
    ca->model->rtt_window_us = rtt_window_ms * 1000U;
    ca->connection_id = atomic_inc_return(&next_connection);
	ca->prior_cwnd = tp->snd_cwnd;
	tp->snd_ssthresh = TCP_INFINITE_SSTHRESH;
	cmpxchg(&sk->sk_pacing_status, SK_PACING_NONE, SK_PACING_NEEDED);
	larp_apply(sk, NULL);
}

static void larp_release(struct sock *sk)
{
    kfree(((struct larp *)inet_csk_ca(sk))->model);
}

static void larp_ack_event(struct sock *sk, u32 flags)
{
	struct larp *ca = inet_csk_ca(sk);
	if (flags & CA_ACK_ECE)
		ca->ecn_pending = 1;
}

static u32 larp_ssthresh(struct sock *sk)
{
	struct larp *ca = inet_csk_ca(sk);
	ca->prior_cwnd = tcp_sk(sk)->snd_cwnd;
	/* TCP invokes this for ECN CWR as well as loss recovery. The callback
	 * itself does not prove loss; use rate_sample.losses and timeout state. */
	return max(tcp_sk(sk)->snd_cwnd / 2, 2U);
}

static void larp_set_state(struct sock *sk, u8 state)
{
	struct larp *ca = inet_csk_ca(sk);
	if (state == TCP_CA_Loss || state == TCP_CA_Recovery)
		ca->loss_pending = 1;
	if (state == TCP_CA_Recovery)
		ca->recovery_start_us = tcp_clock_us();
}

static u32 larp_undo_cwnd(struct sock *sk)
{
	struct larp *ca = inet_csk_ca(sk);
	return max(tcp_sk(sk)->snd_cwnd, ca->prior_cwnd);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
static void larp_cong_control(struct sock *sk, u32 ack, int flag,
			      const struct rate_sample *rs)
#else
static void larp_cong_control(struct sock *sk, const struct rate_sample *rs)
#endif
{
	struct larp *ca = inet_csk_ca(sk);
    u32 old_min = ca->model ? ca->model->min_rtt_us : 0;
    bool refreshing = ca->model && ca->model->refresh_active;
    if (!ca->model) {
        tcp_sk(sk)->snd_cwnd = 2;
        WRITE_ONCE(sk->sk_pacing_rate, min_t(u64, 125000, sk->sk_max_pacing_rate));
        return;
    }
	u8 state = inet_csk(sk)->icsk_ca_state;
	struct larp_feedback f = {
		.now_us = tcp_clock_us(),
		.rtt_us = rs->rtt_us > 0 ? rs->rtt_us : 0,
		.ecn = ca->ecn_pending,
		/* Losses marked across several ACKs can belong to one old flight.
		 * Penalize recovery entry once; packet conservation repairs it. */
		.loss = ca->loss_pending || (state < TCP_CA_Recovery && rs->losses > 0),
		.in_recovery = state >= TCP_CA_Recovery,
		.app_limited = rs->is_app_limited,
        .inflight = tcp_packets_in_flight(tcp_sk(sk)),
        .window_limited = rs->prior_in_flight >= tcp_sk(sk)->snd_cwnd,
	};

	/* Filter single-ACK samples; rate_sample uses the longer of the sender
	 * and receiver intervals, rather than raw ACK spacing alone. */
	if (rs->delivered >= 4 && rs->interval_us > 0 &&
	    rs->interval_us >= max(ca->model->min_rtt_us / 8, 1U)) {
		f.valid_delivery = 1;
        f.sample_start_us = f.now_us >= rs->interval_us ? f.now_us-rs->interval_us : 0;
		f.delivery_rate = div64_u64((u64)rs->delivered *
			tcp_sk(sk)->mss_cache * USEC_PER_SEC, rs->interval_us);
	}
	larp_update(ca->model, &f);
    if (trace_events && refreshing != !!ca->model->refresh_active) {
        pr_info("LARP_EVENT id=%u event=rtt_%s old_us=%u new_us=%u now_us=%llu\n",
            ca->connection_id, ca->model->refresh_active ? "begin" : "end",
            old_min, ca->model->min_rtt_us, f.now_us);
    }
	ca->ecn_pending = ca->loss_pending = 0;
	larp_apply(sk, rs);
}

#define LARP_OPS(label, init_fn) { .init = init_fn, .ssthresh = larp_ssthresh, \
	.release = larp_release, .undo_cwnd = larp_undo_cwnd, .in_ack_event = larp_ack_event, \
	.set_state = larp_set_state, .cong_control = larp_cong_control, \
	.owner = THIS_MODULE, .name = label }

/* Only the fixed feedback-adaptive controller is registered. */
static struct tcp_congestion_ops larp_ops[] __read_mostly = {
	LARP_OPS("larp", larp_init),
};

static int __init larp_register(void)
{
	int i, err;
	BUILD_BUG_ON(sizeof(struct larp) > ICSK_CA_PRIV_SIZE);
	if (initial_mbps < 1 || initial_mbps > 100000 ||
	    flight_cap < 4 || flight_cap > 65535 ||
        rtt_window_ms < 1000 || rtt_window_ms > 600000)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(larp_ops); ++i) {
		err = tcp_register_congestion_control(&larp_ops[i]);
		if (err) {
			while (i--)
				tcp_unregister_congestion_control(&larp_ops[i]);
			return err;
		}
	}
	return 0;
}

static void __exit larp_unregister(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(larp_ops); ++i)
		tcp_unregister_congestion_control(&larp_ops[i]);
}
module_init(larp_register);
module_exit(larp_unregister);
MODULE_AUTHOR("TCP LARP project");
MODULE_DESCRIPTION("Speculative paced launch with early rate relaxation (research)");
MODULE_LICENSE("GPL");
