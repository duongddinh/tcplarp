/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef LARP_CORE_H
#define LARP_CORE_H

/* The decision logic is shared with the native unit tests. Units: bytes/s, us. */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/math64.h>
typedef u64 larp_u64;
typedef u32 larp_u32;
#define larp_div(a, b) div64_u64((a), (b))
#else
#include <stdint.h>
typedef uint64_t larp_u64;
typedef uint32_t larp_u32;
#define larp_div(a, b) ((a) / (b))
#endif

enum larp_mode { LARP_LAUNCH, LARP_RELAX, LARP_PROBE, LARP_PROBE_RTT };
enum larp_recovery { LARP_OPEN, LARP_CWR, LARP_RECOVERY, LARP_RECOVERY_GROW, LARP_TIMEOUT };

struct larp_model {
	larp_u64 rate, ceiling, floor, delivery_rate;
	larp_u64 previous_delivery_rate, filter_stamp_us;
	larp_u64 last_cut_us, last_probe_us;
	larp_u32 min_rtt_us, delay_threshold_us;
	larp_u32 mode, have_cut;
	larp_u32 delay_reference_us;
	larp_u32 learned;
	larp_u32 last_rtt_us, pending_congestion, rtt_window_us;
	larp_u32 refresh_active, refresh_min_us, refresh_samples;
	larp_u64 rtt_stamp_us, refresh_start_us, low_flight_start_us;
};

struct larp_feedback {
	larp_u64 now_us, delivery_rate, sample_start_us;
	larp_u32 rtt_us, inflight;
	int ecn, loss, valid_delivery, app_limited;
	int in_recovery, window_limited;
};

static inline larp_u64 larp_min(larp_u64 a, larp_u64 b)
{
	return a < b ? a : b;
}

static inline larp_u64 larp_max(larp_u64 a, larp_u64 b)
{
	return a > b ? a : b;
}

static inline void larp_start(struct larp_model *m, larp_u64 rate,
		larp_u64 floor, larp_u32 rtt_us, larp_u32 threshold_us,
		larp_u64 now_us)
{
	*m = (struct larp_model) {
		.rate = rate, .ceiling = rate, .floor = floor,
		.min_rtt_us = rtt_us, .delay_threshold_us = threshold_us,
		.last_probe_us = now_us, .mode = LARP_LAUNCH,
		.filter_stamp_us = now_us,
		.last_rtt_us = rtt_us, .rtt_stamp_us = now_us,
		.rtt_window_us = 10000000,
	};
}

/* Saturating arithmetic keeps unrestricted learned rates from wrapping. */
static inline larp_u64 larp_add(larp_u64 a, larp_u64 b)
{
	return a > ~(larp_u64)0 - b ? ~(larp_u64)0 : a + b;
}

static inline larp_u64 larp_scale(larp_u64 a, larp_u32 numerator,
		larp_u32 denominator)
{
	larp_u64 whole = larp_div(a, denominator);
	if (whole > larp_div(~(larp_u64)0, numerator))
		return ~(larp_u64)0;
	return larp_add(whole * numerator,
		larp_div((a % denominator) * numerator, denominator));
}

static inline void larp_update(struct larp_model *m,
		const struct larp_feedback *f)
{
	larp_u64 next, gap, bandwidth, filter_span;
	larp_u32 threshold;
	int queue_high, delay_event;

	if (f->rtt_us) m->last_rtt_us = f->rtt_us;
	if (f->rtt_us && (!m->min_rtt_us || f->rtt_us < m->min_rtt_us)) {
		m->min_rtt_us = f->rtt_us;
		m->rtt_stamp_us = f->now_us;
	}
	if (f->ecn || f->loss) m->pending_congestion = 1;
	gap = larp_max(m->min_rtt_us, 1000);
	/* Keep the maximum over two two-RTT buckets. Startup/ACK gaps cannot
	 * drag an EWMA downward, and obsolete capacity expires within four RTTs. */
	filter_span = gap * 2;
	if (f->in_recovery) {
		/* Loss repair is sender/window limited. Preserve demonstrated
		 * capacity instead of aging it into a recovery-limited rate. */
		m->filter_stamp_us = f->now_us;
	} else if (f->now_us - m->filter_stamp_us >= filter_span) {
		m->previous_delivery_rate =
			f->now_us - m->filter_stamp_us < filter_span * 2 ?
			m->delivery_rate : 0;
		m->delivery_rate = 0;
		m->filter_stamp_us = f->now_us;
	}
	if (f->valid_delivery && f->delivery_rate && !f->app_limited)
		m->delivery_rate = larp_max(m->delivery_rate, f->delivery_rate);
	/* A measured sample grants permission to grow the flight budget; it
	 * does not certify link capacity. Repair/app-limited samples cannot. */
	if (f->valid_delivery && f->delivery_rate && f->rtt_us &&
	    !f->app_limited && !f->in_recovery)
		m->learned = 1;
	bandwidth = larp_max(m->delivery_rate, m->previous_delivery_rate);
	threshold = (larp_u32)larp_max(m->delay_threshold_us,
					larp_div(m->min_rtt_us, 8));
	queue_high = f->rtt_us && m->min_rtt_us &&
		f->rtt_us > (larp_u64)m->min_rtt_us + threshold;
	/* An unchanged standing queue is not a new queue-growth event on each
	 * ACK. Rearm only after draining close to the baseline; further growth
	 * of another threshold also warrants a new drain. The hysteresis keeps
	 * RTT noise near the high threshold from repeatedly rearming cuts. */
	if (f->rtt_us && f->rtt_us <= (larp_u64)m->min_rtt_us + threshold / 2)
		m->delay_reference_us = 0;
    /* Explicit signals are latched across the once-per-base-RTT cut guard. */
    if (m->pending_congestion) {
        if (!m->have_cut || f->now_us-m->last_cut_us>=gap) {
            m->rate=larp_max(m->floor,larp_scale(m->rate,7,10));
            m->last_cut_us=m->last_probe_us=f->now_us;
            m->have_cut=1; m->pending_congestion=0; m->mode=LARP_RELAX;
            if (queue_high) m->delay_reference_us=f->rtt_us;
        }
        return;
    }

    /* An expired baseline is remeasured with <=4 packets outstanding. Never
     * replace it merely because a timer expired or the current RTT is high. */
    if (!m->refresh_active && m->learned && f->rtt_us &&
        !f->app_limited && !f->in_recovery &&
        f->now_us-m->rtt_stamp_us>=m->rtt_window_us) {
        m->refresh_active=1; m->refresh_start_us=f->now_us;
        m->low_flight_start_us=0; m->refresh_min_us=0; m->refresh_samples=0;
    }
    if (m->refresh_active) {
        m->mode=LARP_PROBE_RTT;
        if (f->rtt_us && !f->in_recovery && f->inflight<=4 &&
            f->now_us-m->refresh_start_us>=larp_max(m->last_rtt_us,1000)) {
            if (!m->low_flight_start_us) m->low_flight_start_us=f->now_us;
            m->refresh_min_us=m->refresh_min_us ?
                (larp_u32)larp_min(m->refresh_min_us,f->rtt_us) : f->rtt_us;
            if (m->refresh_samples<1000000) m->refresh_samples++;
        }
        if (m->low_flight_start_us && m->refresh_samples>=3 &&
            f->now_us-m->low_flight_start_us>=larp_max(200000,m->refresh_min_us)) {
            m->min_rtt_us=m->refresh_min_us;
            m->rtt_stamp_us=f->now_us; m->refresh_active=0;
            m->delay_reference_us=0;
            m->delivery_rate=m->previous_delivery_rate=0;
            m->filter_stamp_us=m->last_probe_us=f->now_us;
        } else if (f->now_us-m->refresh_start_us>=
                   larp_max(2000000,(larp_u64)m->last_rtt_us*4)) {
            /* No usable low-flight evidence: retain the old baseline and
             * release the small window. Retry after another full window. */
            m->refresh_active=0; m->rtt_stamp_us=f->now_us;
            m->last_probe_us=f->now_us;
        }
        return;
    }

	delay_event = queue_high && (!m->delay_reference_us ||
		f->rtt_us > (larp_u64)m->delay_reference_us + threshold);
	/* A low delivered rate is not proof of congestion. Only a new queue
	 * growth event, ECN, or loss can trigger a downward correction. */
	if (delay_event) {
		/* One reduction per base RTT prevents repeated ECE/queue feedback
		 * from collapsing the rate on every ACK. The first cut is immediate. */
		if (!m->have_cut || f->now_us - m->last_cut_us >= gap) {
			next = larp_scale(m->rate, 7, 10);
			/* A delay-only drain can retain a delivery-based floor. ECN
			 * and loss must yield even when the recent peak is high: that
			 * peak can reflect a previously monopolized bottleneck. */
			if (bandwidth && !f->loss && !f->ecn)
				next = larp_max(next, larp_scale(bandwidth, 9, 10));
			next = larp_min(next, m->rate);
			m->rate = larp_max(m->floor, next);
			m->last_cut_us = f->now_us;
			m->last_probe_us = f->now_us;
			m->have_cut = 1;
			m->mode = LARP_RELAX;
			if (queue_high)
				m->delay_reference_us = f->rtt_us;
		}
		return;
	}
	/* Unknown current RTT cannot authorize growth. */
	if (f->rtt_us && f->now_us - m->last_probe_us >= gap && !f->app_limited && !f->in_recovery) {
		/* Stable elevated RTT must not permanently disable probing. Probe
		 * at 5% per RTT while queued, rather than making the fast 25%
		 * recovery step or jumping back to a stale monopolistic peak. */
		larp_u64 ceiling = m->ceiling;
		next = larp_add(m->rate, larp_max(larp_div(m->rate, queue_high ? 20 : 4), 1));
		if (bandwidth && !queue_high)
			next = larp_max(next, larp_scale(bandwidth, 102, 100));
		/* Above the launch guess, grow only with a current eligible sample
		 * and low queue delay. Bound each extension by 1.25 times recent
		 * delivery. This bound moves with feedback, not a fixed Gbps value.
		 * A stale/small estimate must never silently cut the current rate. */
		ceiling = larp_max(ceiling, m->rate);
		if (m->learned && f->valid_delivery && f->delivery_rate && !queue_high)
			ceiling = larp_max(ceiling, larp_scale(bandwidth, 5, 4));
		m->rate = larp_min(ceiling, next);
		m->last_probe_us = f->now_us;
		m->mode = LARP_PROBE;
	}
}

static inline larp_u32 larp_window(const struct larp_model *m,
		larp_u32 mss, larp_u32 cap)
{
	larp_u64 bytes, packets;
	if (m->refresh_active) return (larp_u32)larp_min(cap, 4);
	/* Saturate before multiplying at extreme rates/RTTs. */
	bytes = larp_scale(larp_scale(m->rate, 2, 1),
		larp_max(m->min_rtt_us, 1000), 1000000);
	packets = larp_add(larp_div(bytes, mss), !!(bytes % mss));
	return (larp_u32)larp_min(larp_max(packets, 4), cap);
}

static inline larp_u32 larp_ack_window(larp_u32 cwnd, larp_u32 target,
		larp_u32 inflight, larp_u32 acked, larp_u32 ssthresh,
		enum larp_recovery recovery)
{
	if (recovery == LARP_TIMEOUT)
		/* TCP resets cwnd to one on RTO entry. Subsequent ACKs must
		 * grow it again; freezing at one until Loss exits serializes
		 * repair of the entire old flight at one packet per RTT. */
		return (larp_u32)larp_max(1, larp_min(cwnd + acked, target));
	if (recovery == LARP_RECOVERY)
		/* Replace ACKed packets, even if earlier CWR/RTO left cwnd tiny.
		 * A cong_control callback must perform packet conservation itself. */
		return (larp_u32)larp_min(target, larp_max(inflight + acked, 2));
	if (recovery == LARP_RECOVERY_GROW)
		/* After the first recovery RTT, at most two replacements per ACK
		 * allow repair to grow beyond a tiny packet-conservation window. */
		return (larp_u32)larp_min(target, larp_max(cwnd + acked, inflight + acked));
	if (recovery == LARP_CWR)
		return (larp_u32)larp_max(1, larp_min(larp_min(target, ssthresh),
				larp_max(cwnd, inflight + acked)));
	return (larp_u32)larp_max(1, larp_min(target, cwnd + acked));
}
#endif
