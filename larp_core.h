/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef LARP_CORE_H
#define LARP_CORE_H
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/math64.h>
typedef u64 larp_u64;
typedef u32 larp_u32;
#define larp_div(a,b) div64_u64((a),(b))
#else
#include <stdint.h>
typedef uint64_t larp_u64;
typedef uint32_t larp_u32;
#define larp_div(a,b) ((a)/(b))
#endif

/* Units: bytes/s and monotonic microseconds. Shared with native regressions. */
enum larp_mode { LARP_LAUNCH, LARP_RELAX, LARP_PROBE, LARP_PROBE_RTT };
enum larp_recovery { LARP_OPEN, LARP_CWR, LARP_RECOVERY, LARP_RECOVERY_GROW, LARP_TIMEOUT };
struct larp_model {
    larp_u64 rate, ceiling, floor, delivery_rate, previous_delivery_rate;
    larp_u64 filter_stamp_us, last_cut_us, last_probe_us, rtt_stamp_us;
    larp_u64 probe_start_us, probe_base_rate, probe_before_rate, probe_peak_rate;
    larp_u64 next_probe_us, refresh_start_us, low_flight_start_us;
    larp_u32 min_rtt_us, last_rtt_us, delay_threshold_us, delay_reference_us;
    larp_u32 learned, mode, have_cut, pending_congestion;
    larp_u32 probe_active, probe_rtt_us, probe_min_rtt_us, probe_samples;
    larp_u32 refresh_active, refresh_min_us, refresh_samples, rtt_window_us;
};
struct larp_feedback {
    larp_u64 now_us, delivery_rate, sample_start_us;
    larp_u32 rtt_us, inflight;
    int ecn, loss, valid_delivery, app_limited, in_recovery, window_limited;
};
static inline larp_u64 larp_min(larp_u64 a, larp_u64 b) { return a < b ? a : b; }
static inline larp_u64 larp_max(larp_u64 a, larp_u64 b) { return a > b ? a : b; }
static inline larp_u64 larp_add(larp_u64 a, larp_u64 b)
{ return a > ~(larp_u64)0-b ? ~(larp_u64)0 : a+b; }
static inline larp_u64 larp_scale(larp_u64 a, larp_u32 n, larp_u32 d)
{
    larp_u64 whole=larp_div(a,d);
    if (whole > larp_div(~(larp_u64)0,n)) return ~(larp_u64)0;
    return larp_add(whole*n,larp_div((a%d)*n,d));
}
static inline void larp_start(struct larp_model *m, larp_u64 rate,
    larp_u64 floor, larp_u32 rtt, larp_u32 threshold, larp_u64 now)
{
    *m=(struct larp_model){.rate=rate,.ceiling=rate,.floor=floor,
        .min_rtt_us=rtt,.last_rtt_us=rtt,.delay_threshold_us=threshold,
        .last_probe_us=now,.filter_stamp_us=now,.rtt_stamp_us=now,
        .rtt_window_us=10000000,.mode=LARP_LAUNCH};
}
static inline void larp_cancel_probe(struct larp_model *m)
{
    if (m->probe_active) m->rate=m->probe_base_rate;
    m->probe_active=0;
}
static inline void larp_update(struct larp_model *m, const struct larp_feedback *f)
{
    larp_u64 gap, span, bandwidth, next;
    larp_u32 threshold;
    int fresh, high, event;
    if (f->rtt_us) m->last_rtt_us=f->rtt_us;
    if (f->rtt_us && (!m->min_rtt_us || f->rtt_us<m->min_rtt_us)) {
        m->min_rtt_us=f->rtt_us;
        m->rtt_stamp_us=f->now_us;
    }
    gap=larp_max(m->min_rtt_us,1000);
    threshold=(larp_u32)larp_max(m->delay_threshold_us,larp_div(gap,8));
    fresh=f->valid_delivery && f->delivery_rate && f->rtt_us &&
        !f->app_limited && !f->in_recovery;
    if (fresh) m->learned=1;
    if (f->ecn || f->loss) m->pending_congestion=1;

    /* Explicit signals are latched across the once-per-base-RTT cut guard. */
    if (m->pending_congestion) {
        larp_cancel_probe(m);
        if (!m->have_cut || f->now_us-m->last_cut_us>=gap) {
            m->rate=larp_max(m->floor,larp_scale(m->rate,7,10));
            m->last_cut_us=m->last_probe_us=f->now_us;
            m->have_cut=1; m->pending_congestion=0; m->mode=LARP_RELAX;
            m->next_probe_us=larp_add(f->now_us,2*gap);
        }
        return;
    }

    /* An expired baseline is remeasured with <=4 packets outstanding. Never
     * replace it merely because a timer expired or the current RTT is high. */
    if (!m->refresh_active && m->learned && f->rtt_us &&
        !f->app_limited && !f->in_recovery &&
        f->now_us-m->rtt_stamp_us>=m->rtt_window_us) {
        larp_cancel_probe(m);
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
            m->next_probe_us=larp_add(f->now_us,larp_max(m->min_rtt_us,1000));
        } else if (f->now_us-m->refresh_start_us>=
                   larp_max(2000000,(larp_u64)m->last_rtt_us*4)) {
            /* No usable low-flight evidence: retain the old baseline and
             * release the small window. Retry after another full window. */
            m->refresh_active=0; m->rtt_stamp_us=f->now_us;
            m->last_probe_us=f->now_us;
        }
        return;
    }

    span=2*gap;
    if (f->in_recovery) m->filter_stamp_us=f->now_us;
    else if (f->now_us-m->filter_stamp_us>=span) {
        m->previous_delivery_rate=f->now_us-m->filter_stamp_us<2*span ? m->delivery_rate:0;
        m->delivery_rate=0; m->filter_stamp_us=f->now_us;
    }
    if (fresh) m->delivery_rate=larp_max(m->delivery_rate,f->delivery_rate);
    bandwidth=larp_max(m->delivery_rate,m->previous_delivery_rate);
    high=f->rtt_us && f->rtt_us>(larp_u64)m->min_rtt_us+threshold;
    if (f->rtt_us && f->rtt_us<=(larp_u64)m->min_rtt_us+threshold/2)
        m->delay_reference_us=0;
    event=high && (!m->delay_reference_us ||
        f->rtt_us>(larp_u64)m->delay_reference_us+threshold);
    if (event) {
        larp_cancel_probe(m);
        if (!m->have_cut || f->now_us-m->last_cut_us>=gap) {
            /* A peak from a previously monopolized share is not a cut floor. */
            m->rate=larp_max(m->floor,larp_scale(m->rate,7,10));
            m->last_cut_us=m->last_probe_us=f->now_us;
            m->have_cut=1; m->mode=LARP_RELAX;
            m->delay_reference_us=f->rtt_us;
            m->next_probe_us=larp_add(f->now_us,2*gap);
        }
        return;
    }
    if (m->probe_active) {
        /* Ignore samples spanning the launch/settling interval. This does
         * not reconstruct individual packet identities; feedback is timed. */
        if (fresh && f->sample_start_us>=m->probe_start_us+m->probe_rtt_us) {
            m->probe_peak_rate=larp_max(m->probe_peak_rate,f->delivery_rate);
            m->probe_min_rtt_us=m->probe_min_rtt_us ?
                (larp_u32)larp_min(m->probe_min_rtt_us,f->rtt_us):f->rtt_us;
            if (m->probe_samples<1000000) m->probe_samples++;
        }
        if (f->now_us-m->probe_start_us>=3*(larp_u64)m->probe_rtt_us) {
            int keep=m->probe_samples>=3 &&
                m->probe_peak_rate>=larp_scale(m->probe_before_rate,102,100) &&
                m->probe_min_rtt_us<=(larp_u64)m->probe_rtt_us+threshold/2;
            if (!keep) m->rate=m->probe_base_rate;
            m->probe_active=0; m->last_probe_us=f->now_us;
            m->next_probe_us=larp_add(f->now_us,(keep?4:8)*gap);
            m->mode=keep?LARP_PROBE:LARP_RELAX;
        }
        return;
    }
    /* Missing RTT is unknown, not a clear queue. This gate applies to ALL
     * rate increases, including those still below the initial guess. */
    if (!fresh || f->now_us-m->last_probe_us<gap || f->now_us<m->next_probe_us)
        return;
    if (high) {
        /* Reconcile a large request/delivery mismatch only with sustained
         * congestion and no known congestion-window bottleneck. */
        if (!f->window_limited && f->now_us-m->last_probe_us>=
            2*(larp_u64)larp_max(f->rtt_us,1000) &&
            f->delivery_rate<larp_scale(m->rate,4,5)) {
            next=larp_max(larp_scale(m->rate,7,10),larp_scale(f->delivery_rate,105,100));
            m->rate=larp_max(m->floor,larp_min(m->rate,next));
            m->last_probe_us=f->now_us; m->mode=LARP_RELAX;
            return;
        }
        /* One bounded 5% trial; no blind ladder back to R0. A useful stable
         * delivery sample can support a trial ABOVE R0 despite elevated RTT. */
        if (f->delivery_rate<larp_scale(m->rate,4,5)) return;
        next=larp_min(larp_scale(m->rate,105,100),larp_scale(f->delivery_rate,110,100));
        if (next<=m->rate) return;
        m->probe_active=1; m->probe_start_us=f->now_us;
        m->probe_base_rate=m->rate; m->probe_before_rate=f->delivery_rate;
        m->probe_rtt_us=(larp_u32)larp_max(f->rtt_us,1000);
        m->probe_peak_rate=0; m->probe_samples=0; m->probe_min_rtt_us=0;
        m->rate=next; m->mode=LARP_PROBE;
        return;
    }
    next=larp_max(larp_scale(m->rate,5,4),larp_scale(bandwidth,102,100));
    m->rate=larp_min(next,larp_max(larp_max(m->ceiling,m->rate),larp_scale(bandwidth,5,4)));
    m->last_probe_us=f->now_us; m->mode=LARP_PROBE;
}
static inline larp_u32 larp_window(const struct larp_model *m, larp_u32 mss, larp_u32 cap)
{
    larp_u64 bytes,packets;
    if (m->refresh_active) return (larp_u32)larp_min(cap,4);
    bytes=larp_scale(larp_scale(m->rate,2,1),larp_max(m->min_rtt_us,1000),1000000);
    packets=larp_add(larp_div(bytes,mss),!!(bytes%mss));
    return (larp_u32)larp_min(larp_max(packets,4),cap);
}
static inline larp_u32 larp_ack_window(larp_u32 cwnd, larp_u32 target,
    larp_u32 inflight, larp_u32 acked, larp_u32 ssthresh, enum larp_recovery recovery)
{
    if (recovery==LARP_TIMEOUT) return (larp_u32)larp_max(1,larp_min(cwnd+acked,target));
    if (recovery==LARP_RECOVERY) return (larp_u32)larp_min(target,larp_max(inflight+acked,2));
    if (recovery==LARP_RECOVERY_GROW) return (larp_u32)larp_min(target,larp_max(cwnd+acked,inflight+acked));
    if (recovery==LARP_CWR) return (larp_u32)larp_max(1,larp_min(larp_min(target,ssthresh),larp_max(cwnd,inflight+acked)));
    return (larp_u32)larp_max(1,larp_min(target,cwnd+acked));
}
#endif
