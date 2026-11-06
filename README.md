# TCP LARP

Linux TCP congestion control with a speculative paced launch and adaptive rate
and flight limits. Aggressive growth uses 5% queued / 25% clear-delay candidates
within the rate bound. Growth requires current RTT; the minimum RTT is
periodically revalidated. Initial defaults: 1 Gbps pacing and a 512-segment
flight limit; qualified feedback can raise both.

Build against Linux kernel headers (tested on 6.12):

```sh
make
```

![TCP LARP performance](assets/performance.svg)

Three runs per condition in a two-vCPU Linux VM. Points are means; whiskers are
observed ranges. All LARP results, continuous solo controls and competing pairs
were freshly measured (204 observations). Other methods' file-completion curves
are earlier references with unchanged endpoint code, kernel and package versions.
The separate 21-case pilot is excluded from the plotted means.

Solo delivery uses a 6-second window after 2 seconds of warmup; pairs use 10
seconds. File completion includes connection setup. The 25 Mbps path uses a
shallow RED/ECN queue; other paths use FIFO queues sized to one BDP. The first
three panels have logarithmic vertical axes. For LARP/self pairs, the fourth
panel shows the first sender's share; 50% means equal delivery.

At 1 Gbps/30 ms, LARP solo delivery averages 955.7 Mbps and LARP/CUBIC averages
762/192 Mbps. At 2 Gbps/5 ms, LARP/CUBIC averages 970/931 Mbps. Sharing remains
uneven against BBR and on the shallow-queue path, including LARP/self pairs.

JumpStart/CUBIC is an experimental port. QUIC runs over UDP using quic-go
v0.54.0 with its built-in CUBIC selected, TLS 1.3 and no 0-RTT; TCP is unencrypted.
These are transport/implementation comparisons on a virtual testbed.
Research prototype; GPL-2.0-only.
