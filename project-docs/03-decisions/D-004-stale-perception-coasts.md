# D-004 · Stale perception means coast, not brake

- **Date:** 2026-10-08
- **Status:** Accepted

If no perception grid has arrived for 0.5 s, the safety node says `coast`.
By the time it matters the cart has rolled to a stop on its own, without the
damaging brake. The driver also coasts if `/cart/safety` itself goes quiet
for 0.5 s. (An earlier plan option, braking after ~2 s of staleness while
moving, was dropped in favour of coasting only.)
