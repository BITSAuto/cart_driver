"""Map sensor timestamps onto the node clock.

In tesla_sim the drivers integrate on sim time (/clock, running slower than
real time) but Webots stamps sensor messages with wall-clock time. To place a
perception frame where the vehicle was when it was captured, its wall stamp
is converted to sim time using recent (wall, sim) clock pairs:

    sim(stamp) = sim_now - (wall_now - stamp) * rate

where ``rate`` is the observed sim-per-wall speed. On the real vehicle both
clocks are the same and ``identity=True`` skips all this.
"""

import collections


class ClockMap:
    def __init__(self, identity=False, window_s=3.0):
        self.identity = identity
        self.window_s = window_s
        self.pairs = collections.deque()

    def observe(self, wall, sim):
        self.pairs.append((wall, sim))
        while self.pairs and wall - self.pairs[0][0] > self.window_s:
            self.pairs.popleft()

    @property
    def rate(self):
        if len(self.pairs) < 2:
            return 1.0
        (w0, s0), (w1, s1) = self.pairs[0], self.pairs[-1]
        return (s1 - s0) / (w1 - w0) if w1 > w0 else 1.0

    def to_sim(self, stamp_wall):
        if self.identity or not self.pairs:
            return stamp_wall
        wall_now, sim_now = self.pairs[-1]
        return sim_now - (wall_now - stamp_wall) * self.rate
