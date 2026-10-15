# Real-cart test plan (not started)

Nothing here has run on the cart. Blockers first:
[real-cart-gaps.md](../04-knowledge/real-cart-gaps.md). Throughout: empty
roads with campus permission, a person at the physical kill switch, a
spotter walking alongside.

1. **Bench, wheels off the ground** (vehicle_bridge's
   `first-powered-test.md`): encoder zero and sign, ±15° at the wheels,
   steering rate, emergency-brake topic, neutral on lost commands.
2. **Calibration, driven manually:** throttle byte vs speed; coast-down from
   5 km/h (→ `coast_decel`); camera position and cart dimensions (→ `cart`
   profile).
3. **Shadow mode:** drive by hand with perception and both cart_driver nodes
   running but disconnected from the bridge (remap `/cmd_ackermann` and
   `/vehicle/emergency_brake`). Record a bag. Check the planner's choices and
   whether the safety node would have braked falsely on paint, paper or
   shadows. Replay the bag after every change.
4. **Step 2 live:** speed hold on a straight; unplug the camera mid-run → must
   coast, no brake; emergency brake on a cardboard box at walking pace.
5. **Steps 3–4 live:** avoid cardboard boxes; drive over paper and paint.
6. **Road following:** a campus road with clear edges, road-centre cost on,
   shadow mode first, then live. (No recorded routes: [D-021](../03-decisions/D-021-no-recorded-routes.md).)

Record each session in `10-worklog/` and results in
[09-testing-and-results](../09-testing-and-results/README.md).
