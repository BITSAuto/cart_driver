# Q-002 · The cart's real coast deceleration

- **Opened:** 2026-10-08 · **Status:** Open · **Priority:** high for real driving

`coast_decel` 0.4 m/s² sets the emergency-brake threshold. Too high a value
brakes too late; too low brakes unnecessarily. Measure: hold 5 km/h on a
flat road, release the throttle, time the roll-out over marked distances.
Update `config/*.yaml` (`cart_safety.coast_decel`) and tesla_sim's
`coastDecel`.
