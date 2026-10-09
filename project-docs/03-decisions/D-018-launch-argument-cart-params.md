# D-018 · The launch argument is `cart_params`, not `params`

- **Date:** 2026-10-08
- **Status:** Accepted (`5cec4ff`)

Launch arguments are global. Declaring `params` in `sim.launch.py` silently
fed this package's config to the included traversability launch, which reads
`params` for its own config, so perception lost its bonnet mask and the bonnet
was lethal in every grid. Renamed to `cart_params`.
