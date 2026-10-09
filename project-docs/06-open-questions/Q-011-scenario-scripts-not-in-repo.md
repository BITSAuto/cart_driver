# Q-011 · Scenario scripts aren't in the repo

- **Opened:** 2026-10-09 · **Status:** Open · **Priority:** medium

Scenarios A–F and L were run with shell scripts in a session's scratch
directory (start the launch with SIGINT restored, spawn objects, run
`simtools monitor`, stop). Only `tools/simtools.py` is committed. Add a
`tools/scenarios/` runner so anyone can reproduce
[09-testing-and-results](../09-testing-and-results/README.md).
