# Q-009 · Performance on the Orin

- **Opened:** 2026-10-09 · **Status:** Open · **Priority:** medium

The planner rolls out 21 candidates × ~67 steps in Python/numpy at 10 Hz,
and the memory map integrates every grid. Fine on the laptop; unmeasured on
the Orin alongside TensorRT perception.
