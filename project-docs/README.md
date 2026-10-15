# cart_driver · Project docs

The shared engineering record for **cart_driver**, the BITSAuto cart's
driving stack: a 5 km/h throttle-only speed hold, an independent safety
monitor that is the only thing allowed to brake, a short-term memory map, a
relay-aware planner and a road-centre cost that keeps it mid-road. The usage summary is the
top-level [README](../README.md).

## Start here

| If you are... | Read |
| --- | --- |
| new to the project | [01-overview](01-overview/README.md), then [05-status](05-status/README.md) |
| changing driving code | [02-architecture](02-architecture/README.md), the [decisions](03-decisions/README.md) and [bugs-and-lessons](07-bugs-and-lessons/README.md) |
| testing in the sim | [08-guides](08-guides/README.md) and [09-testing-and-results](09-testing-and-results/README.md) |
| preparing the real cart | [real-cart-gaps.md](04-knowledge/real-cart-gaps.md) and [real-cart-test-plan.md](08-guides/real-cart-test-plan.md) |
| an AI agent | [AGENTS.md](../AGENTS.md), then this page |

## Status at a glance (2026-10-09)

Steps 2–4 merged; the road-centre cost in review (PR #6); step 5 (recorded
loop route) dropped on 2026-10-15 (D-021). In the sim: road following on a
7 m road with curbs at 5 km/h (0.14 m from centre, 0 brakes), avoidance with
0.49 m clearance on wide roads, flat clutter driven over, emergency stops
when avoidance is impossible. Gaps: junctions, keeping left, obstacles on
narrow roads. Real cart: not started; it needs a speed source, its
dimensions and coast rate.

## Contents

1. **[Overview](01-overview/README.md)** —
   [project brief](01-overview/project-brief.md) ·
   [system map](01-overview/system-map.md) ·
   [hardware and environment](01-overview/hardware-and-environment.md) ·
   [glossary](01-overview/glossary.md)
2. **[Architecture](02-architecture/README.md)** —
   [components](02-architecture/components.md) ·
   [interfaces](02-architecture/interfaces.md) ·
   [frames and conventions](02-architecture/frames-and-conventions.md) ·
   [speed and safety](02-architecture/speed-and-safety.md) ·
   [odometry and memory map](02-architecture/odometry-and-memory-map.md) ·
   [planner](02-architecture/planner.md) ·
   [route following](02-architecture/route-following.md) ·
   [vehicle profiles](02-architecture/vehicle-profiles.md)
3. **[Decisions](03-decisions/README.md)** — D-001 to D-019
4. **[Knowledge](04-knowledge/README.md)** —
   [driving numbers](04-knowledge/driving-numbers.md) ·
   [perception as the driver sees it](04-knowledge/perception-as-the-driver-sees-it.md) ·
   [sim facts](04-knowledge/sim-facts-that-matter-here.md) ·
   [real-cart gaps](04-knowledge/real-cart-gaps.md)
5. **[Status](05-status/README.md)** —
   [completed](05-status/completed.md) ·
   [in progress](05-status/in-progress.md) ·
   [roadmap](05-status/roadmap.md)
6. **[Open questions](06-open-questions/README.md)** — Q-001 to Q-011
7. **[Bugs and lessons](07-bugs-and-lessons/README.md)** —
   [driving-stack bugs](07-bugs-and-lessons/driving-stack-bugs.md) ·
   [sim and process pitfalls](07-bugs-and-lessons/sim-and-process-pitfalls.md)
8. **[Guides](08-guides/README.md)** —
   [setup](08-guides/setup-and-build.md) ·
   [sim](08-guides/running-in-the-sim.md) ·
   [scenarios](08-guides/running-scenarios.md) ·
   [routes](08-guides/making-routes.md) ·
   [RViz](08-guides/viewing-in-rviz.md) ·
   [real-cart test plan](08-guides/real-cart-test-plan.md)
9. **[Testing and results](09-testing-and-results/README.md)** —
   [unit tests](09-testing-and-results/unit-tests.md) ·
   [step 2](09-testing-and-results/step-2-speed-and-safety.md) ·
   [step 3](09-testing-and-results/step-3-memory-map.md) ·
   [step 4](09-testing-and-results/step-4-planner.md) ·
   [step 5](09-testing-and-results/step-5-loop.md)
10. **[Worklog](10-worklog/README.md)** — one page per work session

## How to keep this book current

This book is only useful if it is never stale. **Every piece of work updates
it in the same branch or PR** as the work itself: code, experiments,
debugging, field tests, measurements, and decisions made in a meeting or a
chat. A PR without a docs update is incomplete (the PR template has the
checklist).

| When you... | Update |
| --- | --- |
| do any work at all | add a page to `10-worklog/` (copy the template in its README) and adjust `05-status/` |
| decide something (including "we won't do X") | add `03-decisions/D-NNN-slug.md` and a row in its index. To reverse a decision, add a new one that supersedes it and mark the old one *Superseded by D-NNN*; don't delete it |
| verify a fact (measurement, datasheet, reading source code, a live test) | add it to the right page in `04-knowledge/` with a confidence tag and *how* it was checked |
| hit something unknown that matters | add `06-open-questions/Q-NNN-slug.md` and a row in its index |
| answer an open question | set its status to *Resolved*, say what resolved it and link to where the answer now lives; keep the page |
| find a bug (yours or anyone's) | record it in `07-bugs-and-lessons/`: symptom, root cause, fix, and the rule that would have prevented it |
| change how to build, run, deploy or calibrate | update `08-guides/` |
| get a new test or scenario result | update `09-testing-and-results/` |
| change architecture, topics, parameters or conventions | update `02-architecture/` |

**Writing rules**
- **Append, don't rewrite history.** If something written here turns out to be
  wrong, add a dated *Correction (YYYY-MM-DD):* under it. Knowing that
  something was believed and then disproved is useful.
- **Say how you know.** Confidence tags: **Verified** (checked directly
  against source code, a live system or a measurement; say which),
  **Documented** (from a datasheet or other docs, not re-checked here),
  **Assumed** (a guess, placeholder or inference; must also appear in open
  questions if it matters). Never let repetition upgrade an assumption.
- **Date everything** (YYYY-MM-DD) and link PRs, commits and issues.
- **One topic per page.** Add pages rather than growing one page forever.
  Each chapter's `README.md` lists its pages; keep that list in sync.
- **Link, don't duplicate.** Facts about another repo's component belong in
  that repo's book; link to it.
- **No secrets or personal details.** No passwords, tokens, private IPs,
  personal machine configs or e-mail addresses. Three of our repos are
  public.
- **Write for someone who wasn't there.** Spell out the why, not just the
  what.
