# Q-012 · road_follower isn't in git; cart_driver vendors its road model

- **Opened:** 2026-10-12 · **Status:** Open · **Priority:** medium

`road_follower` exists only in `~/BITSAuto_All_repo` on the Orin, which is not
a repository, along with uncommitted `tesla_sim` changes it relies on.
`cart_driver/road_model.py` is a copy of its estimator. Put road_follower in
a BITSAuto repository (and commit the tesla_sim changes on a branch), then
depend on it and delete the copy. Until then, changes to either copy must be
made in both.
