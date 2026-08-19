2026-08-18 23:05 에 실려 있던 obs_dim=58 세대(전방 5점, curv_clip 2.0).
2026-08-19 에 obs_dim=60 세대(20260818_85110/pow_healthy.pt)로 교체하면서 여기로 옮겼다.
되돌리려면 이 두 .pt 를 models/ 로 옮기고 config 의 curv_lookahead 에서 120 을 빼고
curv_clip 을 2.0 으로, offline_sim.py 의 CURV_OFF/CURV_CLIP 도 같이 되돌릴 것.
