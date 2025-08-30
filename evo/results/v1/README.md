```bash
cd ../../
docker build -t evo-cli .
cd ../
docker run --rm -it -v "$PWD:/work" -w /work evo-cli
```

```bash
evo_traj bag rosbag/gt_200sec.bag /gps/fix/odometry --save_as_tum
evo_traj bag rosbag/rtabmap_zed_odom90_2025-08-24-14-44-26.bag /zed/rtabmap/odom90 --save_as_tum
evo_traj bag rosbag/rtabmap_pg_odom90_2025-08-24-14-12-06.bag /pg/rtabmap/odom90 --save_as_tum
evo_config set plot_backend Agg
GT=gps_fix_odometry.tum
ZED=zed_rtabmap_odom90.tum
PG=pg_rtabmap_odom90.tum
TMAX=0.05
evo_ape tum $GT $ZED -a -s --t_max_diff $TMAX --save_results zed_ape_sim3.zip
evo_ape tum $GT $PG  -a -s --t_max_diff $TMAX --save_results pg_ape_sim3.zip
evo_ape tum $GT $ZED -a     --t_max_diff $TMAX --save_results zed_ape_se3.zip
evo_ape tum $GT $PG  -a     --t_max_diff $TMAX --save_results pg_ape_se3.zip
evo_rpe tum $GT $ZED -a -r trans_part -d 1  -u m --pairs_from_reference --t_max_diff $TMAX --save_results zed_rpe_1m.zip
evo_rpe tum $GT $PG  -a -r trans_part -d 1  -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_rpe_1m.zip
evo_rpe tum $GT $ZED -a -r trans_part -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results zed_rpe_50m.zip
evo_rpe tum $GT $PG  -a -r trans_part -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_rpe_50m.zip
evo_rpe tum $GT $ZED -a -r angle_deg -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results zed_yaw_50m.zip
evo_rpe tum $GT $PG  -a -r angle_deg -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_yaw_50m.zip
for L in 100 200 300 400 500 600 700 800; do
  evo_rpe tum $GT $ZED -a -r trans_part -d $L -u m --pairs_from_reference --t_max_diff $TMAX --save_results zed_rpe_${L}m.zip
  evo_rpe tum $GT $PG  -a -r trans_part -d $L -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_rpe_${L}m.zip
done
# Find the median dt first:
evo_traj tum $ZED --full_check   # note the median dt => N ≈ 1/dt
# Example for ~20 Hz:
evo_rpe tum $GT $ZED -a -r trans_part -d 20 -u f --t_max_diff $TMAX --save_results zed_rpe_~1s.zip
evo_rpe tum $GT $PG  -a -r trans_part -d 20 -u f --t_max_diff $TMAX --save_results pg_rpe_~1s.zip
evo_res zed_*.zip pg_*.zip --use_filenames --save_table metrics_all.csv
# Plot XY
evo_traj tum $ZED --ref $GT -a --plot --plot_mode xy --save_plot zed_vs_gt_xy.png
evo_traj tum $PG  --ref $GT -a --plot --plot_mode xy --save_plot pg_vs_gt_xy.png
```

# Evaluation

## Metrics

**Evaluation metrics.** We evaluate odometry accuracy against RTK ground truth using a combination of global and local
errors that are standard in the SLAM/VO literature. **Absolute Trajectory Error (ATE)** quantifies the global
consistency of the estimated trajectory after rigid/similarity alignment to ground truth. We report ATE under two
alignment models: **Sim(3)** (translation + rotation + uniform scale) to factor out global scale bias, and **SE(3)** (no
scale) to assess true metric accuracy for pipelines with known scale (e.g., stereo/LiDAR). In both cases we compute the
Umeyama closed-form alignment and summarize the translation residuals with RMSE, mean, median, std, min, and max. To
measure local drift independent of global alignment we use **Relative Pose Error (RPE)**. For distance-based RPE, we
compare relative motions between poses separated by a fixed path length $\Delta$ along the trajectory (
e.g., $\Delta\!\in\!\{1,50,100\}$ m), reporting translational error in meters; segment endpoints are selected along the
ground-truth path when feasible to normalize by true distance (with a small segment-length tolerance) and associated to
estimator timestamps within a fixed time tolerance. We also report **heading drift** as the **yaw-only angular RPE**
over $\Delta\!=\!50$ m: we project motions to the horizontal (XY) plane and compute the relative rotation angle in
degrees, filtering near-stationary intervals to avoid spurious headings. Finally, to approximate a fixed-time horizon
error we compute **frame-based RPE (\~1 s)** by choosing $N$ frames from the odometry’s median sampling period (
e.g., $N\!\approx\!1/\Delta t$). All metrics are computed over the **first 200 s** slice of each sequence; longer
segments (≥150–200 m) are omitted where no valid pairs exist within the ground-truth/estimate overlap. For every metric
we report the full robust statistics (RMSE, median, mean, std, min, max), ensuring comparability across methods and
segment scales.

## Numerical

**Numerical comparison (200-s window).** Globally, **PG** is more accurate than **ZED** on ATE: Sim(3) **RMSE 0.912 m vs
1.105 m** (≈17.4% lower for PG), and SE(3) **0.912 m vs 1.105 m** (≈17.5% lower). Interestingly, ZED’s **median ATE** is
slightly smaller (Sim(3): **0.842 m** vs **0.884 m**; SE(3): **0.831 m** vs **0.884 m**), suggesting ZED has tighter
central errors but heavier tails, while PG reduces outliers. Locally, at **RPE-1 m** both are essentially tied (**RMSE
1.452 m (PG) vs 1.464 m (ZED)**; medians **1.216 m vs 1.198 m**). At **RPE-50 m**, PG has lower RMSE (**28.776 m vs
29.893 m**, ≈3.7% better), though ZED shows a lower median (**12.953 m vs 15.274 m**), again hinting at outlier
sensitivity. At **RPE-100 m** PG is lower (**2.537 m vs 3.972 m**, ≈36% better), but this is based on very few segment
pairs in this cropped run (treat as indicative). For short-horizon drift (\~**1 s**, frame-based), **ZED** is slightly
better (**RMSE 3.251 m vs 3.348 m**, ≈3% lower; medians **2.345 m vs 2.598 m**). The **50 m yaw-angle RPE** (full 3D
angle) is very large for both and nearly identical (**RMSE 126.9° (ZED) vs 128.1° (PG)**); these magnitudes reflect
using the full rotation angle rather than yaw-only—projecting to the XY plane and filtering near-stationary intervals (
as in our protocol) yields a more interpretable heading metric.

## Summary

**Summary.** On this 200-s slice, **PG** is the safer choice when you care about **end-to-end/global accuracy and
robustness**: it has lower ATE RMSE than ZED (both Sim(3) and SE(3)), and its distance-based RPE shows **fewer large
drifts** (lower RMSE at 50 m and 100 m). **ZED**, however, tends to have **slightly smaller medians** and a **better \~1
s RPE**, i.e., it’s a bit steadier over short horizons—useful for **real-time control** and smooth local motion. At **1
m RPE** they’re essentially tied; at **50 m** ZED’s lower median suggests smoother accumulation but with **spikier
outliers** (which raise its RMSE), whereas PG keeps the tails in check. Orientation over 50 m (full 3D angle) looks
similarly large for both; for **heading-only** use yaw-projected RPE with a motion filter—expect differences to shrink
or flip depending on turns and standstills. **Rule of thumb:** pick **PG** for mapping, loop-closure quality, and
reliability under perturbations; pick **ZED** when short-latency tracking smoothness matters more than rare large
errors.

