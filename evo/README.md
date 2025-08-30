## Evaluations

- [PG and ZED comparison for first 200 seconds of `CitrusFarmDataset/01_13B_Jackal`](results/v1/README.md) (Evaluated at commit 016e22429ad7e410c510774746959dc5c1be57ea)
- [NO_PG, PG, and ZED comparison for first 200 seconds of `CitrusFarmDataset/01_13B_Jackal`](results/v2_no_pg_all_metrics/README.md) (Evaluated at commit 016e22429ad7e410c510774746959dc5c1be57ea)

## Run Docker

```bash
docker build -t evo-cli .
cd ..
docker run --rm -it -v "$PWD:/work" -w /work evo-cli
```

## Topic to TUM

```bash
evo_traj bag rosbag/rtabmap_no_pg_odom_2025-08-29-22-27-06.bag  /pg/rtabmap/odom  --save_as_tum
mv pg_rtabmap_odom.tum no_pg_rtabmap_odom.tum
evo_traj bag rosbag/rtabmap_zed_odom_2025-08-20-17-15-03.bag /zed/rtabmap/odom --save_as_tum
evo_traj bag rosbag/rtabmap_pg_odom_2025-08-20-16-56-56.bag  /pg/rtabmap/odom  --save_as_tum
evo_traj bag rosbag/gt_200sec.bag  /gps/fix/odometry  --save_as_tum
```

## Run metrics

```bash
evo_traj tum zed_rtabmap_odom.tum --full_check
evo_traj tum pg_rtabmap_odom.tum --full_check
evo_traj tum no_pg_rtabmap_odom.tum --full_check
```

RPE (time-based, 1s):

```bash
evo_rpe tum gps_fix_odometry.tum zed_rtabmap_odom.tum   -a -r trans_part -d 20 -u f --save_results zed_rpe_~1s.zip
evo_rpe tum gps_fix_odometry.tum pg_rtabmap_odom.tum   -a -r trans_part -d 20 -u f --save_results pg_rpe_~1s.zip
evo_rpe tum gps_fix_odometry.tum no_pg_rtabmap_odom.tum   -a -r trans_part -d 20 -u f --save_results no_pg_rpe_~1s.zip
```

ATE vs. RTK/GT (Sim(3) alignment shown; drop -s for SE(3)):

```bash
evo_ape tum gps_fix_odometry.tum zed_rtabmap_odom.tum -a -s -r trans_part --save_results zed_ape.zip
evo_ape tum gps_fix_odometry.tum pg_rtabmap_odom.tum -a -s -r trans_part --save_results pg_ape.zip
evo_ape tum gps_fix_odometry.tum no_pg_rtabmap_odom.tum -a -s -r trans_part --save_results no_pg_ape.zip
```

[//]: # (```bash)

[//]: # (evo_rpe tum gps_fix_odometry.tum zed_rtabmap_odom.tum -a -r trans_part --delta 1.0 --delta_unit s --save_results zed_rpe_1s.zip)

[//]: # (evo_rpe tum gps_fix_odometry.tum pg_rtabmap_odom.tum -a -r trans_part --delta 1.0 --delta_unit s --save_results pg_rpe_1s.zip)

[//]: # (evo_rpe tum gps_fix_odometry.tum no_pg_rtabmap_odom.tum -a -r trans_part --delta 1.0 --delta_unit s --save_results no_pg_rpe_1s.zip)

[//]: # (```)

Yaw error (use rotation angle):

```bash
evo_rpe tum gps_fix_odometry.tum zed_rtabmap_odom.tum -a -r angle_deg --delta 50 --delta_unit m --save_results zed_yaw_50m.zip
evo_rpe tum gps_fix_odometry.tum pg_rtabmap_odom.tum -a -r angle_deg --delta 50 --delta_unit m --save_results pg_yaw_50m.zip
evo_rpe tum gps_fix_odometry.tum no_pg_rtabmap_odom.tum -a -r angle_deg --delta 50 --delta_unit m --save_results no_pg_yaw_50m.zip
```

### KITTI-style

```bash
evo_rpe tum gps_fix_odometry.tum zed_rtabmap_odom.tum \
  -a -r trans_part -d 1 -u m --pairs_from_reference --t_max_diff 0.05 \
  --save_results zed_rpe_1m.zip
evo_rpe tum gps_fix_odometry.tum pg_rtabmap_odom.tum \
  -a -r trans_part -d 1 -u m --pairs_from_reference --t_max_diff 0.05 \
  --save_results pg_rpe_1m.zip
evo_rpe tum gps_fix_odometry.tum no_pg_rtabmap_odom.tum \
  -a -r trans_part -d 1 -u m --pairs_from_reference --t_max_diff 0.05 \
  --save_results no_pg_rpe_1m.zip
```

### Plot

Plot ZED/PG vs GT:

[//]: # (```bash)

[//]: # (evo_traj tum gps_fix_odometry.tum zed_rtabmap_odom.tum --align --plot --plot_mode xy)

[//]: # (evo_traj tum gps_fix_odometry.tum pg_rtabmap_odom.tum --align --plot --plot_mode xy)

[//]: # (evo_traj tum gps_fix_odometry.tum no_pg_rtabmap_odom.tum --align --plot --plot_mode xy)

[//]: # (```)

Both vs GT:

```bash
evo_traj tum zed_rtabmap_odom.tum pg_rtabmap_odom.tum no_pg_rtabmap_odom.tum \
  --ref gps_fix_odometry.tum -a --plot --plot_mode xy \
  --save_plot both_vs_gt_xy.png
```

### Table

[//]: # (```bash)

[//]: # (evo_ape tum gps_fix_odometry.tum zed_rtabmap_odom.tum -a -s \)

[//]: # (  --save_results zed_ape.zip)

[//]: # (evo_ape tum gps_fix_odometry.tum pg_rtabmap_odom.tum  -a -s \)

[//]: # (  --save_results pg_ape.zip)

[//]: # (evo_ape tum gps_fix_odometry.tum no_pg_rtabmap_odom.tum  -a -s \)

[//]: # (  --save_results no_pg_ape.zip)

[//]: # (```)

Save as CSV:

[//]: # (```bash)

[//]: # (evo_res zed_ape.zip pg_ape.zip zed_rpe_1m.zip \)

[//]: # (  --use_filenames \)

[//]: # (  --save_table results.csv)

[//]: # (```)

Save as PNG:

[//]: # (```bash)

[//]: # (evo_res zed_ape.zip pg_ape.zip zed_rpe_1m.zip  --use_filenames -p --save_plot results.png)

[//]: # (```)

## All Metric Comparison

```bash
evo_traj bag rosbag/gt_200sec.bag /gps/fix/odometry --save_as_tum
evo_traj bag rosbag/rtabmap_no_pg_odom_2025-08-29-22-27-06.bag  /pg/rtabmap/odom  --save_as_tum
mv pg_rtabmap_odom.tum no_pg_rtabmap_odom.tum
evo_traj bag rosbag/rtabmap_zed_odom_2025-08-20-17-15-03.bag /zed/rtabmap/odom --save_as_tum
evo_traj bag rosbag/rtabmap_pg_odom_2025-08-20-16-56-56.bag /pg/rtabmap/odom --save_as_tum
evo_config set plot_backend Agg
GT=gps_fix_odometry.tum
ZED=zed_rtabmap_odom.tum
PG=pg_rtabmap_odom.tum
NO_PG=no_pg_rtabmap_odom.tum
TMAX=0.05
evo_ape tum $GT $ZED -a -s --t_max_diff $TMAX --save_results zed_ape_sim3.zip
evo_ape tum $GT $PG  -a -s --t_max_diff $TMAX --save_results pg_ape_sim3.zip
evo_ape tum $GT $NO_PG  -a -s --t_max_diff $TMAX --save_results no_pg_ape_sim3.zip
evo_ape tum $GT $ZED -a     --t_max_diff $TMAX --save_results zed_ape_se3.zip
evo_ape tum $GT $PG  -a     --t_max_diff $TMAX --save_results pg_ape_se3.zip
evo_ape tum $GT $NO_PG  -a     --t_max_diff $TMAX --save_results no_pg_ape_se3.zip
evo_rpe tum $GT $ZED -a -r trans_part -d 1  -u m --pairs_from_reference --t_max_diff $TMAX --save_results zed_rpe_1m.zip
evo_rpe tum $GT $PG  -a -r trans_part -d 1  -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_rpe_1m.zip
evo_rpe tum $GT $NO_PG  -a -r trans_part -d 1  -u m --pairs_from_reference --t_max_diff $TMAX --save_results no_pg_rpe_1m.zip
evo_rpe tum $GT $ZED -a -r trans_part -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results zed_rpe_50m.zip
evo_rpe tum $GT $PG  -a -r trans_part -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_rpe_50m.zip
evo_rpe tum $GT $NO_PG  -a -r trans_part -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results no_pg_rpe_50m.zip
evo_rpe tum $GT $ZED -a -r angle_deg -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results zed_yaw_50m.zip
evo_rpe tum $GT $PG  -a -r angle_deg -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_yaw_50m.zip
evo_rpe tum $GT $NO_PG  -a -r angle_deg -d 50 -u m --pairs_from_reference --t_max_diff $TMAX --save_results no_pg_yaw_50m.zip
for L in 100 200 300 400 500 600 700 800; do
  evo_rpe tum $GT $ZED -a -r trans_part -d $L -u m --pairs_from_reference --t_max_diff $TMAX --save_results zed_rpe_${L}m.zip
  evo_rpe tum $GT $PG  -a -r trans_part -d $L -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_rpe_${L}m.zip
  evo_rpe tum $GT $NO_PG  -a -r trans_part -d $L -u m --pairs_from_reference --t_max_diff $TMAX --save_results pg_rpe_${L}m.zip
done
# Find the median dt first:
evo_traj tum $ZED --full_check   # note the median dt => N ≈ 1/dt
# Example for ~20 Hz:
evo_rpe tum $GT $ZED -a -r trans_part -d 20 -u f --t_max_diff $TMAX --save_results zed_rpe_~1s.zip
evo_rpe tum $GT $PG  -a -r trans_part -d 20 -u f --t_max_diff $TMAX --save_results pg_rpe_~1s.zip
evo_rpe tum $GT $NO_PG  -a -r trans_part -d 20 -u f --t_max_diff $TMAX --save_results no_pg_rpe_~1s.zip
evo_res zed_*.zip pg_*.zip no_pg_*.zip --use_filenames --save_table metrics_all.csv
# Plot XY
evo_traj tum $ZED --ref $GT -a --plot --plot_mode xy --save_plot zed_vs_gt_xy.png
evo_traj tum $PG  --ref $GT -a --plot --plot_mode xy --save_plot pg_vs_gt_xy.png
evo_traj tum $NO_PG  --ref $GT -a --plot --plot_mode xy --save_plot no_pg_vs_gt_xy.png
```

```bash
evo_traj bag ../../../rosbag/gt_200sec.bag /gps/fix/odometry --save_as_tum
evo_traj bag ../../../rosbag/rtabmap_zed_odom90_2025-08-24-14-44-26.bag /zed/rtabmap/odom90 --save_as_tum
evo_traj bag ../../../rosbag/rtabmap_pg_odom90_2025-08-24-14-12-06.bag /pg/rtabmap/odom90 --save_as_tum
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
