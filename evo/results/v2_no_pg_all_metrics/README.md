```bash
cd ../../
docker build -t evo-cli .
cd ../
docker run --rm -it -v "$PWD:/work" -w /work evo-cli
```

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

# Evaluation

**Global accuracy (ATE).** After alignment to RTK, **PG** yields the best global accuracy: **SE(3) ATE RMSE 0.912 m** vs
**1.105 m** (**ZED**, +17.5%) and **1.537 m** (**NO\_PG**, +40.6%). Results are identical under **Sim(3)** (PG 0.912 m
vs ZED 1.105 m vs NO\_PG 1.536 m), indicating scale is not the driver here. Medians tell a complementary story: **ZED**
attains the lowest central error (SE3 median **0.831 m**) vs **PG 0.884 m** and **NO\_PG 0.990 m**; thus **PG** reduces
outliers (lower RMSE/mean), while **ZED** is slightly tighter around the mode. **NO\_PG** exhibits markedly heavier
tails (max ≈40 m), consistent with sporadic failure modes.

**Local drift—distance based (RPE).** At **1 m**, **ZED** and **PG** are essentially tied (**RMSE 1.464 vs 1.452 m**;
medians **1.198 vs 1.216 m**), whereas **NO\_PG** shows comparable mean/median (**1.507/1.298 m**) but a **\~2.9× higher
RMSE (4.206 m)** due to heavy-tailed segments—i.e., occasional short-range slips. At **50 m**, **NO\_PG** surprisingly
achieves the **lowest RMSE 21.862 m**, outperforming **PG 28.776 m** (-24.0%) and **ZED 29.893 m** (-26.8%); however, *
*ZED** has the **lowest median** at this scale (**12.953 m** vs **13.854 m** for **NO\_PG** and **15.274 m** for **PG
**), indicating smoother “typical” behavior but fatter tails that inflate its RMSE. At **100 m**, only **ZED** and **PG
** have valid pairs; **PG** is clearly better (**2.537 m** vs **3.972 m**, −36%), consistent with its stronger global
robustness on this sequence.

**Short-horizon drift (\~1 s, frames).** Over \~1 s windows, **ZED** is best (**RMSE 3.251 m**), **PG** is close (*
*3.348 m**, +2.9%), and **NO\_PG** is worst (**6.454 m**, \~+99% vs ZED). This indicates the middle state is not yet
stable at very short horizons.

**Orientation over 50 m (full angle).** Using the full 3D rotation-angle RPE, **ZED** and **PG** are similar (**126.9°**
vs **128.1°** RMSE), while **NO\_PG** is notably worse (**176.6°**, \~+39% vs ZED). (As discussed earlier, a yaw-only,
XY-projected variant is more interpretable for heading; the relative gap typically narrows there.)

**Interpretation.** For this run, **PG** is the most **robust globally** (lowest ATE RMSE; better 100 m drift), **ZED**
is **steadier at short horizons** and often has the **best medians** (i.e., smoother typical segments but with
occasional spikes), and **NO\_PG**—while unstable at very short horizons and in orientation—shows an **interesting sweet
spot at 50 m** with the **lowest drift RMSE** among the three. Practically: favor **PG** when end-to-end accuracy and
long-segment consistency matter (mapping, loop-closure stability); favor **ZED** when short-latency tracking smoothness
is critical (control); and treat **NO\_PG** as promising for moderate-range segments but in need of **outlier
suppression** to rein in its short-horizon and angular failures.
