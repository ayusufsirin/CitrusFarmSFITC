# SFITC

## Start ROS Core:

```bash
roscore
```

## Play SensorSuiteV2:

```bash
rosbag play -l -u 10 ~/Development/DS/SensorSuiteV2/test4\(pg-ileri-geri-sol-sag\).bag 
```

```bash
rosbag play -l -s 40 -u 10 ~/Development/DS/SensorSuiteV2/2025-04-15-15-30-12.bag
```

```bash
rosbag play -l -s 40 -u 10 ~/Development/DS/SensorSuiteV2/test10.bag
```

## Play CitrusFarm

Play VLP:

```bash
rosbag play -l -u 10 ~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/base_2023-07-18-14-26-48_0.bag 
```

Play ZED:

```bash
rosbag play -l -u 10 ~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/zed_2023-07-18-14-26-49_0.bag
```

Play odom:

```bash
rosbag play -l -u 10 ~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/odom_2023-07-18-14-26-48.bag
```

Play together:

```bash
rosbag play -l -u 10 \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/base_2023-07-18-14-26-48_0.bag \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/zed_2023-07-18-14-26-49_0.bag \
~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/odom_2023-07-18-14-26-48.bag
```

## Start RViz:

```bash
rviz -d ~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/rviz.rviz
```

## Run code:

```bash
python3 main.py
```