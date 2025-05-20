Start ROS Core:

```bash
roscore
```

Play VLP:

```bash
rosbag play -l -u 10 ~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/base_2023-07-18-14-26-48_0.bag 
```

Play VLP:

```bash
rosbag play -l -u 10 ~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/zed_2023-07-18-14-26-49_0.bag
```

Start RViz:

```bash
rviz -d ~/Development/DS/Citrus-Farm-Dataset/01_13B_Jackal/rviz.rviz
```

Run code:

```bash
python3 main.py
```