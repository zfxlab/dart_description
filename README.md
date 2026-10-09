# dart_description

`dart_description` 是飞镖视觉系统的 ROS 2 描述包，集中维护场地、基地、飞镖检测模块、发射机构和双目相机的 URDF/xacro 模型及 TF 关系，并提供 RViz 调试工具和基于 URDF 的目标几何计算工具。

当前面向 ROS 2 Jazzy。

## 功能

- 通过 `robot_state_publisher` 发布整套系统的 `/robot_description`、`/tf` 和 `/tf_static`。
- 支持红、蓝方各自的基地现场位姿偏移，长度使用米，角度使用弧度。
- 描述飞镖检测模块滑轨，中心为零位，运动范围为 `[-0.28, 0.28] m`。
- 描述发射机构 yaw 关节、双目相机安装位姿及 optical frame。
- 在 RViz 中显示机器人模型、TF、相机视场、基地 ROI 和场地网格。
- 从 URDF 自动推导检测模块、绿灯和装甲板相对镖架的几何函数，并导出 CSV。

## 目录结构

```text
dart_description/
├── config/
│   ├── environment.yaml       # 基地 ROI 和场地网格
│   ├── sensor_fov.yaml        # 双目相机视场参数
│   └── site/default.yaml      # 红蓝方基地现场偏移
├── launch/
│   ├── description.launch.py  # 发布 robot_description 和 TF
│   └── view_description.launch.py  # 完整 RViz 调试环境
├── meshes/                    # STL 模型
├── rviz/description.rviz      # RViz 配置
├── scripts/
│   ├── environment_visualizer.py
│   ├── sensor_fov_visualizer.py
│   └── target_geometry.py     # URDF 几何计算与 CSV 导出
└── urdf/dart_system.urdf.xacro
```

## 坐标系与关节

主要 TF 树如下：

```text
field_link
├── base_nominal_link
│   └── base_link
│       └── rail_origin_link
│           └── dart_detection_module_link
│               ├── green_light_link
│               └── armor_link
└── dart_pedestal_link
    └── dart_base_link
        └── launcher_frame
            └── stereo_camera_center_link
                ├── left_camera_mount_link
                │   └── left_camera_optical_frame
                └── right_camera_mount_link
                    └── right_camera_optical_frame
```

坐标约定：

- `field_link`：场地左下角为原点，`x` 向右、`y` 向前、`z` 向上。
- 相机 optical frame：遵循 ROS 光学坐标约定，`z` 向前、`x` 向右、`y` 向下。
- 除 `field_link` 和相机 optical frame 外，其余机械坐标系统一采用右手坐标系：`x` 向前、`y` 向左、`z` 向上。
- 统一的轴定义不表示这些坐标系在空间中相互平行；固定安装姿态、目标倾角以及可动关节的旋转均由 URDF 中对应 joint 的 `origin rpy` 和关节位置描述。
- 所有长度单位为米，URDF 中所有角度单位为弧度。

可动关节：

| 关节 | 类型 | 轴 | 范围 |
| --- | --- | --- | --- |
| `dart_detection_module_slide_joint` | prismatic | `+x` | `[-0.28, 0.28] m` |
| `launcher_yaw_joint` | revolute | `+z` | `[-π/9, 0] rad` |

检测模块滑轨的零位位于行程中心，`-0.28 m` 和 `0.28 m` 分别对应两个机械端点。

## 获取源码

仓库使用 Git LFS 管理 STL、PCD 等大文件。首次使用前安装并初始化 Git LFS：

```bash
sudo apt update
sudo apt install git-lfs
git lfs install
```

克隆 `stereo` 分支并拉取 LFS 文件：

```bash
git clone --branch stereo https://github.com/zfxlab/dart_description.git
cd dart_description
git lfs pull
```

更新已有仓库及其 LFS 文件：

```bash
git pull --ff-only
git lfs pull
```

## 依赖与构建

安装 ROS 2 Jazzy 后，在 `dart_description` 仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select dart_description --symlink-install
source install/setup.bash
```

主要运行依赖包括 `robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`、`xacro`、`rclpy`、NumPy 和 PyYAML，完整列表见 `package.xml`。

## 启动模型与 TF

只启动 xacro、`robot_state_publisher` 和 TF 发布：

```bash
ros2 launch dart_description description.launch.py
```

可用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `site_file` | `config/site/default.yaml` | 红蓝方及现场基地偏移配置 |
| `use_sim_time` | `false` | 是否使用仿真时间 |

指定现场配置：

```bash
ros2 launch dart_description description.launch.py \
  site_file:=/absolute/path/to/site.yaml
```

## 启动完整 RViz 调试环境

```bash
ros2 launch dart_description view_description.launch.py
```

该启动文件包含：

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `sensor_fov_visualizer.py`
- `environment_visualizer.py`
- RViz 2

可用参数：

| 参数 | 默认配置 | 说明 |
| --- | --- | --- |
| `site_file` | `config/site/default.yaml` | 红蓝方及现场基地偏移 |
| `sensor_fov_config` | `config/sensor_fov.yaml` | 左右相机 FOV Marker 参数 |
| `environment_config` | `config/environment.yaml` | 基地 ROI 和场地网格参数 |

## 现场基地偏移

`config/site/default.yaml` 同时保存红、蓝方标定值：

```yaml
is_red: true

red:
  xyz_m: [0.0, 0.0, 0.0]
  rpy_rad: [0.0, 0.0, 0.0]

blue:
  xyz_m: [0.0, 0.0, 0.0]
  rpy_rad: [0.0, 0.0, 0.0]
```

- `is_red: true` 使用 `red`，`false` 使用 `blue`。
- `xyz_m` 表示 `base_link` 在 `base_nominal_link` 中的位置偏移。
- `rpy_rad` 按 roll、pitch、yaw 顺序表示姿态偏移，单位为弧度。
- 配置在启动时读取，修改后需要重启 launch。
- 文件不存在、字段缺失、类型错误或包含非有限数值时，启动会直接报告错误。

## 可视化配置

### 双目相机视场

`config/sensor_fov.yaml` 为左右相机分别配置：

- optical frame 名称
- 图像宽高
- `fx`、`fy`、`cx`、`cy`
- 近、远截面距离
- Marker 颜色、透明度和线宽

该工具采用理想针孔模型，不模拟镜头畸变、遮挡或图像校正后的裁剪。Marker 发布到：

```text
/sensor_fov_markers
```

### 基地 ROI 与场地网格

`config/environment.yaml` 配置基地轴对齐包围盒以及场地网格的范围、高度、间距和线宽。Marker 发布到：

```text
/environment_markers
```

两个 MarkerArray 话题均采用 transient-local durability，新启动的 RViz 订阅者也能收到最近一次数据。

## URDF 目标几何计算工具

`target_geometry.py` 从 xacro 读取关节树、滑轨方向与限位，计算检测模块、绿灯和装甲板中心在 `dart_base_link` XY 平面内的位置、距离和 yaw，并对完整滑轨行程采样生成 CSV。

构建后运行：

```bash
ros2 run dart_description target_geometry.py
```

也可以直接从源码运行：

```bash
python3 scripts/target_geometry.py
```

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--urdf` | 包内 `dart_system.urdf.xacro` | 指定其他 URDF/xacro 文件 |
| `--samples` | `561` | 包含两个端点的全行程采样数 |
| `--output` | `target_geometry.csv` | CSV 输出路径 |

当前行程为 `0.56 m`，默认 561 个点对应 `1 mm` 间隔。CSV 字段为：

```text
slide_m
module_x_m, module_y_m
green_x_m, green_y_m, green_distance_m, green_yaw_rad
armor_x_m, armor_y_m, armor_distance_m, armor_yaw_rad
yaw_delta_rad, distance_delta_m
```

计算约定：

- 位置和距离单位为米，yaw 单位为弧度。
- `yaw = atan2(y, x)`，正值向左，符合 x 前、y 左、z 上坐标系的右手定则。
- `yaw_delta_rad = armor_yaw_rad - green_yaw_rad`，归一化到 `[-π, π)`。
- `distance_delta_m = armor_distance_m - green_distance_m`。
- 距离只取 `dart_base_link` 的 XY 平面距离，不包含高度差。
- CSV 会根据 URDF 当前限位生成；修改滑轨零位或限位后应重新生成旧 CSV。

工具运行时还会在终端打印各位置、距离和 yaw 关于滑轨位置 `s` 的解析函数。

## License

MIT，见 [LICENSE](LICENSE)。
