# 项目名称

这是一个Delta机械臂在Rviz中的仿真

## 快速入门

要开始使用这个项目，请按照以下步骤操作：

1. 在终端中输入以下命令以编译项目：
```shell
mkdir catkin_ws & cd catkin_ws
git clone git@github.com:Dwl2021/Delta-Display.git
catkin_make
source ./devel/setup.bash
. run.sh
```

2. 发送坐标以确定机械臂末端的位置(逆运动学)：

```shell
. pub_point.sh 0.05 -0.03 0.1
```

使用 `pub_point.sh` 脚本发送相对于飞机的坐标，以确定 delta 机械臂末端的位置。三个参数表示机体坐标系下的x,y,z坐标，单位是米(m)。


3. 发送三个弧度来确定舵机运动角度：
```shell
. angles.sh 0.5 0.5 0.5 
```

使用 `angles.sh` 脚本发送舵机转动的角度，从而确定末端的位置，三个参数表示舵机1,2,3的转动角度，要求使用弧度制(rad)。


4. 发送消息改变飞机的位姿：

```shell
. location_plane.sh
```

通过修改``loctaion_plane.sh``脚本修改飞机的位置和姿态，可修改position(单位m)和orientation。

```shell
rostopic pub /pose_plane  geometry_msgs/Pose "position:
  x: 1.0
  y: 1.0
  z: 2.0
orientation:
  x: 0.80
  y: 0.0
  z: 0.0
  w: 0.6"
```


## 参数配置
在 display.launch 文件中，您可以调整以下参数：

```xml
<param name="scale" type="double" value="0.001" />
<param name="staticR" type="double" value="67" />
<param name="dynar" type="double" value="24" />
<param name="upper_delta_display" type="double" value="100" />
<param name="lower_delta_display" type="double" value="160" />
```

这些参数的含义如下：

- scale：用于毫米单位转换为米单位的比例，默认值为 0.001。
- staticR：静平台半径，默认值为 67mm。
- dynar：动平台半径，默认值为 24mm。
- upper_delta_display：主动臂长度，默认值为 100mm。
- lower_delta_display：从动臂长度，默认值为 160mm。

请注意，由于机械臂的比例已经固定，建议仅修改 scale 参数以适应您的需求。
