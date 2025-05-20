
# Fusion2ROS2 Exporter 

This is an **Autodesk Fusion 360 Add-In** that allows you to export your 3D robot models into a **ROS 2-compatible package** with minimal effort.

The exported package includes:
- URDF files  
- STL meshes  
- Configuration files  
- Launch files  

It’s ready to be used with ROS 2 out of the box and supports simulation in both:
- Gazebo Classic
- Gazebo Sim (Ionic/Harmonic)

---

## Key Features

### ROS 2 Controller Support
- Supports both position and velocity controllers

### Built-in Sensor Plugin Integration
Easily add common Gazebo sensors by renaming your components with the correct suffix.

| Sensor Type     | Suffix | Description              |
|------------------|--------|---------------------------|
| 2D Lidar         | `L2D`  | Line scan sensor          |
| 3D Lidar         | `L3D`  | 3D Lidar                  |
| RGB Camera       | `CAM`  | Standard color camera     |
| RGB-D Camera     | `DCAM` | Depth camera              |
| IMU              | `IMU`  | Inertial measurement unit |
| GPS              | `GPS`  | GNSS receiver             |

**Usage:**  
To attach a sensor to a link, rename the component like this:  
`linkname_SUFFIX` (e.g., `chassis_IMU`).

---

## Simulator Selection

After selecting your destination folder during export, a dialog will prompt for the simulator key:

- `GZC` → Gazebo Classic  
- `GZS` → Gazebo Sim (Ionic)  

Note: These keys are case-sensitive.

---

## Important Design Rules

To ensure a successful export, follow these rules:

1. Your design must include a grounded component named `base_link`  
   - It cannot be the child of any joint
2. Each robot link must be a separate component
3. Nested components (assemblies within assemblies) are not supported
4. Parallel linkages are not supported
5. All joints must be explicitly named  
   - Avoid Fusion's default names with spaces
6. Each component can have only one sensor suffix
7. `base_link` must not have a sensor suffix
8. Components with sensors must:
   - Be connected with a revolute joint
   - Have joint limits: `min = 0`, `max = 0`, `rest = 0`
9. Each component must have a unique name before suffixing
10. Sensor suffixes are case-sensitive
11. Avoid keyword duplication  
    - Example: `IMU_IMU` is invalid

## Additional Considerations

1. When exporting with the `GZS` simulator, the **GPU Lidar plugin** is used. This is the most reliable option when using the **Ogre2 rendering engine** in Gazebo Sim.

2. Your Fusion 360 design file name **should not start with an underscore or an uppercase letter**, as this does not follow ROS package naming conventions.

3. Try to use **simple meshes** in your design. Complex meshes may be harder to render and can significantly impact simulation performance.

4. If simulation-to-reality (sim2real) alignment is important, define a **custom physical material** in Fusion that reflects your robot's real properties. The default material in Gazebo is **steel**, which is quite dense and may not accurately reflect your robot’s behavior.

5. To change sensor output topics in **Gazebo Sim**, do **not** modify the config file. Instead, change the topic name using the `remappings` argument of the **gz_ros_bridge** in your launch files.



This repository is based on these repositories [dheena2k2/fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2.git) and [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf).





## Installation

Run the following command in your shell.

##### Windows (In PowerShell)

```powershell
cd <path to fusion2urdf-ros2>
Copy-Item ".\URDF_Exporter_Ros2\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
```

##### macOS (In bash or zsh)

```bash
cd <path to fusion2urdf-ros2>
cp -r ./URDF_Exporter_Ros2 "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/"
```

## What is this script?
This is a fusion 360 script to export urdf from fusion 360 directly.

This exports:
* .urdf file of your model
* .launch.py files to simulate your robot on gazebo and rviz
* .stl files of your model

### Sample 

The following test model doesn't stand upright because the z axis is not upright in default fusion 360.
Make sure z axis is upright in your fusion 360 model if you want. 

#### original model
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/industrial_robot.png" alt="industrial_robot" title="industrial_robot" width="300" height="300">

#### Gazebo simulation of exported .urdf and .launch.py
* center of mass
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/center_of_mass.png" alt="center_of_mass" title="center_of_mass" width="300" height="300">

* collision
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/collision.png" alt="collision" title="collision" width="300" height="300">

* inertia
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/inertia.png" alt="inertia" title="inertia" width="300" height="300">


## Before using this script

Before using this script, make sure that your model has all the "links" as components. You have to define the links by creating corresiponding components. For example, this model(https://grabcad.com/library/spotmini-robot-1) is not supported unless you define the "base_link". 

In addition to that, you should be careful when define your joints. The **parent links** should be set as **Component2** when you define the joint, not as Component1. For example, if you define the "base_link" as Component1 when you define the joints, an error saying "KeyError: base_link__1" will show up.

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/spot_mini.PNG" alt="spot_mini" title="spot_mini" width="300" height="300">

Also, make sure components of your model has only bodies. **Nested components are not supported**.
For example, this works:

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/only_bodies.PNG" alt="only_bodies" title="only_bodies" width="300" height="300">

but this doesn't work since the "face (3):1" component contains other components. A component must contain only bodies:

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/nest_components.PNG" alt="nest_components" title="nest_components" width="300" height="300">

Sometimes this script exports abnormal urdf without any error messages. In that case, the joints should have problems. Redefine the joints and run again.

In addition to that, make sure that this script currently supports only "Rigid", "Slider" and "Revolute".


## Complex Kinematic Loops and Spherical joints (may be fixed later):

DO NOT use Fusion 360's inbuilt joint editor dialouge for positioning joints

For example, [@rohit-kumar-j](https://github.com/rohit-kumar-j) had this complicated robot to assemble. There are over.. some 50 joints in all, including some forming loops within the structure like a [4-bar mechanism](https://www.youtube.com/watch?v=eYOt6SEKHFs&ab_channel=YuhangHu), also called **kinematic loops**.

![image](https://user-images.githubusercontent.com/37873142/133144979-30218496-09d4-40bb-9af7-95448a7665ee.png)

As you can see below, when fusion initailly forms joints, it might not align where you want it to align to. In the image below, the cylinder's cap side doesn't exaclty coincide with the position of the pin where it needs to be join. The red arrow shows the mismatch in initial joint positioning by fusion.

![image](https://user-images.githubusercontent.com/37873142/133145309-298f17a4-bd62-48fa-b1c2-54f58e26fce4.png)

If you were to manually drag the parts and align them as shown below, it would cause cascading problems with the visual and collision properties of certain links. 

![Capture](https://user-images.githubusercontent.com/37873142/133146628-c4c2b8dd-ac7b-41e8-bd62-1d2c2b80adce.PNG)

Below you can see one of the cylinders is mismatched as compared to the others (red and grey colors are cylinders) 
(The below urdf is visualized in pybullet)

![image](https://user-images.githubusercontent.com/37873142/133141659-440a0a4a-1afa-4751-99ba-fc3db02f7450.png)

See Also: Similar to this issue, but only for a few axes [here](https://github.com/yanshil/Fusion2PyBullet/issues/6) (turns out there was a fusion API change back then, and the exporter wasn't yet updated [See this commit](https://github.com/syuntoku14/fusion2urdf/commit/8786e6318cdcaaf32070148451a27ab6e4f6697d), but it now is)


**The fix for this is to leave Fusion's joint controls unedited and form joints for the robot joints (See below)**


A similar issue with another set of joints at the ankle was fixed by following the above fromat. [Here is the video](https://www.youtube.com/watch?v=0hfkm7vv5o8&ab_channel=JRohit)

For spherical joints, it is better to keep them revolute and define the joints as spherical, later in the generated URDF(provided the urdf parser in your visualizer/physics engine(gazebo,webots,pybullet,mujoco,etc) supports spherical joints, in pybullet it does).
The ankle joint below has 4 spherical joints and only two of them were defined as revolute while exporting from fusion 360. The other 2 spherical joints were created in pybullet using pybullet's inbuilt functions for creating kinematic loops.(see the gif below)

![youtube-video-gif](https://user-images.githubusercontent.com/37873142/133144404-45d9e444-8ddb-4b5f-8970-6e637b750faa.gif)


## In some cases, before export Turn off "Capture design history"

For preplanning the component placement when working/assembling your own robot. It is recommended to have separate names for components and save individual components in a separate folder, create a back up and, break link with the original. This folder can be later deleted after genearating the urdf. See [Issue #51](https://github.com/syuntoku14/fusion2urdf/issues/51) for problem with "copy-paste" vs "copy-paste new".



## How to use

As an example, I'll export a urdf file from this cool fusion360 robot-arm model(https://grabcad.com/library/industrial-robot-10).
This was created by [sanket patil](https://grabcad.com/sanket.patil-16)

### Install in Shell 

Run the [installation command](#installation) in your shell.

### Run in Fusion 360

Click ADD-INS in fusion 360, then choose ****Scripts and Add-Ins > URDF_Exporter_Ros2****. 

**This script will change your model. So before running it, copy your model to backup.**

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/copy.png" alt="copy" title="copy" width="300" height="300">

Run the script and wait a few seconds(or a few minutes). Then a folder dialog will show up. Choose where you want to save the urdf (A folder "Desktop/test" is chosen in this example").
Maybe some error will occur when you run the script. Fix them according to the instruction. In this case, something wrong with joint "Rev 7". Probably it can be solved by just redefining the joint.

![error](https://github.com/syuntoku14/fusion2urdf/blob/images/error.png)

**You must define the base component**. Rename the base component as "base_link". 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/cautions.PNG" alt="cautions" title="cautions" width="300" height="300">

In the above image, base_link is grounded. Right-click it and click "Unground". 

Now you can run the script. Let's run the script. Choose the folder to save and wait for a few seconds. You will see many "old_components" in the components field, please ignore them. 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/result.PNG" alt="results" title="results" width="250" height="300">

You have successfully exported the urdf file. Also, you got `.stl` files in the "Desktop/test/mm_stl" repository. This will be required at the next step. The existing fusion CAD file is no more needed. You can delete it. 

The folder "Desktop/test" will be required in the next step. Move them into your ros environment.


#### In your ROS environment

Place the generated _description package directory in your own ROS workspace. "model_ws" is used in this example.
Then, run catkin_make in catkin_ws.

```bash
cd ~/model_ws/
colcon build
```

Now you can see your robot in rviz by using the following command.

Open a new terminal

```bash
cd ~/model_ws/
source install/setup.bash
ros2 launch (whatever your robot_name is)_description display.launch.py
```

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/rviz_robot.png" alt="rviz" title="rviz" width="300" height="300">

If you want to simulate your robot on gazebo, just run
```bash
ros2 launch (whatever your robot_name is)_description gazebo.launch.py
```

**Enjoy your Fusion 360 and ROS2 life!**
