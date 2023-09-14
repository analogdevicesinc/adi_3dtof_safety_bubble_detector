# Analog Devices 3DToF Safety Bubble Detector

## Overview
The **ADI 3DToF Safety Bubble Detector** is a ROS (Robot Operating System) package for the Safety Bubble Detection application. The Safety Bubble Detectors are the basic building block of any AGV/AMR. 
The safety zone is a virtual area around an AGV/AMR. The Safety Bubble Detectors are used to detect the 
presence of any object inside this zone and prevent the AGV/AMR from colliding on to the object.

The **ADI 3DToF Safety Bubble Detector** is developed as a ROS application running on the ADI’s *EVAL-ADTF3175D-NXZ* Time-of-Flight platform. The Safety Bubble Detection algorithm is highly optimized 
to run at 30FPS on the *EVAL-ADTF3175D-NXZ* platform.
The node uses [*ADI ToF SDK*](https://github.com/analogdevicesinc/ToF/) APIs to capture the frames from the sensor. The algorithm is run on the captured images and the output is published as ROS topics.
The Node publishes the detection flag and the output visualization image as the topics. The Depth and IR images are also published as ROS topics. The topics are published at 30FPS. The default topic names are listed below.

<div style="text-align:center"><img src="./docs/images/adi_3dtof_safety_bubble_detector.png" alt="Connection Diagram"/></div>

![arch_diagram](docs/images/architecture_diagram.png)

[![Noetic](https://img.shields.io/badge/-NOETIC-green?style=plastic&logo=ros)](http://wiki.ros.org/noetic) [![Ubuntu 20.04](https://img.shields.io/badge/-UBUNTU%2020.04-orange?style=plastic&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/focal/) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE) ![ARM64](https://img.shields.io/badge/arm64-blue?style=plastic&logo=arm&logoColor=white) ![x86_64](https://img.shields.io/badge/x86__64-blue?style=plastic&logo=intel&logoColor=white) 

---
# Hardware

- [EVAL-ADTF3175D-NXZ Module](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADTF3175.html#eb-overview)
- USB Type-C to Type-A cable - with 5gbps data speed support
- Host laptop with intel i5 or higher cpu running Ubuntu-20.04LTS

 :memo: _Note_: Refer the [EVAL-ADTF3175D-NXZ User Guide](https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175d-nxz) to ensure the Eval module has adequate power supply during operation.

The image below shows the connection diagram of the setup:

<div style="text-align:center"><img src="./docs/images/connection_diagram.png" alt="Connection Diagram"/></div>
  
  
> :memo:
> **ADSD3500 Firmware :**  
> Make sure the sensor is flashed with the compatible FW. The minimum version is listed below:  
> **CR/DV series : 4.1.0.0**  
> **AM series : 4.2.0.0**  
> Follow the below instructions to read the FW version  
>1. Login to the EVAL-ADTF3175D-NXZ module using ssh. On the Host machine open the “Terminal” and run the following command to ssh to the device.  
>    ```bash
>       $  ssh analog@10.42.0.1
>          Username: analog   
>          Password: analog     
>    ```  
>2. Run the follwing commands
>   ```bash 
>       $ cd ~/Workspace/Tools/ctrl_app
>       $ ./ctrl_app
>   ```
> The output would look like below,  
>   **V4L2 custom control interface app version: 1.0.1**  
>   **59 31**   
>   **<span style="color:red">**04 02 01 00**</span> 61 35 39 61 66 61 64 36 64 36 63   38 65 37 66 62 31 35 33 61 32 64 62 38 63 64 38 38 34 30 33 35 39 66 31 37 31 39 35 61**   
>   **59 31**   
> The first four bytes in the third line represents the FW version. For example for the output above, the version is **4.2.1.0**  
  
> If the firware version is older than this please upgrade the FW using the following instructions

>1. Install ADI ToF SDK release [v4.2.0](https://github.com/analogdevicesinc/ToF/releases/tag/v4.2.0)  
>2. After installing goto the inastallation folder and run the following commands to download the image   
>   ```bash
>       $ cd ~/Analog\ Devices/ToF_Evaluation_Ubuntu_ADTF3175D-Relx.x.x/image.
>       $ chmod +x get_image.sh and ./get_image.sh.
>   ```
>-   Latest image will be downloaded at ./image path as NXP-Img-Relx.x.x-ADTF3175D-.zip. Extract this folder using unzip NXP-Img-Relx.x.x-ADTF3175D-.zip command.
>
>-   This folder contains the NXP image and ADSD3500 firmware(Fw_Update_x.x.x.bin).  

>3. Run the following command to copy the Fimware to the NXP device
>   ```bash
>       $ scp Fw_Update_4.2.4.bin analog@10.42.0.1:/home/analog/Workspace
>          Username: analog 
>          Password: analog
>   ```    
>4. Now login to the device and run the Firmware upgrade command.  
>**:warning: <span style="color:red"> Do not interrupt/abort while the upgrade is in progress.Doing this may corrupt the module.**</span>  
>   ```bash
>        $ ssh analog@10.42.0.1 
>           Username: analog 
>           Password: analog   
>        $ cd Workspace/ToF/build/examples/data_collect/
>        $ ./data_collect --fw ~/Workspace/Fw_Update_x.x.x.bin config/config_default.json
>   ```  
>-  Reboot the board after the successful operation.  
           
For details refer to [EVAL-ADTF3175D-NXZ NVM upgrade guide](https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175d-nxz-upgrade-firmware)
  
  
---

## Getting Started with the SW package

Assumptions before building this package:
* Linux System or WSL2(In real time Only single camera can be connected, for multiple cameras only FileIO mode works.) running Ubuntu 20.04LTS
* To install WSL2 and ROS follow these [steps](https://jackkawell.wordpress.com/2020/06/12/ros-wsl2/) 
* ROS Noetic: If not installed, follow these [steps](http://wiki.ros.org/noetic/Installation/Ubuntu).
* Setup catkin workspace (with workspace folder named as "catkin_ws"). If not done, follow these [steps](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#:~:text=you%20installed%20ROS.-,Create%20a%20ROS%20Workspace,-catkin).

1. Download and install the latest version of *ADI 3DToF Safety Bubble Detector* from the Release pages.

2. After installing the software, go to the installation folder(~/Analog Devices/ADI3DToFSafetyBubbleDetector-Relx.x.x) and run the get_image.sh script. This script will download the custom Ubuntu 20.04 image for the EVAL-ADTF3175D-NXZ. 

3.	Flash .img file to the SD card, follow steps in this link [EVAL-ADTF3175D-NXZ Users Guide](https://wiki.analog.com/resources/eval/user-guides/eval-adsd3100-nxz/flashing_image_instructions) to flash the .img file to SD card.
    
    *Note*: This image contains the necessary software and code to start using the ROS node. The source code for the ```adi_3dtof_safety_bubble_detector``` can be found in ```/home/analog/catkin_ws/src/```

4.	Follow the instructions below to run the *adi_3dtof_safety_bubble_detector* application on the EVAL-ADTF3175D-NXZ module.

5.	Connect the EVAL-ADTF3175D-NXZ module to the PC using the USB3.0 cable and wait for the network to come up. By default, the device ip is set to **10.42.0.1**. Refer to [EVAL-ADTF3175D-NXZ Startup Guide](https://wiki.analog.com/eval-adtf3175d-nxz-startup#software_download) for details.

6.	Login to the EVAL-ADTF3175D-NXZ module using ssh. On the Host machine open the “Terminal” and run the following command to ssh to the device.
    ```bash
    $ ssh analog@10.42.0.1 
      Username: analog 
      Password: analog   
    ```

    *Note*: If you do not have a Linux Host machine, then install Windows Subsystem for Linux(WSL) and Ubuntu 20.04 on Windows. 
    Refer to this [link](https://learn.microsoft.com/en-us/windows/wsl/install) for instructions.


>    :memo:  
> 1. **Setting Date/Time:**  
>Make sure the Date/Time is set properly before compiling and running the application. Connecting to a WiFi network would make sure the Date/Time is set properly. The custom Ubuntu 20.04 image is configured to connect to a network with following SSID and Password by default.  
    ```  
    SSID : ADI,  
    Password: analog123  
    ```  
>You can either setup a network with the above properties or configure the Device to connect to any available network.  
>Alternatively, the Host machine can be setup as a local NTP server and the devices can be configured to update Date/Time using the Host machine.  
>Refer to below links for setting and configuring NTP on Ubuntu machines.
>-  https://ubuntuforums.org/showthread.php?t=862620  
>-  https://timetoolsltd.com/ntp/how-to-install-and-configure-ntp-on-linux/  
>-  https://askubuntu.com/questions/14558/how-do-i-setup-a-local-ntp-server

> 2. The ROS Noetic and dependent packages are already installed in the EVAL-ADTF3175D-NXZ image and the source code for the *adi_3dtof_safety_bubble_detector* is present in `/home/analog/catkin_ws/src/` folder. The package is also pre-built, hence there is no need to build the package.  
>    If the source files are modified, then use the following commands to build the package.  
>>```bash
>> $ cd ~/catkin_ws/  
>> $ catkin_make -DCMAKE_BUILD_TYPE=RELEASE -j2  
>>```
>
>    Note: `/home/analog/catkin_ws/` is set up as the catkin workspace and this workspace is already sourced in the `~/.bashrc`

7.	Running the ROS Node:

    On the EVAL-ADTF3175D-NXZ device, the ROS Master is set to the IP address of the Host machine, hence it is required to run `roscore` on the Host machine (*applicable only to Linux host*).

    On the Linux Host, open a terminal and run the following command
    ```bash
    $ roscore
    ```
    On the Device:
    ```bash
    $ roslaunch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_single_camera.launch
    ```

    >:memo:*Note:*   
    >If you are using WSL as the Host machine, then setting Host as ROS Master does not work. In this case, you must unset the ROS master on device.
    >Please note that only *one device* can be connected to WSL. to use *multiple devices*, use Linux machine as host. 
    >Run the following command to unset the ROS Master and use the EVAL-ADTF3175D-NXZ as the ROS master.
    >On the WSL Host, open an Ubuntu 20.04 Terminal and run the following command
    >```bash
    >$ export ROS_MASTER_URI=http://10.42.0.1:11311
    >$ export ROS_IP=10.42.0.100
    >$ export ROS_HOSTNAME="Your Device name"
    >```
    >On Device,
    >```bash
    >$ unset ROS_MASTER_URI
    >$ roslaunch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_single_camera.launch
    >```

    At this stage, the *adi_3dtof_safety_bubble_detector* will be launched and start publishing the topics ```/cam1/depth_image, /cam1/ir_image, /cam1/out_image, /cam1/obect_detected and /cam1/camera_info```.

    To see the depth and IR images on the Host machine, simply open the RVIZ and add ```/cam1/depth_image```, ```/cam1/ir_image``` and ```/cam1/out_image``` topics to visualize the images

8.  Publishing compressed output image

    This feature enables multiple sensors to run in 30FPS.

    change the below parameter in ```adi_3dtof_safety_bubble_detector_single_camera.launch``` file.
    
    *arg_enable_output_image_compression* set it to 1

    On the Device:
    ```bash
    $ roslaunch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_single_camera.launch
    ```

    On host:
    ```bash
    $ roslaunch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_single_camera_host.launch
    ```
    *Note:* The camera prefix with device is publishing output image should match with ```ns_prefix_cam``` argument present in ```adi_3dtof_safety_bubble_detector_single_camera_host``` launch file.


---
## Output Images

Sample output images are shown below:

```/cam1/depth_image```

![depth_image](docs/images/depth_image.png)


```/cam1/ir_image```

![ir_image](docs/images/ir_image.png)


```/cam1/out_image```

![output_image](docs/images/out_image.png)


### To setup Safety Bubble Detector with 4 devices refer [Setting up 4 device for Safety Bubble Detector](docs/4DevicesSetup.md)

---

# Nodes

## adi_3dtof_safety_bubble_detector_node

### Published topics

These are the default topic names, topic names can be modified as a ROS parameter.

+ **/depth_image**
    - 16-bit Depth image of size 512X512 

+ **/ir_image**
    - 16-bit IR image of size 512X512

+ **/out_image**
    - 8-bit output image of size 512X512

+ **/object_detected**
    - boolean to indicate the object detection

+ **/camera_info**
    - Camera info

If RVL image-compression is enabled:

+ **/depth_image/compressedDepth** 
    - 512X512 16-bit Depth image from adi_3dtof_safety_bubble_detector node compressed with RVL compression 
+ **/ir_image/compressedDepth** 
    - 512X512 16-bit IR image from adi_3dtof_safety_bubble_detector node compressed with RVL compression 
+ **/out_image/compressed** 
    - 512X512 8-bit output image from adi_3dtof_safety_bubble_detector node compressed with RVL compression 

### Parameters

+ **param_camera_link** (String, default: "adi_camera_link")
    - Name of camera Link

+ **param_optical_camera_link** (String, default: "optical_camera_link")
    - Name of optical camera Link

+ **param_virtual_camera_link** (String, default: "virtual_camera_link")
    - Name of virtual camera Link

+ **param_safety_zone_radius_in_mtr** (float, default: 1.0f)
    - Safety zone radius

+ **param_safety_bubble_shape** (int, default: 0)
    - Safety bubble shape, _0:circular, 1:Square_

+ **param_virtual_camera_height** (float, default: 5.0f)
    - Virtual camera height, default value is 5 meters.

+ **param_input_sensor_mode** (int, default: 0)
    - Input mode, _0:Real Time Sensor, 2:Rosbag bin_

+ **param_output_sensor_mode** (int, default: 0)
    - Output mode, _0:No output files written, 1:avi and csv output files are written_

+ **param_input_file_name_or_ros_topic_prefix_name** (String, default: "no name")
    - Input filename : Applicable only if the input mode is 2 or 3
    - If input mode is 2 this parameter represents input file name
    - If input mode is 3 this parameter represents the prefix of ros topics. 

+ **param_enable_ransac_floor_detection** (int, default: 1)
    - enable option for ransac floor detection, _0: disable, 1:enable_

+ **param_enable_depth_ir_compression** (int, default: 0)
    - enable option to publish depth and ir compressed images, _0: disable, 1:enable_

+ **param_enable_output_image_compression** (int, default: 0)
    - enable option to publish compressed output image, _0: disable, 1:enable_

+ **param_safety_bubble_sensitivity** (int, default: 10)
    - number of connected pixels to trigger object detection

+ **param_enable_floor_paint** (int, default: 0)
    - enable option to visualize for floor paint, _0: disable, 1:enable_

+ **param_enable_safety_bubble_zone_visualization** (int, default: 0)
    - enable option to visualize safety bubble zone, _0: disable, 1:enable_

+ **param_input_image_width** (int, default: 1024)
    - image width

+ **param_input_image_height** (int, default: 1024)
    - image height

+ **param_processing_scale** (int, default: 0)
    - scale factor to determine some algorithm parameters

+ **param_ransac_distance_threshold_mtr** (float, default: 0.025f)
    - the height of the floor ransac can find, default value is 2.5 cms

+ **param_ransac_max_iterations** (int, default: 10)
    - maxium iterations ransac is allowed

+ **param_ab_threshold** (int, default: 10)
    - abThreshold for the sensor 
    
+ **param_confidence_threshold** (int, default: 10)
    - confidenceThreshold for the sensor.
    - For Sensor serial number CR and DV default value is 10, for Sensor serial number AM default value is 25.

+ **param_enable_depth_ir_compression** (int, default: 0)
    - Enables RVL compression for the depth images 
    
+ **param_config_file_name_of_tof_sdk** (String, default: "config/config_crosby_old_modes.json")
    - Configuration fie name of ToF SDK  
      _"config_crosby_old_modes.json" - For CR/DV series of Eval Boards  
      "config_crosby_adsd3500_new_modes.json" - For AM series of Eval Boards_

+ **param_frame_type** (String, default: "qmp")
    - Frame Type  
      _"qmp" - For CR/DV series of Eval Boards  
      "lr-qnative" - For AM series of Eval Boards_

## adi_3dtof_safety_bubble_detector_stitch_host_node

### Published topics

+ **/combo_safety_bubble_out_image**
    - 8-bit output image of size 512X512

+ **/combo_safety_bubble_object_detected**
    - boolean topic indicates object detection   

### Subscribed topics

+ **/object_detected**
    - boolean topic indictes object detection

+ **/out_image/compressed**
    - subscribes output image from adi_3dtof_safety_bubble_detector node

### Parameters

+ **camera_prefix** (String, default: "no name")
    - ROS Topic prefix name to subscribe

> :memo: _Notes:_ 
> - _If any of these parameters are not set/declared, default values will be used._

> - _Enabling file input may slow down the speed of publishing._

---
# Dynamic Reconfigure
Using Dynamic Reconfigure some parameters of *adi_3dtof_safety_bubble_detector* ROS node can be modifed during run time. The Perspective file is present in ```rqt_config/``` folder.  

<div style="text-align:center"><img src="./docs/images/adi_3dtof_safety_bubble_detector_rqt.png" alt="Dynamic Reconfigure"/></div>  
The GUI can be started by running the following command.

``` roslaunch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_rqt.launch ```  

Make sure the *adi_3dtof_safety_bubble_detector* is already running before executing this command.

---

---
# Limitations

None

---
# Support

Please contact the `Maintainers` if you want to evaluate the algorithm for your own setup/configuration.

Any other inquiries are also welcome.

---
# Appendix 1:
# Steps to run the Node on a Host machine in File-IO mode
The Node can be run on a Host machine without the need for the actual 3DToF sensor. This mode is supported for users who would want to test some algorithms on the recorded video files. In this mode the *adi_3dtof_safety_bubble_detector_node* will read the video file and publish the frames as ROS topics. Follow the below instructions to build and run the node in File-IO mode.

*Note:* It is assumed that the correct version of ROS is installed and configured properly, if not please install the ROS from [here](http://wiki.ros.org/noetic/Installation/Ubuntu) 

## Requirement on file-io input video files
To run the *adi_3dtof_safety_bubble_detector_node* in file-io mode, the video files should be given as input.
Please follow the below instructions to set up the input video files.
1. After installing the software, go to the installation folder (~/Analog Devices/ADI3DToFSafetyBubbleDetector-Relx.x.x)
2. Run the *get_videos.sh* script which will download the *adi_3dtof_input_video_files.zip* file in the current directory.
3. Unzip it and copy the directory as *~/catkin_ws/src/adi_3dtof_input_video_files*.
4. Update the input file argument *arg_input_file_name_or_ros_topic_prefix_name* in the launch file *adi_3dtof_safety_bubble_detector_single_camera.launch* as per the above file path.

## Steps to run *adi_3dtof_safety_bubble_detector_node* node

1. Clone the repo and checkout the corect release branch/
tag into catkin workspace directory

    ```bash
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/analogdevicesinc/adi_3dtof_safety_bubble_detector.git -b v1.0.0
    ```
2. Install dependencies:
    ```bash
    $ cd ~/catkin_ws/
    $ rosdep install --from-paths src -y --ignore-src    
    ```
3. Build the package
    ```bash
    $ cd ~/catkin_ws/src
    $ catkin_make -DCMAKE_BUILD_TYPE=RELEASE -DHOST_BUILD=TRUE -j2
    $ source devel/setup.bash
    ```
4. To run the *adi_3dtof_safety_bubble_detector_node* in File-IO mode, we need to make some changes in the launch file. Change the following parameters in launch file.

    *arg_input_sensor_mode* to be set to *2*
    
    *arg_input_file_name_or_ros_topic_prefix_name* to be set to the input file name

5. After updating the launch file, run the roslaunch with the updated launch file.
    ```bash
    $ roslaunch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_single_camera.launch
    ```

At this stage, the *adi_3dtof_safety_bubble_detector_node* will be launched and start publishing the topics ```/cam1/depth_image, /cam1/ir_image, /cam1/out_image, /cam1/camera_info and /cam1/object_detected```.

To see the depth and IR images open an other Terminal and open the RVIZ and add ```/cam1/depth_image```, ```/cam1/ir_image``` and ```/cam1/out_image``` topics to visualize the images
    

For details on the parameters please refer to the launch files present in the ```launch/``` folder.
<br>  
<br>

