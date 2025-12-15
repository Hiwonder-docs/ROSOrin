# 1\. ROSOrin User Manual

## 1.1 Introduction

ROSOrin is a versatile ROS-based platform designed for teaching and research. Its patented all-in-one modular chassis makes it easy to switch between Mecanum, Ackermann, and differential drive setups, letting you adapt to different movement modes in seconds. The chassis features a swing arm suspension that keeps all four wheels evenly balanced. This ensures smooth operation on uneven surfaces, prevents wheel slip from affecting motor encoders, and keeps movement stable and efficient. Powered by high-performance hardware—including NVIDIA Jetson, Raspberry Pi 5, LiDAR, 3D depth camera, a 6-microphone array, and an AI voice interaction module—ROSOrin supports a wide range of applications: robot motion control, SLAM navigation, path planning, 3D object recognition, tracking and obstacle avoidance, multi-point navigation, gesture interaction, voice interaction, and sound source localization. With multimodal large AI models and the 6-microphone array, ROSOrin can understand its surroundings, plan actions, and carry out tasks intelligently, opening the door to advanced embodied AI applications.

<img src="../_static/media/chapter_1\section_1/media/image218.png" style="width:600px"  class="common_img" />

### 1.1.1 Packing List

The included components of the ROSOrin robot are listed in the table below.

<img src="../_static/media/chapter_1\section_1/media/image1.png"   class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image2.png"   class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image3.png"   class="common_img" />



## 1.2 Accessories Installation and Startup Preparation

### 1.2.1 Camera Installation

<img src="../_static/media/chapter_1\section_1/media/image219.png"   class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image220.png"   class="common_img" />

### 1.2.2 Voice Module Installation

<img src="../_static/media/chapter_1\section_1/media/image221.png"   class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image222.png"   class="common_img" />

### 1.2.3 Wiring Instruction

#### 1.2.3.1 Jetson Nano Wiring

<img src="../_static/media/chapter_1\section_1/media/image225.png" style="width:600px"  class="common_img" />

<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Access Module List</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        .table-container {
            margin: 0 auto;
            background-color: white;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        table {
            width: 100%;
            table-layout: fixed; /* Ensure equal column widths */
            border-collapse: collapse;
        }
        th, td {
            border: 1px solid #ddd;
            padding: 12px 15px;
            text-align: center;  /* Center text in both th and td */
            vertical-align: middle; /* Center vertically */
        }
        th {
            background-color: #f2f2f2;
            font-weight: bold;
        }
        tr:nth-child(even) {
            background-color: #f9f9f9;
        }
    </style>
</head>
<body>
    <div class="table-container">
        <table>
            <thead>
                <tr>
                    <th>Serial No.</th>
                    <th>Access Module</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td><strong>1</strong></td>
                    <td>Power Supply Port</td>
                </tr>
                <tr>
                    <td><strong>2</strong></td>
                    <td>HDMI Port</td>
                </tr>
                <tr>
                    <td><strong>3</strong></td>
                    <td>LiDAR</td>
                </tr>
                <tr>
                    <td><strong>4</strong></td>
                    <td>Monocular/Depth Camera</td>
                </tr>
                <tr>
                    <td><strong>5</strong></td>
                    <td>STM32 Controller</td>
                </tr>
                <tr>
                    <td><strong>6</strong></td>
                    <td>AI Voice Interaction Box/6-Microphone Array</td>
                </tr>
            </tbody>
        </table>
    </div>
</body>
</html>



#### 1.2.3.2 Jetson Orin Nano / Orin NX Wiring

<img src="../_static/media/chapter_1\section_1/media/image223.png" style="width:600px"  class="common_img" />

<!DOCTYPE html>
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Access Module List</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        .table-container {
            margin: 0 auto;
            background-color: white;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        table {
            width: 100%;
            border-collapse: collapse;
        }
        th, td {
            border: 1px solid #ddd;
            padding: 12px 15px;
            text-align: left;
        }
        th {
            background-color: #f2f2f2;
            font-weight: bold;
        }
        tr:nth-child(even) {
            background-color: #f9f9f9;
        }
    </style>
</head>
<body>
    <div class="table-container">
        <table>
            <thead>
                <tr>
                    <th>Serial No.</th>
                    <th>Access Module</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td><strong>1</strong></td>
                    <td>Jetson Orin Nano/Orin NX Controller Power Supply Port</td>
                </tr>
                <tr>
                    <td><strong>2</strong></td>
                    <td>DC Port</td>
                </tr>
                <tr>
                    <td><strong>3</strong></td>
                    <td>LiDAR</td>
                </tr>
                <tr>
                    <td><strong>4</strong></td>
                    <td>Monocular/Depth Camera</td>
                </tr>
                <tr>
                    <td><strong>5</strong></td>
                    <td>STM32 Controller</td>
                </tr>
                <tr>
                    <td><strong>6</strong></td>
                    <td>AI Voice Interaction Box/6-Microphone Array</td>
                </tr>
            </tbody>
        </table>
    </div>
</body>
</html>



#### 1.2.3.3 Raspberry Pi 5 Wiring

<img src="../_static/media/chapter_1\section_1/media/image224.png" style="width:600px"  class="common_img" />



<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Module Integration List</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        .table-container {
            margin: 0 auto;
            background-color: white;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        table {
            width: 100%;
            border-collapse: collapse;
        }
        th, td {
            border: 1px solid #ddd;
            padding: 12px 15px;
            text-align: left;
        }
        th {
            background-color: #f2f2f2;
            font-weight: bold;
        }
        tr:nth-child(even) {
            background-color: #f9f9f9;
        }
    </style>
</head>
<body>
    <div class="table-container">
        <table>
            <thead>
                <tr>
                    <th>Serial No.</th>
                    <th>Access Module</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td><strong>1</strong></td>
                    <td>LiDAR</td>
                </tr>
                <tr>
                    <td><strong>2</strong></td>
                    <td>Monocular/Depth Camera</td>
                </tr>
                <tr>
                    <td><strong>3</strong></td>
                    <td>STM32 Controller</td>
                </tr>
                <tr>
                    <td><strong>4</strong></td>
                    <td>AI Voice Interaction Box/6-Microphone Array</td>
                </tr>
                <tr>
                    <td><strong>5</strong></td>
                    <td>Ethernet Port</td>
                </tr>
            </tbody>
        </table>
    </div>
</body>
</html>



### 1.2.4 Ackermann Chassis Switch

1. Use an M3 hex screwdriver to remove all five M3 hex screws from the Mecanum chassis.

<img src="../_static/media/chapter_1\section_1/media/image163.png" style="width:800px"  />

2. Take the Ackermann chassis part and attach it to the robot using the M3 hex screws.

<img src="../_static/media/chapter_1\section_1/media/image164.png"  style="width:800px"  />

3. Remove the M4×10 screws inside the wheels, then take out the black M2.5×6 screws from the couplers, and detach both Mecanum wheels.

<img src="../_static/media/chapter_1\section_1/media/image165.png"  style="width:800px"  />

4. Mount the Ackermann wheels using the same black M2.5×6 and M4×10 screws. Once installed, the assembly is complete.

<img src="../_static/media/chapter_1\section_1/media/image166.png" style="width:800px"  />

<img src="../_static/media/chapter_1\section_1/media/image167.png"  style="width:800px"  />



> [!NOTE]
>
> **After assembling the Ackermann chassis, connect the servo to the position shown in the diagram for proper operation.**



### 1.2.5 Differential Drive Chassis Switch

1. Remove the M4×10 screws inside the wheels, then take out the black M2.5×6 screws from the couplers, and detach all the Mecanum wheels.

<img src="../_static/media/chapter_1\section_1/media/image306.png"  style="width:500px"  />

<img src="../_static/media/chapter_1\section_1/media/image307.png"  style="width:500px"  />

2. Mount the four differential wheels using the same black M2.5×6 and M4×10 screws. Once installed, the assembly is complete.
   
   <img src="../_static/media/chapter_1\section_1/media/image308.png"  style="width:500px"  />
   
   <img src="../_static/media/chapter_1\section_1/media/image309.png"  style="width:500px"  />
   



## 1.3 Initial Setup and Power-On

In this section, you will learn about the startup sequence of the robotic arm and verify the functionality of each module. After completing this step, you can proceed to the following chapters to explore app control and wireless controller control.

### 1.3.1 Power-On Preparations

1. To ensure stable operation, recharge the battery promptly when its voltage drops below 10 V.

2. Do not place the robot near the edge of a table to avoid accidental falls and damage.

3. Always operate the robot on a flat, stable surface.

4. Maintain a safe distance from the robot before powering on to prevent accidental contact with moving parts.

5. Before powering on, ensure that the wiring is correct and the robot is fully charged. If the robot doesn't power on, check the wiring of each module.

<p id ="anther3.2"></p>

### 1.3.2 Power-On Status

1. Ensure that the robot’s power switch is not turned on.
   
   <img src="../_static/media/chapter_1\section_1/media/image43.png"  width="505" height="325" />
   
2. Remove the hex screws from the bottom of the robot and open the metal cover.
   
   <img src="../_static/media/chapter_1\section_1/media/image44.png" width="478" height="369" />
   
3. Make sure to connect the red wire to the red terminal and the black wire to the black terminal as shown in the figure below. The connectors are designed with a reverse polarity prevention feature. Do not force them if they don’t match, you may rotate the connector and try again. Then, reassemble the metal cover.
   
   <img src="../_static/media/chapter_1\section_1/media/image45.png"  width="491" height="354" />
   


<img src="../_static/media/chapter_1\section_1/media/image46.png"  width="491" height="354" />

4. Place the robot on a flat, smooth surface and press the switch located on the rear left side of the robot.

<img src="../_static/media/chapter_1\section_1/media/image43.png"  width="491" height="354" />

5. The blue LED1 at the lower right of the expansion board will light up and start blinking. At this stage, only the network configuration service is running, the ROS system and other services have not yet fully started. Wait until the buzzer emits a short beep — this indicates the system has finished booting.

(1) The LED locations on the Jetson controllers are shown in the diagram below.

<img src="../_static/media/chapter_1\section_1/media/image50.png" class="common_img" />

(2) The LED locations on the Raspberry Pi 5 controller are shown in the diagram below.

<img src="../_static/media/chapter_1\section_1/media/image226.png" class="common_img" />

6. By default, the device is configured in AP direct-connect mode. After startup, a Wi-Fi hotspot beginning with “HW” will appear. To connect via mobile app or remote desktop, enter the default password: hiwonder.

<img src="../_static/media/chapter_1\section_1/media/image243.png" class="common_img" />

> [!NOTE]
>
> **If the hotspot does not appear after startup, troubleshoot as follows:**
>
> * **Verify that all steps in the Power-On Status section have been followed.**
> * **If LED1 stays solid blue instead of blinking, the system may be in LAN mode. Long-press the KEY1 button on the expansion board for 5–10 seconds. If LED1 starts blinking, the HW Wi-Fi hotspot has been re-enabled.**
> * **If LED1 does not blink after pressing KEY1, the system may not detect the SD card or SSD. Remove and reinsert the SD card.**
> * **If the LED remains solid after reinserting, the SD card may be corrupted, or the system image has not been flashed for kits without a controller. You may replace or reflash the image to the SD card.**
> * **If the issue persists, the Raspberry Pi 5 controller or Jetson controller may be faulty. Please contact customer support for assistance.**

The following table outlines how to test each hardware module.

<table>
  <tr>
    <th>Module</th>
    <th>Verification Steps</th>
    <th>Result</th>
  </tr>
  <tr>
    <td>Extension Board LED</td>
    <td>Observe the LED's lighting and flashing behavior</td>
    <td>By default, the LED is blue and flashing in AP direct connection mode, indicating that network service configuration is complete.</td>
  </tr>
  <tr>
    <td>Buzzer</td>
    <td>Check for a short beep</td>
    <td>A short beep from the buzzer indicates that the onboard hardware of the extension board is functioning normally.</td>
  </tr>
  <tr>
    <td>LiDAR</td>
    <td>Observe the rotation</td>
    <td>The built-in blue LED of the LiDAR lights up and rotates continuously.</td>
  </tr>
  <tr>
    <td>KEY1 on Extension Board</td>
    <td>Switch network status</td>
    <td>After connecting to the STA local network mode via the mobile app, press and hold the KEY1 button to check if the LED1 indicator flashes.</td>
  </tr>
  <tr>
    <td>Microphone, Sound Card, Speaker</td>
    <td>Say Hello Hiwonder to the robot after powering on</td>
    <td>There is feedback to the wake word, and the speaker plays the response "I'm here." Only available for the kits that include a voice device.</td>
  </tr>
  <tr>
    <td>Depth Camera</td>
    <td>1. Open the app and connect to the robot.<br>2. Open the "Robot Control" function and check the real-time feed from the depth camera.</td>
    <td>Displays the real-time feed and rotates.</td>
  </tr>
  <tr>
    <td>STM32 Controller + Encoder DC Gear Motor</td>
    <td>After powering on, use the Robot Control function via the wireless controller or the app.</td>
    <td>The robot moves normally.</td>
  </tr>
</table>






## 1.4 Battery Usage and Charging Instructions

Since the robot must be powered off during transportation and the battery cannot be fully charged, you need to connect the battery cable to charge the battery before first use. Charging the battery from 10 V to about 12.3 V takes approximately three hours.

> [!NOTE]
>
> **When the battery voltage falls below 10V, the buzzer will emit a "beep-beep-beep" low voltage warning. If the battery is low, turn off the robot immediately and charge the battery according to the recommended charging procedure.**

### 1.4.1 Lithium Battery Care

1\. Always use the dedicated charger included with the kit to charge the robot. Turn off the robot while charging. Do not operate the robot and charge the battery at the same time.

2\. When the charger is connected to the battery but not plugged into a power outlet, the indicator light shows green. During charging, the indicator light turns red. When fully charged, it will return to green.

Do not plug the charger directly into the DC power input on the Jetson controller, as shown in the diagram below, as it may damage the controller. The image is for reference only.

<img src="../_static/media/chapter_1\section_1/media/image42.png" style="width:600px" />

4\. Disconnect the charging cable promptly after charging to avoid overcharging and battery damage.

5\. To ensure stable robot performance, recharge the battery when its voltage drops below 10 V, indicated by a beeping buzzer on the expansion board.

6\. If the robot will not be used for an extended period, fully charge the battery and disconnect the battery cable.

7\. Store the battery in a cool, dry place to prevent reduced lifespan due to overheating or moisture. Do not hit, throw, or step on the battery.

8\. Do not use the battery in environments with strong static electricity or magnetic fields, as this may damage its safety protection circuitry.

9\. Do not plug the battery directly into a wall socket. Do not short-circuit the battery terminals with metal objects.

10\. Over-discharging may prevent the battery from recharging and could render it unusable. For long-term storage, fully charge the battery first.

11\. Do not attempt to modify, solder, or alter the battery or charger in any way.

12\. Keep batteries away from high temperatures and liquids to avoid overheating, fire hazards, or moisture-related damage.

Important Notice: Hiwonder is not responsible for any damage, economic loss, or safety incidents resulting from improper use of the product that does not follow the instructions outlined in this manual.

### 1.4.2 Charging Instructions

1. The power input is located on the right side at the back of the robot. Plug in the charger to begin charging.

<img src="../_static/media/chapter_1\section_1/media/image47.png"  style="width:600px" />

<img src="../_static/media/chapter_1\section_1/media/image48.png" style="width:600px" />

2. Check the indicator light on the charger to monitor the charging status. The indicator shows red while charging and turns green when charging is complete.

<img src="../_static/media/chapter_1\section_1/media/image49.png" width="377" height="230" />

> [!NOTE]
>
> **After charging is complete, unplug the charger promptly to prevent overcharging.**



## 1.5 App Installation and Connection

> [!NOTE]
> 
> * **Please grant all permissions requested during installation to ensure the app functions properly.**
> 
> * **Turn on your phone’s GPS and Wi-Fi before opening the app.**

This section explains how to install the **WonderAI** app to control the robot.

<p id ="anther5.1"></p>

### 1.5.1 App Installation

The app installation package is located in the directory:  
[2 Softwares\\1. App Installation Package](https://drive.google.com/drive/folders/1mHf3Gjlc3ENjyu-mNOEsJ4TomKiiId-S). Transfer the APK file to your phone and install it.

Or scan the QR code below to download the app.

<img src="../_static/media/chapter_1\section_1/media/image227.png" style="width:300px" class="common_img" />

### 1.5.2 Connection Modes

After installing the app, you can proceed to connect the robot. The robot supports two network modes:

1. AP Mode (Direct Connection): The controller creates a hotspot that your phone can connect to directly, but no Internet access in this mode.

2. STA Mode (LAN Connection): The controller connects to a specified Wi-Fi network, and Internet access is available in this mode.

By default, the robot starts in AP direct connection mode. Regardless of whether the user chooses AP direct connection or STA LAN mode, the robot’s features remain the same.

Tip: We recommend starting with the AP direct connection mode to quickly explore and experience the robot’s functions. You can switch to LAN mode later based on your specific needs.

#### 1.5.2.1 AP Mode Connection (Must Read)

> [!NOTE]
>
> **This section demonstrates the connection using the Android device and the Mecanum wheel version. If switching to the Ackermann or differential drive versions, refer to the Changing Chassis Type section first, then follow this procedure for connection.**

1. Open the **WonderAi** app and select **Developer** > **ROSOrin Mecanum**.

<img src="../_static/media/chapter_1\section_1/media/image254.png" style="width:600px" class="common_img" />

2. Tap the **+** button in the bottom-right corner and choose **Direct Connection Mode**.

<img src="../_static/media/chapter_1\section_1/media/image255.png" style="width:600px" class="common_img" />

3\. Search for the robot's WiFi network. The hotspot name starts with **HW**, and the password is **hiwonder**.

<img src="../_static/media/chapter_1\section_1/media/image243.png" style="width:400px" class="common_img" />

4\. Return to the app, tap the corresponding robot icon, and enter the mode selection screen.

<img src="../_static/media/chapter_1\section_1/media/image256.png" style="width:600px" class="common_img" />

> [!NOTE]
>
> **If a message pops up saying “Network is unavailable, continue?”, tap “Keep Connection” to proceed.**

5. If a message appears saying **Whether to switch and enter the searched product interface?**, it means the wrong product was selected in Step 1. Tap **Confirm** to automatically switch to the correct version’s mode selection screen.

<img src="../_static/media/chapter_1\section_1/media/image56.png" style="width:500px" class="common_img" />

6. The mode selection interface is shown below.

<img src="../_static/media/chapter_1\section_1/media/image57.png" style="width:600px" class="common_img" />

<p id ="anther5.2.2"></p>

#### 1.5.2.2 LAN Mode Connection (Optional)

> [!NOTE]
>
> **After setting the robot to LAN mode, it will no longer broadcast a WiFi network starting with HW.**

1. First, connect your phone to a 5G Wi-Fi network. For example, connect to **Hiwonder_5G**. A dual-band router, when configured with separate SSIDs for each band, will have distinct default Wi-Fi names for the two frequencies.
2. Open the **WonderAi** app and select **Developer** > **ROSOrin**.

<img src="../_static/media/chapter_1\section_1/media/image254.png" style="width:600px" class="common_img" />

3. Tap the **+** button in the bottom-right corner and choose **LAN Mode**.

<img src="../_static/media/chapter_1\section_1/media/image257.png" style="width:600px" class="common_img" />

4. The app will prompt you to enter the Wi-Fi password for the network you're connected to. Make sure you enter the correct password, a wrong password will prevent the connection. After entering the password, tap **OK**.

<img src="../_static/media/chapter_1\section_1/media/image258.png" style="width:400px" class="common_img" />

5. Tap **Go to connect device hotspots** to switch to the Wi-Fi settings.

<img src="../_static/media/chapter_1\section_1/media/image260.png" style="width:600px" class="common_img" />

6\. In the Wi-Fi list, find the hotspot starting with **HW**, and connect to it using the password **hiwonder**. After connecting, tap the **Back** icon to return to the app.

7\. The app has started connecting to the robot.

<img src="../_static/media/chapter_1\section_1/media/image261.png" style="width:300px" class="common_img" />

8\. After a few seconds, the robot's icon and name will appear on the main screen. The LED1 indicator on the expansion board will stay on.

<img src="../_static/media/chapter_1\section_1/media/image256.png" style="width:600px" class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image64.png" style="width:400px" class="common_img" />

9. Press and hold the robot’s icon to display its current IP address.

<img src="../_static/media/chapter_1\section_1/media/image263.png" style="width:400px" class="common_img" />

10. The IP address can be entered in the remote desktop client to establish a connection. For detailed connection instructions, refer to section[ 7. Development Environment Setup](#anther7.0).

### 1.5.3 App Control

You can control the robot via the WonderAi app and explore its AI vision features. This section explains the operation of each function within the app. This section demonstrates the process using an iOS device, but the same method applies to Android devices.

#### 1.5.3.1 Preparation

1. First, power on the robot. For details on startup status, refer to section [3.2  Power-On Status](#anther3.2).

2. Next, install the WonderAi app and connect the robot. For step-by-step instructions, see section [5.1 App Installation](#anther5.1).

#### 1.5.3.2 App Modes

The app provides five modes, including Robot Control, Lidar, Target Tracking, Line Following, and Driverless.

<img src="../_static/media/chapter_1\section_1/media/image65.png" style="width:600px" />

The table below offers a detailed overview of each mode.

<table>
  <thead>
    <tr>
      <th>Icon</th>
      <th>Mode</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image66.png" /></td>
      <td>Robot Control</td>
      <td>Control the movement of the robot.</td>
    </tr>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image67.png" /></td>
      <td>LiDAR</td>
      <td>Provides three functions: LiDAR obstacle avoidance, LiDAR following, and LiDAR guarding.</td>
    </tr>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image68.png" /></td>
      <td>Target Tracking</td>
      <td>Select the color of the target object, and the robot will track it.</td>
    </tr>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image69.png" /></td>
      <td>Line Following</td>
      <td>Lay down a line and set its color as the target recognition color. The robot will follow the line.</td>
    </tr>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image70.png" /></td>
      <td>Driverless</td>
      <td>Experience the autonomous driving feature.</td>
    </tr>
  </tbody>
</table>




#### 1.5.3.3 Robot Control

Tap Robot Control on the mode selection screen to enter the control interface.

<img src="../_static/media/chapter_1\section_1/media/image71.png" style="width:600px" class="common_img" />

The interfaces for the Mecanum/Differential and Ackermann versions differ slightly. The Mecanum/Differential interface is shown below:

- #### Mecanum/Differential Drive Chassis

<img src="../_static/media/chapter_1\section_1/media/image72.png" style="width:600px" class="common_img" />

1\. The buttons on the left, from top to bottom, correspond to gravity control, forward/backward movement, and speed adjustment.

2\. The center displays the feedback video.

3\. The buttons on the right, from top to bottom, control steering.

4\. The top menu icons allow <img src="../_static/media/chapter_1\section_1/media/image73.png" width="37" height="33" style="display:inline;vertical-align:middle;"/> screenshot, <img src="../_static/media/chapter_1\section_1/media/image74.png" width="37" height="34" style="display:inline;vertical-align:middle;"/> hide navigation bar, and <img src="../_static/media/chapter_1\section_1/media/image75.png" width="37" height="32" style="display:inline;vertical-align:middle;"/> switch to full-screen mode. This function is typically used alongside the wireless controller.

5\. Clicking the full-screen button <img src="../_static/media/chapter_1\section_1/media/image75.png" width="37" height="32" style="display:inline;vertical-align:middle;"/> displays the feedback video in full screen, which is useful for monitoring real-time video while controlling the robot with the controller.

<img src="../_static/media/chapter_1\section_1/media/image76.png" style="width:600px" class="common_img" />

- #### Ackermann Chassis

<img src="../_static/media/chapter_1\section_1/media/image77.png" style="width:600px" class="common_img" />

For more details:

1\. The buttons on the left, from top to bottom, control forward/backward movement and speed adjustment.

<img src="../_static/media/chapter_1\section_1/media/image78.png" width="189" height="186" />

The forward/backward slider controls the robot chassis movement.

<img src="../_static/media/chapter_1\section_1/media/image79.png" width="468" height="138" />

Dragging the slider adjusts the speed of the robot.

2\. The right side displays the feedback video from the depth camera, serving as the video transmission function.

<img src="../_static/media/chapter_1\section_1/media/image80.png" style="width:600px" class="common_img" />

3\. Within the feedback video panel, the buttons from top to bottom control the front wheel steering.

<img src="../_static/media/chapter_1\section_1/media/image81.png" width="375" height="147" />

For the Ackermann chassis, dragging the horizontal slider adjusts the rotation of the front wheels.

#### 1.5.3.4 LiDAR

> [!NOTE]
> 
> * **Before stating the feature, ensure that the robot is on a spacious surface with enough room to move freely.**
> 
> * **In LiDAR Obstacle Avoidance and LiDAR Following modes, the detection range is a 90° fan-shaped area in front of the robot.**
> 
> * **The LiDAR Guard mode is not supported when using the Ackerman chassis.**

- #### Interface Overview

Tap **LiDAR** on the mode selection screen to enter the control interface.

<img src="../_static/media/chapter_1\section_1/media/image82.png" style="width:600px" class="common_img" />

The LiDAR mode includes three features: Avoid obstacle, Lidar following, and Lidar guarding. This interface is divided into two sections:

1. In the left panel, enable or disable the feature.

2. In the right panel, display the live video feed from the camera.

<img src="../_static/media/chapter_1\section_1/media/image83.png" style="width:600px" class="common_img" />

- #### Function

<table>
  <thead>
    <tr>
      <th>Icon</th>
      <th>Function Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image84.png" width="150" height="35" alt="LiDAR Obstacle Avoidance Switch Icon"></td>
      <td>Turn on/off LiDAR obstacle avoidance mode.</td>
    </tr>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image85.png" width="150" height="35" alt="LiDAR Follow Switch Icon"></td>
      <td>Turn on/off LiDAR following mode.</td>
    </tr>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image86.png" width="150" height="35" alt="LiDAR Guard Switch Icon"></td>
      <td>Turn on/off LiDAR guarding mode.</td>
    </tr>
    <tr>
      <td><img src="../_static/media/chapter_1\section_1/media/image87.png" width="141" height="73" alt="Camera Feed Icon"></td>
      <td>Display the current camera feed.</td>
    </tr>
  </tbody>
</table>




- #### Operating Steps and Effects

1. **Avoid obstacle**

   The robot continuously moves forward. When an obstacle is detected, it automatically changes direction to avoid it.

2. **Lidar following**

   When an obstacle is detected, the robot adjusts its position to maintain a safe distance from the object.

3. **Lidar guarding**

   When an obstacle is detected, the robot turns to face the object.

#### 1.5.3.5 Target Tracking

> [!NOTE]
> 
> * **Place the target object on the same surface as the robot and move it in a horizontal direction, which ensures a smoother tracking experience.**
> 
> * **Choose an appropriate color range for target extraction. If the range is too wide, unwanted colors may be included. If the range is too narrow, the target may be lost. Also, avoid having objects with similar colors to the target in the camera’s view.**

- #### Interface Overview

Tap **Target Tracking** on the mode selection screen to enter the control interface.

<img src="../_static/media/chapter_1\section_1/media/image88.png" style="width:600px" class="common_img" />

This interface is divided into two sections:

1. The left panel contains the mode switch and color extraction tools.

2. In the right panel, display the live video feed from the camera.

<img src="../_static/media/chapter_1\section_1/media/image89.png" style="width:600px" class="common_img" />

- #### Function

<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <title>Icon Function Description</title>
    <style>
        table {
            width: 100%;
            border-collapse: collapse;
            margin: 50px 0;
        }
        th, td {
            border: 1px solid #ddd;
            padding: 50px;
            text-align: left;
        }
        th {
            background-color: #f2f2f2;
            font-weight: bold;
        }
        img {
            display: block;
            margin: 0 auto;
        }
    </style>
</head>
<body>
    <table>
        <thead>
            <tr>
                <th>Icon</th>
                <th>Function Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><img src="../_static/media/chapter_1\section_1/media/image90.png" width="200" height="50" alt="Mode Switch Icon"></td>
                <td>Turn on/off the mode.</td>
            </tr>
            <tr>
                <td><img src="../_static/media/chapter_1\section_1/media/image91.png" width="200" height="40" alt="Color Adjustment Icon"></td>
                <td>Adjust the color threshold range, with a value range of 0.05-1.00.</td>
            </tr>
            <tr>
                <td><img src="../_static/media/chapter_1\section_1/media/image92.png" width="200" height="66" alt="Color Extraction Icon"></td>
                <td>Extract the color from the specified area in the feed.</td>
            </tr>
            <tr>
                <td><img src="../_static/media/chapter_1\section_1/media/image93.png" width="200" height="60" alt="Color Confirmation Icon"></td>
                <td>After clicking the "Pick" button, it will switch to the "OK" button. Used to confirm the extracted color.</td>
            </tr>
            <tr>
                <td><img src="../_static/media/chapter_1\section_1/media/image94.png" width="150" height="120" alt="Color Display Icon"></td>
                <td>Display the extracted color.</td>
            </tr>
            <tr>
                <td><img src="../_static/media/chapter_1\section_1/media/image95.png" width="150" height="78" alt="Camera Feed Icon"></td>
                <td>Display the current camera feed.</td>
            </tr>
        </tbody>
    </table>
</body>
</html>

- #### Operating Steps and Effects

1\. Tap the **Pick** button, then drag the red circle in the video feed to the target object to select its color.

<img src="../_static/media/chapter_1\section_1/media/image96.png" style="width:600px" class="common_img" />

2\. Tap **OK**, and the selected color will be displayed in the **Selected color** area.

3\. Tap the **Start** button to activate the feature. Move the target object, and the robot will follow accordingly.

#### 1.5.3.6 Line Following

> [!NOTE]
> 
> * **Before starting this feature, lay out the tracking path with tape and place the robot on the track.**
> 
> * **Choose an appropriate color range for target extraction. If the range is too wide, unwanted colors may be included. If the range is too narrow, the target may be lost. Also, avoid having objects with similar colors to the target in the camera’s view.**
> 
> * **After the feature starts, please ensure there is no other object containing the recognition color except the target object within the camera view. Otherwise, the recognition result will be affected.**

- #### Interface Overview

Tap **Line Following** on the mode selection screen to enter the control interface.

<img src="../_static/media/chapter_1\section_1/media/image98.png" style="width:600px" class="common_img" />

This interface is divided into two sections:

1. The left panel contains the mode switch and color extraction tools.

2. In the right panel, display the live video feed from the camera.

<img src="../_static/media/chapter_1\section_1/media/image99.png" style="width:600px" class="common_img" />

- #### Function

<table>
  <tr>
    <th>Icon</th>
    <th>Function Description</th>
  </tr>
  <tr>
    <td><img src="../_static/media/chapter_1\section_1/media/image90.png" width="200" height="50" /></td>
    <td>Turn on/off the mode.</td>
  </tr>
  <tr>
    <td><img src="../_static/media/chapter_1\section_1/media/image91.png" width="200" height="36" /></td>
    <td>Adjust the color threshold range, with a value range of 0.05-1.00.</td>
  </tr>
  <tr>
    <td><img src="../_static/media/chapter_1\section_1/media/image92.png" width="200" height="50" /></td>
    <td>Extract the color from the specified area in the feed.</td>
  </tr>
  <tr>
    <td><img src="../_static/media/chapter_1\section_1/media/image93.png" width="150" height="50" /></td>
    <td>After clicking the "Pick" button, it will switch to the "OK" button. Used to confirm the extracted color.</td>
  </tr>
  <tr>
    <td><img src="../_static/media/chapter_1\section_1/media/image94.png" width="150" height="90" /></td>
    <td>Display the extracted color.</td>
  </tr>
  <tr>
    <td><img src="../_static/media/chapter_1\section_1/media/image102.png" width="151" height="101" /></td>
    <td>Display the current camera feed.</td>
  </tr>
</table>




- #### Operating Steps and Effects

1\. In this section, red tape is used as an example. After clicking the **Pick** button, drag the red circle in the live feed to the track to select its color.

2\. Tap **OK**, and the selected color will be displayed in the **Selected color** area.

3\. Tap **Start** to begin the feature, and the robot will follow the colored line automatically.

<img src="../_static/media/chapter_1\section_1/media/image104.png" style="width:600px" class="common_img" />

#### 1.5.3.7 Driverless

- #### Interface Overview

Tap **Driverless** on the mode selection screen to enter the control interface.

<img src="../_static/media/chapter_1\section_1/media/image105.png" style="width:600px" class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image106.png" style="width:600px" class="common_img" />

This interface is used for the autonomous driving feature. It is mainly divided into two sections:

1. The **Start** button on the left is used to activate the autonomous driving function.

2. The central area displays the live feed from the camera.

- #### Function

| **Icon**| **Function**|
|----------|----------|
| <img src="../_static/media/chapter_1\section_1/media/image90.png" width="94" height="31" />| Enable and disable the feature.|
| <img src="../_static/media/chapter_1\section_1/media/image107.png" width="151" height="99" />| Display real-time detection results for traffic signs, lane markings, and traffic lights.|

- #### Operating Steps and Effects

Click the **Start** button to begin the autonomous driving feature. The robot will drive automatically along the center of the lane, slow down when approaching pedestrian crossings, respond to traffic signs accordingly, and follow traffic light signals, such as stopping at red lights and moving on green lights.

<img src="../_static/media/chapter_1\section_1/media/image109.png" style="width:600px" class="common_img" />



## 1.6 Wireless Controller Control

### 1.6.1 Notes

1\. Before powering on the device, make sure the wireless controller receiver is properly inserted. This can be ignored if the receiver was pre-inserted at the factory.

2\. Pay attention to battery polarity when placing the batteries.

<img src="../_static/media/chapter_1\section_1/media/image110.png"  width="529" height="202" />

(1) Each time the robot is powered on, the app auto-start service will launch, which includes the wireless controller control service. If this service has not been closed, no additional actions are needed—simply connect and control.

(2) Since signals from wireless controllers can interfere with each other, it is recommended not to use this function when multiple robots are in the same area to avoid misconnection or unintended control.

(3) After turning on the wireless controller, if it does not connect to the robot within 30 seconds or remains unused for 5 minutes after connection, it will enter sleep mode automatically. **To wake up the wireless controller and exit sleep mode, press the “START” button.**

(4) If the robot does not respond while using the controller, try starting the relevant services. For detailed steps, refer to the document 2. Motion Control Course.

### 1.6.2 Device Connection

1. After the robot powers on, slide the wireless controller switch to the **ON** position. At this point, the red and green LED indicators on the wireless controller will start flashing simultaneously.

2. Wait a few seconds for the robot and wireless controller to pair automatically. Once pairing is successful, the green LED will remain solid while the red LED turns off.

<img src="../_static/media/chapter_1\section_1/media/image111.png" width="493" height="198" />

### 1.6.3 Button Functions

The following table describes the functions of the controller buttons and joysticks from the robot’s first-person perspective:

> [!NOTE]
>
> **Lightly pushing the joysticks in any direction allows low-speed movement.**

| **Button**| **Function**| **Description**|
|----------|----------|----------|
| START| Stop and reset the robot.| Press|
| Left joystick up| Move forward| Push|
| Left joystick down| Move backward| Push|
| Left joystick left| Move left, only for Mecanum chassis.| Push|
| Left joystick right| Move right, only for Mecanum chassis.| Push|
| Right joystick left| Turn left, only controls front wheels on Ackerman chassis| Push|
| Right Joystick right| Turn right, only controls front wheels on Ackerman chassis| Push|

<p id ="anther7.0"></p>

## 1.7 Development Environment Setup

### 1.7.1 Remote Control Tool Introduction and Installation

#### 1.7.1.1 Tool Introduction

There are two ways to remotely control the robot: graphical control and command-line control.

NoMachine and VNC are graphical remote-control softwares. After installation, you can connect to the robot’s hotspot and directly control the robot from your computer. The Jetson Nano, Jetson Orin Nano, and Jetson Orin NX controllers use NoMachine for connection, while the Raspberry Pi 5 controller uses VNC. When connected via the two softwares, you can clearly see the robot’s system desktop, making it convenient for intuitive operation.

In contrast to NoMachine and VNC, MobaXterm connects via SSH and focuses on command-line control. It does not display the robot’s full system desktop—only the command-line interface. For users who are familiar with terminal commands, this method allows faster robot control while reducing computational load and memory usage.

MobaXterm also comes with a lightweight X11 server, which enables graphical applications to be displayed directly when needed. Regardless of which controller you are using, MobaXterm’s SSH connection method is always supported.

In a word, NoMachine and VNC are best for scenarios requiring intuitive, visual operation, while MobaXterm is better suited for fast command execution. Choose the remote-control software according to your specific needs.

#### 1.7.1.2 Nomachine Installation

> [!NOTE]
>
> **This tool is compatible with Jetson series controllers.**

1\. Navigate to the directory and double-click to open the folder [2. Softwares / 2. Remote Desktop Software / 1. Graphical Remote Desktop Access Tool](https://drive.google.com/drive/folders/1r76Nz2xTq8l1RhrOdA8oRi_nQzrka6Q3), and then double-click to open the installation package **nomachine_8.4.2_10_x64**.

2\. Click **Next**.

<img src="../_static/media/chapter_1\section_1/media/image112.png" width="453" height="351" />

3\. Select **English** as the installation language. Accept the license agreement by checking the box, then click **Next**.

<img src="../_static/media/chapter_1\section_1/media/image113.png" width="453" height="351" />

4\. Keep the default installation path and click **Next** again.

<img src="../_static/media/chapter_1\section_1/media/image114.png" width="453" height="351" />

5\. Wait for a moment, and the setup completion screen will appear. Click **Finish** to exit the installer.

<img src="../_static/media/chapter_1\section_1/media/image115.png" width="453" height="351" />

6\. Click **Yes** to restart the computer. This step must not be skipped!

<img src="../_static/media/chapter_1\section_1/media/image116.png" width="453" height="210" />

#### 1.7.1.3 VNC Installation

> [!NOTE]
>
> **This tool is compatible with Raspberry Pi controllers.**

1. Locate the folder at: [2. Software \\ 2. Remote Desktop Software \\ 1. Graphical Remote Desktop Access Tool](https://drive.google.com/drive/folders/1r76Nz2xTq8l1RhrOdA8oRi_nQzrka6Q3) to install it. Double-click the file **VNC-Viewer-6.17.731-Windows** in this folder. In the pop-up dialog, select **English** as the installation language and click **OK**.

<img src="../_static/media/chapter_1\section_1/media/image228.png" width="453" height="210" />

2. On the next screen, click **Next**.

<img src="../_static/media/chapter_1\section_1/media/image229.png" width="453" height="351" />

3. Agree to the license agreement, then click **Next**. Keep the default installation path, and click **Next**.

<img src="../_static/media/chapter_1\section_1/media/image230.png" width="453" height="351" />

4. Click **Install** to begin installation.

<img src="../_static/media/chapter_1\section_1/media/image231.png" width="453" height="351" />

5. Wait for the installation to complete, then click **Finish** to complete the setup.

<img src="../_static/media/chapter_1\section_1/media/image232.png" width="453" height="351" />

6. After installation, launch VNC Viewer by clicking its desktop icon <img src="../_static/media/chapter_1\section_1/media/image233.png"  />.

#### 1.7.1.4 MobaXterm Installation

> [!NOTE]
>
> **This tool is compatible with any controller.**

1. Locate the folder: [2. Software \\ 2. Remote Desktop Software \\ 2. SSH Remote Client](https://drive.google.com/drive/folders/1tmIAnsntM6cYGNldA9xtH_V3_THMVNdx), and open the installer and follow the prompts to complete the installation.

<img src="../_static/media/chapter_1\section_1/media/image234.png" class="common_img"  />

2. Click **Next**.

<img src="../_static/media/chapter_1\section_1/media/image235.png" width="453" height="351"  />

3. Accept the license agreement by checking the box, then click **Next**.

<img src="../_static/media/chapter_1\section_1/media/image236.png" width="453" height="351"  />

4. Keep the default installation path and click **Next** again.

<img src="../_static/media/chapter_1\section_1/media/image237.png" width="453" height="351"  />

5. Click **Install**.

<img src="../_static/media/chapter_1\section_1/media/image238.png" width="453" height="351"  />

6. Wait for a moment, the setup completion screen will appear. Click **Finish** to exit the installer.

<img src="../_static/media/chapter_1\section_1/media/image239.png" width="453" height="351"  />

<p id ="anther7.2"></p>

### 1.7.2 AP Mode Connection Steps

**AP Mode (Direct Connection): The controller creates a hotspot that your phone and computer can connect to directly, but cannot access external networks.**

> [!NOTE]
>
> Before connecting the Jetson Orin Nano or Jetson Orin NX controller via NoMachine, ensure that a virtual display or an external monitor is connected. **The connection location is shown in the diagram below at port 2.**. This section provides only a general overview.

<img src="../_static/media/chapter_1\section_1/media/image223.png" style="width:600px"  class="common_img" />

<p id ="anther7.2.1"></p>

#### 1.7.2.1 Connecting via NoMachine

**AP Mode (Direct Connection): The controller creates a hotspot that your phone can connect to directly, but cannot access external networks.**

1\. The robot is set to AP mode by default. After powering it on, it will generate a Wi-Fi hotspot starting with **HW**. On your computer, search for and connect to the hotspot as shown in the figure by entering the password **hiwonder**.

<img src="../_static/media/chapter_1\section_1/media/image240.png"  />

2\. Open NoMachine and enter the IP address **192.168.149.1** in the search bar. Click **Configure connection to new host 192.168.149.1**.

<img src="../_static/media/chapter_1\section_1/media/image117.png" width="566" height="103" />

3\. Once opened, change the **Name** field to **Robot**, leaving the other options unchanged, and click **Add**.

<img src="../_static/media/chapter_1\section_1/media/image241.png" style="width:600px"  />

4. Then, double-click the entry named **Robot**.

<img src="../_static/media/chapter_1\section_1/media/image242.png" style="width:600px"  />

5. In the **Username** and **Password** fields, enter the following based on your controller: 

  For Jetson Nano, the default username is **hiwonder** and the password is **hiwonder**. 
  For Jetson Orin Nano, the default username is **ubuntu** and the password is **ubuntu**. 
  For Jetson Orin NX, the default username is **ubuntu** and the password is **ubuntu**. 
  After entering the credentials, check the **Remember password** box and click the **Login** button. The following example uses the Jetson Orin Nano version:

<img src="../_static/media/chapter_1\section_1/media/image119.png" style="width:600px" class="common_img" />

> [!NOTE]
>
> **If the robot is configured in STA Mode, follow the same steps as above, but make sure to replace the IP address, Username, and Password in steps 2, 3, and 4 accordingly.**

6. For configuration settings, select the appropriate checkboxes and click **OK** to proceed.

<img src="../_static/media/chapter_1\section_1/media/image120.png" style="width:600px" class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image121.png" style="width:600px" class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image122.png" style="width:600px" class="common_img" />

<img src="../_static/media/chapter_1\section_1/media/image123.png" style="width:600px" class="common_img" />

#### 1.7.2.2 Connecting via VNC

1) Search for and connect to the hotspot starting with **HW** on your computer, as shown in the diagram below. The password for the connection is **hiwonder**.

<img src="../_static/media/chapter_1\section_1/media/image243.png" style="width:400px" class="common_img" />

2. Open the VNC client and enter the IP address in AP mode: **192.168.149.1** in the address bar, then press **Enter**. If a warning about an unsecure connection appears, click **Continue**.

<img src="../_static/media/chapter_1\section_1/media/image244.png" style="width:600px" class="common_img" />

3. Wait for the login window to appear, then follow the steps in order: enter the username → enter the password → check **Remember password** → click **OK**.

Enter **pi** in the Username field, and enter **raspberrypi** in the Password field.

<img src="../_static/media/chapter_1\section_1/media/image245.png" style="width:600px" class="common_img" />

4. Once connected, the Raspberry Pi remote desktop will be displayed as shown below.

<img src="../_static/media/chapter_1\section_1/media/image246.png" style="width:600px" class="common_img" />

#### 1.7.2.3 Connecting via MobaXterm

Taking the AP mode as an example, the same steps also apply to LAN mode, but you need to replace the IP address accordingly.

1. On the main interface, click **Session** in the upper-right corner to create a new session. In the session window, enter the robot’s recorded IP address **192.168.149.1** and then click **OK**.

<img src="../_static/media/chapter_1\section_1/media/image247.png" style="width:600px" class="common_img" />

2. Select **SSH**.

<img src="../_static/media/chapter_1\section_1/media/image248.png" style="width:600px" class="common_img" />

3. Enter the fixed IP address for AP mode: **192.168.149.1**.

<img src="../_static/media/chapter_1\section_1/media/image249.png" style="width:600px" class="common_img" />

4. If a window like the one below appears, click the third option.

<img src="../_static/media/chapter_1\section_1/media/image250.png" style="width:600px" class="common_img" />

5. When prompted for the username and password, enter the credentials based on your controller version as shown below. The following example uses the Jetson Nano version:

For Jetson Nano, the default username is **hiwonder** and the password is **hiwonder**. 
For Jetson Orin Nano, the default username is **ubuntu** and the password is **ubuntu**. 
For Jetson Orin NX, the default username is **ubuntu** and the password is **ubuntu**. 
For Raspberry Pi 5, the default username is **pi** and the password is **raspberrypi**.

<img src="../_static/media/chapter_1\section_1/media/image251.png" style="width:600px" class="common_img" />

> [!NOTE]
> 
> * **The username must be entered in lowercase. Even if the account was originally created with uppercase letters, you must type it in lowercase when logging in.**
> * **After entering the username, press Enter to proceed to the password field.**
> * **When typing the password, no characters will be displayed. After finishing, press Enter again to log in.**

6. If the password is correct, you will successfully access the system, and the interface will appear as shown below.

<img src="../_static/media/chapter_1\section_1/media/image252.png" style="width:600px" class="common_img" />

### 1.7.3 LAN Mode Connection

**STA Mode (LAN Connection): The controller connects to a specified Wi-Fi network, and Internet access is available in this mode.**

> [!NOTE]
>
> * **If you want to configure the robot to use LAN Mode via your smartphone, make sure to enable the location service on your phone beforehand.**
>
> * **You cannot switch to LAN mode through the system’s default network settings, as shown in the figure below. Since the Wi-Fi has been specially configured, you will need to follow the network setup instructions provided later in this document.**
>
> * **After setting the robot to LAN mode, it will no longer broadcast a WiFi network starting with HW.**

Configuration steps:

1. Click the terminal icon <img src="../_static/media/chapter_1\section_1/media/image125.png" width="35" height="29" style="display:inline;vertical-align:middle;"/> in the system desktop to open a command-line window.

2. Enter the following command and press Enter to go to the configuration directory.

```bash
cd wifi_manager
```

3. Then enter the following command and press Enter to open the configuration file:

```bash
vim wifi_conf.py
```

4. First, set the value of **WIFI_MODE** to 2. 1 stands for Direct Connection Mode, while 2 stands for LAN Mode.

<img src="../_static/media/chapter_1\section_1/media/image128.png"  />

5. Next, modify **WIFI_STA_SSID** and **WIFI_STA_PASSWORD** to the Wi-Fi name and password of your router. For example, **hiwonder_5G** is used here. Ensure that the WiFi name does not contain spaces.

<img src="../_static/media/chapter_1\section_1/media/image129.png"  />

> [!NOTE]
>
> **It's recommended to use a 5G Wi-Fi signal for faster transmission. If you experience lag with a standard Wi-Fi connection, consider switching to a 5G network.**

6. After confirming the input is correct, press **ESC**, then type `:wq` to save and exit the file.

<img src="../_static/media/chapter_1\section_1/media/image130.png"  />

7. Restart the robot's Wi-Fi service using the following command.

```bash
sudo systemctl restart wifi.service
```

> [!NOTE]
> 
> * **After restarting the robot's Wi-Fi service, NoMachine will disconnect automatically. This happens because the device is in LAN mode, and connecting to a different Wi-Fi network causes the IP address to change.**
> * **To switch to Direct Connection Mode, simply edit the configuration file, comment out all the lines, save the changes, and restart the robot.**

8. For instructions on finding the device’s IP address, refer to [5.2.2 LAN Mode Connection (Optional)](#anther5.2.2), then follow the steps in [7.2 AP Mode Connection Steps](#anther7.2), updating the IP address with the one you’ve found.

### 1.7.4 Fixed IP Connection via USB Data Cable

> [!NOTE]
>
> **This section applies only to Jetson series controllers. It is not supported for the Raspberry Pi 5 controller.**

The robot can achieve smoother remote operation by enabling the remote NDIS-compatible device and using a fixed IP address of **192.168.55.1**. This method does not require connecting to the robot's hotspot or Wi-Fi in LAN mode. The steps are as follows:

1. For the Jetson Nano controller, connect the robot to the computer using a Micro-USB data cable.

<img src="../_static/media/chapter_1\section_1/media/image132.png" style="width:600px" class="common_img" />

2. For the Jetson Orin Nano controller, connect the robot to the computer using a Type-C data cable.

<img src="../_static/media/chapter_1\section_1/media/image264.png" style="width:600px" class="common_img" />

3. On the computer, right-click **This PC** on the desktop and select **Manage**.

<img src="../_static/media/chapter_1\section_1/media/image133.png" width="302" height="240" />

4. Click **Device Manager** in the left panel, then find the NDIS driver under **Network Adapters**. Right-click the driver and select **Update Driver**.

5. Next, follow the steps in[ 7.2 AP Mode Connection Steps](#anther7.2), and change the IP address field to **192.168.55.1**.

### 1.7.5 Changing Chassis Type

> [!NOTE]
> 
> * **After replacing the chassis with a different type, the robot’s chassis type must be updated in the software.**
> * **The system is set to a Mecanum chassis by default. If using an Ackermann or differential chassis, the chassis type should be updated in the software first.**

Steps:

1\. Double-click the system desktop icon <img src="../_static/media/chapter_1\section_1/media/image157.png" width="50"  style="display:inline;vertical-align:middle;" style="display:inline;vertical-align:middle;"/> to open the configuration tool.

2\. After the program loads, go to the **Machine** settings, where the current chassis type is `ROSOrin_Mecanum`.

<img src="../_static/media/chapter_1\section_1/media/image158.png" class="common_img" />

3\. If using an Ackermann chassis, click the drop-down menu to switch to **ROSorin_Acker**. If using a four-wheel differential chassis, click to select `ROSOrin_Differential`.

<img src="../_static/media/chapter_1\section_1/media/image159.png" class="common_img" />

4\. Click **Save**.

<img src="../_static/media/chapter_1\section_1/media/image160.png" class="common_img" />

5\. Click **Apply** and wait for the service to restart.

<img src="../_static/media/chapter_1\section_1/media/image161.png" class="common_img" />

6\. Finally, click **Quit** to close the software and complete the chassis change.

<img src="../_static/media/chapter_1\section_1/media/image162.png" class="common_img" />



## 1.8 Manual Mapping

This section demonstrates how to quickly experience manual mapping. No complex operations are required, simply tap the corresponding icon on the touchscreen to begin.

Manual mapping requires controlling the robot using either a wireless controller or a keyboard. For a single-robot environment, using a controller is recommended for convenience. In a multi-robot environment, it is recommended to use a keyboard to avoid signal interference. Since signals from wireless controllers can interfere with each other, it is recommended not to use this function when multiple robots are in the same area, to avoid misconnection or unintended control.

After mapping is completed, the generated map will be saved. You can then enable the autonomous navigation feature to test navigation using the created map. Please note that the autonomous navigation feature will always use the most recently created map. Any newly created map will overwrite the previous map, regardless of the mapping method used.

### 1.8.1 Preparation

Before starting the robot, first ensure that the remote desktop tool has been installed according to section [7. Development Environment Setup](#anther7.0), and that the connection to the robot has been established.

Next, confirm that the wireless controller’s receiver is properly connected to the robot's USB port and securely plugged in.

### 1.8.2 Operation Steps

#### 1.8.2.1 ROS1 Mapping

> [!NOTE]
> 
> * **This section is intended for the Jetson Nano controller.**
> * **This mode requires a pre-prepared enclosed space on a flat surface. If obstacles are placed, their height must exceed the Lidar’s horizontal level.**

1. Place the robot in the prepared area, ensuring the space is spacious, flat, and enclosed to allow sufficient room for movement in all directions.

2. On the software desktop, select and open the SLAM icon for manual mapping.
   
   <img src="../_static/media/chapter_1\section_1/media/image169.png" width="89" height="188" />
   
3. Multiple terminals will launch and run automatically, so it might take some time for the interface to fully load.
   
   <img src="../_static/media/chapter_1\section_1/media/image170.png" width="553" height="348" />
   
4. When the display interface appears, the system has started successfully.
   
   <img src="../_static/media/chapter_1\section_1/media/image171.png" width="531" height="288" />
   
5. You can now control the robot for mapping using the wireless controller. Function descriptions for each button are listed below:
   
> [!NOTE]
>
> **Keep the controller within a reasonable distance to avoid disconnection.**

<table>
  <tr>
    <td>Button/Joystick</td>
    <td>Operation</td>
    <td>Function</td>
  </tr>
  <tr>
    <td>START</td>
    <td>Press</td>
    <td>Exit sleep mode</td>
  </tr>
  <tr>
    <td rowspan="2">Left Joystick</td>
    <td>Push forward/backward</td>
    <td>Control robot's forward and backward movement</td>
  </tr>
  <tr>
    <td>Push left/right</td>
    <td>Control robot's left and right translation. Mecanum wheel chassis only.</td>
  </tr>
  <tr>
    <td>Right Joystick</td>
    <td>Push left</td>
    <td>Control robot's left turn</td>
  </tr>
</table>

If using a keyboard for control, connect to the remote desktop and select a terminal that allows keyboard control.

<img src="../_static/media/chapter_1\section_1/media/image172.png" width="553" height="351" />

The following table outlines the keyboard controls:

| **Button**| **Function**| **Description**|
|----------|----------|----------|
| W| Move forward| Short-press to switch to the forward state, then the robot will keep moving forward. |
| S| Move backward| Short-press to switch to the backward state, then the robot will keep moving backward. |
| A| Turn left| Long-press to interrupt the forward or backward state and rotate counterclockwise in place. |
| D| Turn right| Long-press to interrupt the forward or backward state and rotate clockwise in place. |

When pressing **W** or **S**, the robot will continuously move forward or backward. When pressing **A** or **D**, the robot will interrupt the current forward or backward action and rotate in place. Releasing **A** or **D** will stop the rotation, and the robot will remain stationary.

> [!NOTE]
>
> **For the Mecanum wheel chassis, sideways movement is not supported when using keyboard control.**

6. After movement is complete, tap the **Save** button in the lower-left corner to store the created map.

<img src="../_static/media/chapter_1\section_1/media/image173.png" style="width:600px" />

7. Once mapping is finished, the quick mapping function can be turned off by clicking the button <img src="../_static/media/chapter_1\section_1/media/image174.png"  style="display:inline;vertical-align:middle;"/> in the upper-left corner of the interface.

#### 1.8.2.2 ROS2 Mapping

> [!NOTE]
> 
> * **This section applies to the Jetson Orin Nano, Jetson Orin NX, and Raspberry Pi 5 controllers.**
> * **This mode requires a pre-prepared enclosed space on a flat surface. If obstacles are placed, their height must exceed the Lidar’s horizontal level.**

1) Place the robot in the area where mapping will take place.
2) Tap the touchscreen, select and open the SLAM icon for manual mapping.

<img src="../_static/media/chapter_1\section_1/media/image265.png"  />

3. Multiple terminals will launch and run automatically, so it might take some time for the interface to fully load.

<img src="../_static/media/chapter_1\section_1/media/image266.png" style="width:600px" />

4. When the display interface appears, the system has started successfully.

<img src="../_static/media/chapter_1\section_1/media/image171.png" style="width:600px" />

5. You can now control the robot for mapping using the wireless controller. Function descriptions for each button are listed below:

> [!NOTE]
>
> **Keep the controller within a reasonable distance to avoid disconnection.**

<table>
  <tr>
    <td>Button/Joystick</td>
    <td>Operation</td>
    <td>Function</td>
  </tr>
  <tr>
    <td>START</td>
    <td>Press</td>
    <td>Exit sleep mode</td>
  </tr>
  <tr>
    <td rowspan="2">Left Joystick</td>
    <td>Push forward/backward</td>
    <td>Control robot's forward and backward movement</td>
  </tr>
  <tr>
    <td>Push left/right</td>
    <td>Control robot's left and right translation. Mecanum wheel chassis only.</td>
  </tr>
  <tr>
    <td>Right Joystick</td>
    <td>Push left</td>
    <td>Control robot's left turn</td>
  </tr>
</table>


To use keyboard control, connect via the remote desktop and switch the terminal to keyboard control mode.

<img src="../_static/media/chapter_1\section_1/media/image267.png" style="width:600px"/>

The following table outlines the keyboard controls:

| **Button**| **Function**| **Description**|
|----------|----------|----------|
| W| Move forward| Short-press to switch to the forward state, then the robot will keep moving forward. |
| S| Move backward| Short-press to switch to the backward state, then the robot will keep moving backward. |
| A| Turn left| Long-press to interrupt the forward or backward state and rotate counterclockwise in place. |
| D| Turn right| Long-press to interrupt the forward or backward state and rotate clockwise in place. |

When pressing **W** or **S**, the robot will continuously move forward or backward. When pressing **A** or **D**, the robot will interrupt the current forward or backward action and rotate in place. Releasing **A** or **D** will stop the rotation, and the robot will remain stationary.

6. After movement is complete, press the **Save Map** button to store the created map.

<img src="../_static/media/chapter_1\section_1/media/image268.png" style="width:600px"/>

7. Once mapping is finished, the quick mapping function can be turned off by clicking the **Close** button in the command line terminal.

<img src="../_static/media/chapter_1\section_1/media/image269.png" style="width:600px"/>



## 1.9 Autonomous Mapping

> [!NOTE]
> 
> * **This section is intended for the Jetson Nano controller.**
> * **Enabling this function starts the robot’s movement immediately, so it must be placed on a flat surface beforehand.**
> * **This mode requires a pre-prepared enclosed space on a flat surface. If obstacles are placed, their height must exceed the Lidar’s horizontal level. The robot will stop automatically after completing autonomous mapping within the enclosed space.**

1) Tap the touchscreen, then select and open the SLAM Automatic icon for autonomous mapping.

<img src="../_static/media/chapter_1\section_1/media/image270.png" style="width:100px"/>

2. Opening it will automatically launch the program in the terminal. Once the interface appears, the setup is complete.

<img src="../_static/media/chapter_1\section_1/media/image271.png" style="width:600px"/>

3. Then, the robot will start moving and mapping autonomously.

4. After completing autonomous mapping within the enclosed space, it will stop and automatically save the map.



## 1.10 Autonomous Navigation

> [!NOTE]
>
> **When starting navigation, the system will load the most recently saved map, whether it was created manually or through autonomous mapping. If both methods have been used, only the latest saved map will be kept.**

### 1.10.1 ROS1 Autonomous Navigation

> [!NOTE]
>
> **This section is intended for the Jetson Nano controller.**

1\. Power on the robot, then select the Navigation icon from the Quick Navigation menu.

<img src="../_static/media/chapter_1\section_1/media/image175.png" width="93" height="213" />

2\. A terminal will launch and run automatically, and it might take some time for the interface to fully load.

<img src="../_static/media/chapter_1\section_1/media/image176.png" style="width:600px" />

3\. When the following interface appears, the setup is complete.

<img src="../_static/media/chapter_1\section_1/media/image177.png" style="width:600px" />

4\. In the software menu bar, **2D Pose Estimate** is used to set the robot’s initial position, **2D Nav Goal** is used to set a single target point, and **Publish Point** is used to set multiple target points.

<img src="../_static/media/chapter_1\section_1/media/image178.png" width="552" height="27" />

5\. Click the icon <img src="../_static/media/chapter_1\section_1/media/image179.png"  style="display:inline;vertical-align:middle;"/>, then click on the map to set a target point. Press and drag with the mouse to adjust the robot’s orientation after reaching the target. Once selected, the robot will automatically generate a path and move to the target point. 

> [!NOTE]
>
> **The icon <img src="../_static/media/chapter_1\section_1/media/image179.png" style="display:inline;vertical-align:middle;" /> changes the robot's initial position in the map, while <img src="../_static/media/chapter_1\section_1/media/image180.png" style="display:inline;vertical-align:middle;" /> sets the single-point navigation target.**

<img src="../_static/media/chapter_1\section_1/media/image181.png" style="width:600px" />

<img src="../_static/media/chapter_1\section_1/media/image182.png" style="width:600px" />

6\. To set multiple navigation points, click the icon <img src="../_static/media/chapter_1\section_1/media/image183.png"  style="display:inline;vertical-align:middle;"/> and left-click on each point on the map in sequence.

> [!NOTE]
>
> **Click Publish Point before setting each target point.**

<img src="../_static/media/chapter_1\section_1/media/image184.png" style="width:600px" />

<img src="../_static/media/chapter_1\section_1/media/image185.png" style="width:600px" />

After all target points are set, click **Start Navigation** at the lower-left corner to begin navigation. During navigation, the robot will automatically avoid obstacles. To stop navigation, click **Stop Navigation**. The robot will stop after reaching the current target point.

Clicking **Clear Goals** will remove all set target points.

<img src="../_static/media/chapter_1\section_1/media/image186.png" style="width:600px" />

### 1.10.2 ROS2 Autonomous Navigation

> [!NOTE]
>
> **This section applies to the Jetson Orin Nano, Jetson Orin NX, and Raspberry Pi 5 controllers.**

1) To begin, power on the robot, tap the touchscreen, and select the Navigation icon from the Quick Navigation menu.

<img src="../_static/media/chapter_1\section_1/media/image272.png" style="width:100px" />

2. A terminal window will open, and the program will start running and wait for the navigation interface to appear.

<img src="../_static/media/chapter_1\section_1/media/image273.png" style="width:600px" />

3. In the software menu, **2D Pose Estimate** sets the robot’s initial position. **2D Goal Pose** sets a single target point for the robot, suitable for basic navigation tasks without considering obstacle avoidance or path planning. Publish Point sets multiple target points for the robot. **Nav2 Goal** sets more complex navigation goals, such as specifying a target point, a target orientation, or a target area.

<img src="../_static/media/chapter_1\section_1/media/image274.png" style="width:600px" />

4. Click the icon <img src="../_static/media/chapter_1\section_1/media/image275.png" style="width:200px;display:inline;vertical-align:middle;"/> to adjust the robot’s initial position on the map so it matches its actual location.

5. Click the icon <img src="../_static/media/chapter_1\section_1/media/image276.png"  style="width:150px;display:inline;vertical-align:middle;" />, then click on the map to set a target point. If you click and drag, you can also define the robot’s orientation after it reaches the target.

<img src="../_static/media/chapter_1\section_1/media/image277.png" style="width:600px" />

6. Click the lower-left icon <img src="../_static/media/chapter_1\section_1/media/image278.png"  style="width:300px;display:inline;vertical-align:middle;"/> to enable multi-point navigation. Then, click the icon <img src="../_static/media/chapter_1\section_1/media/image279.png" style="width:100px;display:inline;vertical-align:middle;"/> once for each target point you want to set. Click and drag to define the robot’s orientation at each point. Repeat this process to add multiple target points.

> [!NOTE]
>
> **Click the Nav2 Goal before setting each target point.**

<img src="../_static/media/chapter_1\section_1/media/image280.png" style="width:600px" />

7. Once all target points are set, click **Start Waypoint Following** in the lower-left corner to begin. During navigation, the robot will automatically avoid obstacles along the way.

<img src="../_static/media/chapter_1\section_1/media/image281.png" style="width:600px" />



## 1.11 Hardware Introduction

This section introduces the robot’s hardware, including the electronic control system, ROS controller, LiDAR, depth camera, and other sensors.

### 1.11.1 Hardware System

Diagram of the STM32 controller ports:

<img src="../_static/media/chapter_1\section_1/media/image217.png"  style="width:600px" class="common_img" />

### 1.11.2 Electronic Control System

The robot’s electronic control system uses an STM32 controller as the low-level motion controller, connected to multiple DC geared motors with encoders. It also features a built-in IMU sensor with accelerometer and gyroscope, enabling chassis motion control and sensor data acquisition.

#### 1.11.2.1 STM32 Controller

The STM32 robot motion controller is an open-source controller specifically designed for robotics development. It is compact and elegantly designed. A single board can control various chassis types, including Mecanum chassis, Ackerman chassis, differential drive, and Tank chassis.

The controller uses the STM32F407VET6 as its main processor, based on the ARM Cortex-M4 core, running at 168 MHz, with 512 KB of on-chip Flash and 192 KB of SRAM, and it includes an FPU and DSP instructions. The system block diagram of the STM32F40x/41x series is shown below.

<img src="../_static/media/chapter_1\section_1/media/image13.png" style="width:600px" class="common_img" />

The controller features an onboard IMU with accelerometer and gyroscope sensors, supports up to four DC encoder motors, and includes two 5V power outputs with a maximum current of 5A. With abundant onboard resources and expansion interfaces, it is ideal for ROS robot development and can be paired with Jetson series ROS controllers to form a complete ROS robot system.

The front panel layout of the controller is shown in the figure below:

<img src="../_static/media/chapter_1\section_1/media/image14.png" style="width:600px" class="common_img" />

The controller provides the circuit schematic and features an SWD debug port. It supports USB serial programming for STM32 firmware updates and enables secondary development. On-board resources and peripheral example code are provided to facilitate learning and usage.

The onboard resources and peripherals of the STM32F407VET6 processor are shown in the figure below. For detailed information, please refer to the appendix documents in the folder [STM32F407XX Data Sheet](https://drive.google.com/drive/folders/1GH1QO6Cm1baFc5bdf17NPWIly7CVZbx_).

<img src="../_static/media/chapter_1\section_1/media/image15.png" style="width:600px" class="common_img" />

- **IMU Sensor**

A 6-axis IMU sensor, the MPU6050, features a 3-axis accelerometer and a 3-axis gyroscope. It connects to the controller via the I2C port. The six axes of the IMU accelerometer and gyroscope are defined as shown in the figure below.

<img src="../_static/media/chapter_1\section_1/media/image16.png"  />

<img src="../_static/media/chapter_1\section_1/media/image17.png"  />

- **Reserved Ports**

The reserved expansion ports provide connections for custom development.

<img src="../_static/media/chapter_1\section_1/media/image18.png" style="width:600px" class="common_img" />

- **SWD Debug Port**

The pins highlighted in red, 3V3, PA14, GND, PA13, serve as the SWD debug port, supporting SWD debugging.

PA14 serves as SWD_CLK and PA13 as SWD_DIO. Connect these pins to the corresponding port for debugging.

<img src="../_static/media/chapter_1\section_1/media/image19.png" style="width:600px" class="common_img" />

- **Power Supply Ports**

The STM32 controller provides multiple external power ports, distributed as shown in the figure below.

<img src="../_static/media/chapter_1\section_1/media/image20.png" style="width:600px" class="common_img" />

For more detailed information, please refer to [4. Hardware Resources\\1. STM32 Controller Files](https://drive.google.com/drive/folders/1AGNvcPICZXoWAm6O9d3FL_fcuz0Vdyp0).

#### 1.11.2.2 Power Supply

The robot uses a dedicated 11.1V 6000mAh lithium battery.

<table border="1">
  <tr>
    <td>Capacity</td>
    <td>11.1V 6000mAh</td>
    <td>Charging Voltage</td>
    <td>12.6V</td>
  </tr>
  <tr>
    <td>Discharge Current</td>
    <td>≤10A</td>
    <td>Discharge Connector</td>
    <td>SM Connector, Anti-reverse</td>
  </tr>
  <tr>
    <td>Safety</td>
    <td colspan="3">Supports overcharge protection, overcurrent protection, overdischarge protection, and short circuit protection</td>
  </tr>
</table>


The robot should be charged using the provided dedicated charger. The red light on the charger indicates that it is charging, while the green light means the battery is either fully charged or not connected. Green light when no load, red light when charging, green light when fully charged.

<img src="../_static/media/chapter_1\section_1/media/image24.jpeg"  style="width:600px"  class="common_img" />

> [!NOTE]
>
> **Do not charge the robot while it is powered on. Charging is only allowed when the robot is turned off.** 

#### 1.11.2.3 Hall Encoder DC Geared Motor

The Hall encoder is a speed-sensing module that uses a Hall-effect sensor, paired with a strong magnetic disk, and outputs pulse signals through AB-phase channels. The motor operates at 12V. The figure below shows the motor used in the robot and its pin configuration:

<img src="../_static/media/chapter_1\section_1/media/image25.png" style="width:200px" />

<img src="../_static/media/chapter_1\section_1/media/image26.png" style="width:400px"  />

For more details:

1\. The voltage between VCC and GND depends on the microcontroller supply, typically 3.3V or 5V.

2\. The frequency of the square wave output on C1 and C2 depends on the motor speed and the number of pole pairs in the magnetic disk.

3\. The voltage between M1 and M2 corresponds to the motor operating voltage.

The table below lists the specifications of the motors used in the robot:

<table border="1">
  <tr>
    <td>Rated Voltage</td>
    <td>12V</td>
    <td>Gear Ratio</td>
    <td>1:90</td>
  </tr>
  <tr>
    <td>No-load Speed</td>
    <td>110rpm</td>
    <td>Rated Speed</td>
    <td>85rpm</td>
  </tr>
  <tr>
    <td>Stall Torque</td>
    <td>15kg.cm</td>
    <td>Rated Torque</td>
    <td>2.6kg.cm</td>
  </tr>
  <tr>
    <td>Stall Current</td>
    <td>3.2A</td>
    <td>Rated Current</td>
    <td>0.36A</td>
  </tr>
  <tr>
    <td>Rated Power</td>
    <td>Approx. 8.3W</td>
    <td>Encoder Type</td>
    <td>AB Two-phase Encoder</td>
  </tr>
  <tr>
    <td>Motor Type</td>
    <td>Brushed Permanent Magnet</td>
    <td>Output Shaft</td>
    <td>6mm D-shaped Eccentric Shaft</td>
  </tr>
  <tr>
    <td>Encoder Voltage</td>
    <td>3.3~5V</td>
    <td>Magnetic Ring Lines</td>
    <td>11 Lines</td>
  </tr>
  <tr>
    <td>Interface Type</td>
    <td>PH2.0-6PIN</td>
    <td>Function</td>
    <td>Built-in Pull-up Shaping, Single-chip</td>
  </tr>
</table>



#### 1.11.2.4 PWM Servo

The Ackerman chassis of the ROSOrin robot uses a PWM servo to steer the two front wheels. The servo used is an LD-1501MG digital servo.

<img src="../_static/media/chapter_1\section_1/media/image27.png" class="common_img" />

<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <title>Servo Specifications</title>
    <style>
        table {
            border-collapse: collapse;
            width: 100%;
            margin: 20px auto;
            font-family: "Microsoft Yahei", Arial, sans-serif;
        }
        th, td {
            border: 1px solid #ddd;
            padding: 12px;
            text-align: center;
        }
        thead tr {
            background-color: #ffc107;
            font-weight: bold;
        }
        tbody tr:nth-child(even) {
            background-color: #f9f9f9;
        }
    </style>
</head>
<body>
<table>
    <tbody>
        <tr>
            <td>Operating Voltage</td>
            <td>DC 6-8.4V</td>
        </tr>
        <tr>
            <td>No-load Current</td>
            <td>100mA</td>
        </tr>
        <tr>
            <td>Stall Current</td>
            <td>2.4~3A</td>
        </tr>
        <tr>
            <td>Control Method</td>
            <td>PWM Pulse Width Control</td>
        </tr>
        <tr>
            <td>PWM Pulse Width Range</td>
            <td>500~2500μs, corresponds to 0~180°</td>
        </tr>
        <tr>
            <td>Pulse Period</td>
            <td>20ms</td>
        </tr>
        <tr>
            <td>Rotation Speed</td>
            <td>0.16sec/60° (DC 7.4V)</td>
        </tr>
        <tr>
            <td>Stall Torque</td>
            <td>13kg.cm (DC 6V), 15kg.cm (DC 6.5V), 17kg.cm (DC 7.4V)</td>
        </tr>
        <tr>
            <td>Rotation Range</td>
            <td>0~180°</td>
        </tr>
        <tr>
            <td>Servo Accuracy</td>
            <td>0.3°</td>
        </tr>
        <tr>
            <td>Wire Length</td>
            <td>30cm</td>
        </tr>
        <tr>
            <td>Gear Type</td>
            <td>Metal Gears</td>
        </tr>
        <tr>
            <td>Dimensions</td>
            <td>54.38mm × 20.04mm × 45.5mm</td>
        </tr>
        <tr>
            <td>Weight</td>
            <td>61g</td>
        </tr>
        <tr>
            <td>Applicable For</td>
            <td>Various bionic robot joints and steering joints</td>
        </tr>
    </tbody>
</table>
</body>
</html>



#### 1.11.2.5 OLED Display Module

The OLED display module features a 0.91-inch blue OLED display, offering a wide viewing angle, fast response, stable graphics, high brightness, and high resolution. It is driven by the SSD1306 chip, which controls the content displayed on the OLED screen. The module also includes LEGO-compatible mounting holes, enabling a variety of creative DIY designs.

<img src="../_static/media/chapter_1\section_1/media/image28.png" class="common_img" />

Specifications:

<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <title>OLED Display Specifications</title>
    <style>
        table {
            border-collapse: collapse;
            width: 100%;
            margin: 20px auto;
            font-family: Arial, sans-serif;
        }
        th, td {
            border: 1px solid #ccc;
            padding: 8px;
            text-align: left;
        }
        th {
            font-weight: bold;
        }
    </style>
</head>
<body>
<table>
    <tbody>
        <tr>
            <td>Operating Voltage</td>
            <td>DC 5V</td>
        </tr>
        <tr>
            <td>Communication</td>
            <td>I2C</td>
        </tr>
        <tr>
            <td>PWR Indicator</td>
            <td>Lights up after sensor power is supplied</td>
        </tr>
        <tr>
            <td>Dimensions</td>
            <td>50mm × 20mm</td>
        </tr>
        <tr>
            <td rowspan="7">0.91-inch Blue OLED Screen Specifications</td>
            <td>Resolution: 128 × 32</td>
        </tr>
        <tr>
            <td>Outline Dimensions (mm): 30.0 × 11.50 × 1.45</td>
        </tr>
        <tr>
            <td>Display Area (mm): 22.384 × 5.584</td>
        </tr>
        <tr>
            <td>Pixel Pitch (mm): 0.175 × 0.175</td>
        </tr>
        <tr>
            <td>Pixel Size (mm): 0.159 × 0.159</td>
        </tr>
        <tr>
            <td>Driver Chip: SSD1306</td>
        </tr>
        <tr>
            <td>Module Structure: COG</td>
        </tr>
        <tr>
            <td colspan="2">Modular installation, compatible with LEGO series</td>
        </tr>
    </tbody>
</table>
</body>
</html>

Port Description:

<table>
  <thead>
    <tr>
      <th>Pin</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>GND</td>
      <td>Ground</td>
    </tr>
    <tr>
      <td>5V</td>
      <td>Power Input</td>
    </tr>
    <tr>
      <td>SDA</td>
      <td>SDA Data Line</td>
    </tr>
    <tr>
      <td>SCL</td>
      <td>SCL Clock Line</td>
    </tr>
  </tbody>
</table>


#### 1.11.2.6 PS2 Wireless Controller

A USB receiver is connected to the chassis, enabling chassis movement control via a PS2 wireless controller.

<img src="../_static/media/chapter_1\section_1/media/image29.png" width="552" height="343" />

<img src="../_static/media/chapter_1\section_1/media/image30.png" width="553" height="288" />

PS2 controller pinout and connection diagram to the controller:

<img src="../_static/media/chapter_1\section_1/media/image31.png" width="330" height="282" />

<img src="../_static/media/chapter_1\section_1/media/image32.png"  width="553" height="441" />

Concept:

The USB receiver transmits differential signals through a pair of data lines, D+ and D-. When receiving the signals sent from the controller, it forwards them to the STM32, which then performs decoding.

STM32 Decoding: The STM32 reads the electrical signals on the data lines and converts them into binary data. Following the PS2 protocol, it decodes this data into specific commands.

The button mappings correspond to the table below.

<table>
  <thead>
    <tr>
      <th>Button</th>
      <th>Function</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>START</td>
      <td>Stop and reset chassis, wake up controller connection</td>
      <td>Press START</td>
    </tr>
    <tr>
      <td>Left Joystick Left</td>
      <td>Move Forward</td>
      <td>Hold</td>
    </tr>
    <tr>
      <td>Left Joystick Right</td>
      <td>Move Backward</td>
      <td>Hold</td>
    </tr>
    <tr>
      <td>Right Joystick Left</td>
      <td>Turn left, while Mecanum chassis moves left.</td>
      <td>Hold</td>
    </tr>
    <tr>
      <td>Right Joystick Right</td>
      <td>Turn right, while Mecanum chassis moves right.</td>
      <td>Hold</td>
    </tr>
  </tbody>
</table>



### 1.11.3 ROS Controller

The ROSOrin robot provides full support for ROS controllers. The operation is similar across different controllers. The Raspberry Pi 5 runs debian 12, while the Jetson series runs Ubuntu. The table below compares the specifications of Raspberry Pi 5, Jetson Nano, Jetson Orin Nano, and Jetson Orin NX controllers.

<img src="../_static/media/chapter_1\section_1/media/image282.png" style="width:600px" />

#### 1.11.3.1 Jetson Nano

<img src="../_static/media/chapter_1\section_1/media/image33.png" style="width:600px" />

The robot uses a Jetson Nano as its ROS controller, consisting of a Jetson Nano core board and an expansion board. The core board is a compact yet powerful computer capable of running mainstream deep learning frameworks, providing sufficient computing power for most AI projects.

The expansion board exposes the LED indicators and control buttons, enabling network status to be monitored through LED signals and network modes to be switched from a mobile device. It also provides GPIO and I2C ports for hardware expansion.

**The core board runs Ubuntu 18.04 with the ROS Melodic environment pre-installed, and a ROS 2 Humble environment configured within a Docker container.**

#### 1.11.3.2 Jetson Orin Nano/Jetson Orin NX

<img src="../_static/media/chapter_1\section_1/media/image283.png" style="width:600px" />

The robot uses a Jetson Orin Nano as its ROS controller, consisting of a Jetson Orin Nano core board and an expansion board. The core board is a compact yet powerful computer capable of running mainstream deep learning frameworks, providing sufficient computing power for most AI projects.

The expansion board exposes the LED indicators and control buttons, enabling network status to be monitored through LED signals and network modes to be switched from a mobile device. It also provides GPIO and I2C ports for hardware expansion.

**The core board runs Ubuntu 22.04 and comes preinstalled with the ROS 2 Humble environment.**

#### 1.11.3.3 Raspberry Pi 5

<img src="../_static/media/chapter_1\section_1/media/image284.png" style="width:600px" />

### 1.11.4 Deptrum Depth Camera

<img src="../_static/media/chapter_1\section_1/media/image285.png" style="width:600px" />

The robot is equipped with a Deptrum depth camera.

The Aurora930, part of the Deptrum® Aurora 930 series, uses 3D structured light technology to capture the three-dimensional structure of objects and spaces. Fusing RGB images from the color camera with depth data, it provides efficient and convenient 3D perception capabilities.

Its specifications are shown in the diagram and table below.

<table>
    <tr>
        <td rowspan="12" style="vertical-align: middle; text-align: center;">Module Parameters</td>
        <td>Overall Size</td>
        <td>76.5 × 20.7 × 21.8 mm</td>
    </tr>
    <tr>
        <td>Baseline</td>
        <td>40 mm</td>
    </tr>
    <tr>
        <td>Interface</td>
        <td>USB 2.0 Wafer Socket</td>
    </tr>
    <tr>
        <td>Depth Accuracy</td>
        <td>8mm@1m</td>
    </tr>
    <tr>
        <td>Depth Precision</td>
        <td>3mm@0.5m, 7mm@1m</td>
    </tr>
    <tr>
        <td>Working Distance</td>
        <td>15~300cm</td>
    </tr>
    <tr>
        <td>Operating Temperature</td>
        <td>-10°C~55°C</td>
    </tr>
    <tr>
        <td>Operating Humidity</td>
        <td>0%~95%, Non-condensing</td>
    </tr>
    <tr>
        <td>Ambient Light</td>
        <td>3~80000Lux</td>
    </tr>
    <tr>
        <td>Power Supply</td>
        <td>5V±10%, 1.5A</td>
    </tr>
    <tr>
        <td>Power Consumption</td>
        <td>Total Avg&lt;1.6W</td>
    </tr>
    <tr>
        <td>Safety</td>
        <td>Class 1 Laser Safety</td>
    </tr>
    <tr>
        <td rowspan="6" style="vertical-align: middle; text-align: center;">Image Performance</td>
        <td>Depth Data Format</td>
        <td>Raw 16</td>
    </tr>
    <tr>
        <td>Depth Resolution/Frame Rate</td>
        <td>640×400/12fps/ 74°×51°</td>
    </tr>
    <tr>
        <td>Color Data Format</td>
        <td>NV12</td>
    </tr>
    <tr>
        <td>Color Resolution/Frame Rate</td>
        <td>640×400/12fps/ 74°×51°</td>
    </tr>
    <tr>
        <td>Infrared Data Format</td>
        <td>Raw 8</td>
    </tr>
    <tr>
        <td>Infrared Resolution/Frame Rate</td>
        <td>640×400/12fps/ 74°×51°</td>
    </tr>
    <tr>
        <td rowspan="2" style="vertical-align: middle; text-align: center;">Firmware Capabilities</td>
        <td>Firmware Upgrade</td>
        <td>Supports USB Online Upgrade</td>
    </tr>
    <tr>
        <td>Warm Start Delay</td>
        <td>&lt;300ms</td>
    </tr>
    <tr>
        <td style="vertical-align: middle; text-align: center;">System Compatibility</td>
        <td>Compatibility</td>
        <td>Linux \ ARMv8 \ ROS \ Windows</td>
    </tr>
</table>




### 1.11.5 LiDAR

LiDAR is a sensor that uses laser beams to obtain precise positional information. It has a wide range of applications in robotics, including obstacle avoidance, object following, SLAM mapping, and navigation.

The robot is equipped with the MS200 LiDAR.

<img src="../_static/media/chapter_1\section_1/media/image37.png" style="width:400px" />

<table>
    <tr>
        <td>Lidar Model</td>
        <td>Aurida MS200</td>
        <td>Measurement Principle</td>
        <td>TOF Distance Measurement</td>
    </tr>
    <tr>
        <td>Recommended Scenarios</td>
        <td>Indoor and Outdoor Applications</td>
        <td>Supply Voltage</td>
        <td>5V</td>
    </tr>
    <tr>
        <td>Scanning Range</td>
        <td>360°</td>
        <td>Measurement Radius</td>
        <td>Black Object: 12m</td>
    </tr>
    <tr>
        <td>Communication Rate</td>
        <td>230400bps</td>
        <td>Sampling Frequency</td>
        <td>4500 times/s</td>
    </tr>
    <tr>
        <td>Scanning Frequency</td>
        <td>7~15Hz, Default 10Hz</td>
        <td>Angular Resolution</td>
        <td>0.4° @5Hz 0.8°@10Hz</td>
    </tr>
    <tr>
        <td>Supply Current</td>
        <td>Typical 260mA</td>
        <td>Output Interface</td>
        <td>Standard Asynchronous Serial (UART)</td>
    </tr>
    <tr>
        <td>Operating Temperature</td>
        <td>-10°~ 50°</td>
        <td rowspan="2" style="vertical-align: middle; text-align: center;">Distance Accuracy</td>
        <td rowspan="2" style="vertical-align: middle;">&le; 4mm [0.1 m~2m)<br>&le; 15mm [2m~12m]</td>
    </tr>
    <tr>
        <td>Product Size</td>
        <td>37.7 × 37.5 × 32.5 mm</td>
    </tr>
</table>




Wiring Instructions:

<img src="../_static/media/chapter_1\section_1/media/image38.png" style="width:400px" />

<table>
  <thead>
    <tr>
      <th>Pin</th>
      <th>Signal</th>
      <th>Attribute</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>1</td>
      <td>Tx</td>
      <td>Serial Data Transmit</td>
      <td>Tx (Device Transmit, 0V~3.3V)</td>
    </tr>
    <tr>
      <td>2</td>
      <td>Rx</td>
      <td>Serial Data Receive</td>
      <td>Rx (Device Receive, 0V~3.3V)</td>
    </tr>
    <tr>
      <td>3</td>
      <td>GND</td>
      <td>Power Input Negative</td>
      <td>GND (0V)</td>
    </tr>
    <tr>
      <td>4</td>
      <td>VCC</td>
      <td>Power Input Positive</td>
      <td>DC5V (4.5V~5.5V)</td>
    </tr>
  </tbody>
</table>



### 1.11.6 Microphone Array Module

This is an optional hardware module that enables voice wake-up and voice control functions on the robot. For related tutorials, please refer to the document [8. Voice Interaction Applications](https://wiki.hiwonder.com/projects/ROSOrin/en/latest/docs/8_Voice_Control_Course.html#voice-interaction-applications).

The robot is equipped with an R818 noise reduction board and a circular six-microphone array. The microphones are arranged in a planar layout to capture and process the spatial characteristics of the sound field. The system supports sound source localization, background noise suppression, interference and echo reduction, enabling 360° effective audio capture.

**R818 Noise Reduction Board:**

The R818 board is a multi-microphone voice front-end solution. It features a quad-core high-performance edge computing processor. Integrated with iFLYTEK’s voice algorithms, it utilizes the microphone array’s spatial filtering capability to locate the speaker, form a directional pickup beam, and suppress off-axis noise, achieving clearer and more accurate far-field voice capture. For human-robot interaction devices, it also includes a high-performance echo cancellation algorithm, reducing the difficulty in speech and semantic recognition. This allows developers to quickly integrate features such as multi-microphone audio capture, wake-up, noise reduction, and echo cancellation.

<img src="../_static/media/chapter_1\section_1/media/image39.png" style="width:600px" />

<table>
  <tr>
    <th>Port No.</th>
    <th>Port Name</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>1</td>
    <td>Serial Port</td>
    <td>Can be used for PC communication</td>
  </tr>
  <tr>
    <td>2</td>
    <td>Reference Signal Port</td>
    <td>Amplifier / Echo Cancellation Reference Signal</td>
  </tr>
  <tr>
    <td>3</td>
    <td>Microphone Port</td>
    <td>Connects a 6-channel microphone array</td>
  </tr>
  <tr>
    <td>4</td>
    <td>Independent Power Port</td>
    <td>Power input port</td>
  </tr>
  <tr>
    <td>5</td>
    <td>UAC Port</td>
    <td>Audio output port</td>
  </tr>
</table>


## 1.12. ROS Introduction

The ROS robot’s core system consists of two main parts. The first is chassis control, driven by the STM32 controller, responsible for robot motion control and sensor data acquisition. The second is ROS control, centered on the ROS main controller, which runs the ROS system and associated functional algorithms.

### 1.12.1 ROS Controller Hardware Connection

The standard connection setup requires a power cable and a USB serial cable. Communication between the STM32 controller and the ROS controller is established via the onboard USB serial interface. The ROS controller can be powered directly through the STM32’s power output port. The connection diagram is shown below:

<img src="../_static/media/chapter_1\section_1/media/image10.png" style="width:600px" />

### 1.12.2 ROS Serial Communication Overview

Serial communication is one of the most common input and output methods in microcontroller development and robotic systems. In this robot, data exchange between the Jetson Nano and the STM32 controller is carried out via a serial interface.

To ensure smooth communication between software tools and hardware devices, a standardized RRC protocol based on hexadecimal data transmission is adopted. This protocol serves as the foundation for communication and programming between the upper and lower controllers in this and subsequent products.

For detailed information on the communication protocol and its parsing, refer to the document RRC Communication Protocol with the Host Computer located under [4. Hardware Materials \\ 1 STM32 Controller Resources](https://drive.google.com/drive/folders/1AGNvcPICZXoWAm6O9d3FL_fcuz0Vdyp0).



## 1.13 STM32 Source Code

### 1.13.1 Introduction

The STM32 controller serves as the robot’s underlying motion control unit. It comes pre-flashed with the corresponding firmware at the factory and is ready for immediate use. For most ROS development purposes, updating the STM32 firmware is not necessary. However, users who wish to perform low-level STM32 development can update or debug the STM32 code as needed.

The STM32 controller supports ISP updates via the USB serial interface and can also be updated or debugged through the SWD interface. As the robot’s low-level driver board, the STM32 is responsible for motor PID control, encoder and IMU data acquisition, RGB control, and supports multiple control methods, including PS2 wireless controller, app via Bluetooth, and RC transmitter. It communicates with the ROS-based chassis driver node via serial interface, receiving target velocity vectors from the ROS core and sending back real-time speed calculated from odometry, IMU data, and battery voltage information. To better support these functions, the STM32 runs software built on the FreeRTOS embedded operating system.

The STM32 source code for the STM32 controller can be found in the folder [3. Source Code / STM32](https://drive.google.com/drive/folders/1p0jbaTP2Rz_vNw9noJqti7kszAbOuTUw). For a detailed explanation of the implementation principles and code analysis, refer to [4. Hardware Materials \\ 1 STM32 Controller Resources](https://drive.google.com/drive/folders/1iwbRvNJl7AVSGvrAtW_tkIPJzw2ef7kN).

### 1.13.2 Control Process

The robot supports multiple control methods, all based on adjusting the robot’s target velocity. The target velocity is processed through inverse kinematics to calculate the desired motor speeds, which serve as inputs to the motor PID controllers. After PID computation, PWM control signals are output via the STM32 timers to the motor drivers. The motor drivers then control the motor rotation, while encoders collect the real-time motor speed and feed it back to the PID controllers, forming a closed-loop speed control system. The STM32 motor control flow for the robot is shown below.

<img src="../_static/media/chapter_1\section_1/media/image187.png"  style="width:600px" />

> [!NOTE]
>
> **Motor requirements vary by robot type: Mecanum and four-wheel differential robots require four motors, while Ackerman robots require two motors, with the front wheels steered by servos.**

### 1.13.3 Program Framework

The underlying source code is developed on FreeRTOS. Unlike interrupt-driven control, FreeRTOS executes tasks in a scheduled, time-shared manner, whereas interrupts typically have a higher priority than any task. Robot task allocation is as follows:

**Robot_Task:** Responsible for robot control, kinematics processing, IMU data acquisition, and data transmission, as well as miscellaneous management tasks. This includes low-frequency operations such as battery monitoring, IMU calibration, and buzzer alerts.

### 1.13.4 Program Analysis

For a more detailed explanation of the STM32 code, refer directly to the source files, which contain comprehensive comments. The source files are located in the tutorial package under STM32 Controller Resources.

### 1.13.5 Kinematics Models

The software supports multiple robot chassis types, each with distinct characteristics.

Mecanum wheels enable full 360° omnidirectional movement, allowing the robot not only to move forward and turn but also to translate in any planar direction.

Ackerman chassis operates like a car, with front-wheel steering controlled by servos.

For detailed explanations of the kinematics models for each chassis type, refer to the documents and video tutorials under [2. Chassis Motion Control Course / 1. Kinematics Analysis](https://drive.google.com/drive/folders/1BGtaWjom-v2P8jHi4WCcyUZ3gogAmsNI) and [2. Motion Control](https://wiki.hiwonder.com/projects/ROSOrin/en/latest/docs/2_Chassis_Motion_Control_Course.html#motion-control).

### 1.13.6 Project Compilation

Once the program is written, it must be compiled into machine code to run on the embedded system. Keil5 provides a built-in compiler to convert the source code into an executable file.

The compiler generates target files based on the processor architecture and instruction set. Additionally, the compiler can optimize the code to produce a more efficient and stable executable. The compilation process is as follows:

1. Enable the option to generate a hex file:
   
   (1) In the menu bar, click **Project**, then select **Options for File ‘app.c’**.
   
   <img src="../_static/media/chapter_1\section_1/media/image189.png"  style="width:600px" />
   
   (2) Click the **Output** tab, check the three options shown in red, and click **OK**.
   
   <img src="../_static/media/chapter_1\section_1/media/image190.png" style="width:600px" />
   
   (3)  In the menu bar, click **Project** and select **Build Target** from the dropdown to compile the project.
   
   <img src="../_static/media/chapter_1\section_1/media/image191.png" style="width:600px" />
   


(4) Alternatively, click the build icon on the toolbar to start compilation.

<img src="../_static/media/chapter_1\section_1/media/image192.png" style="width:600px" />

(5) Once compilation completes successfully, the **Build Output** window at the bottom of the interface will display a success message.

<img src="../_static/media/chapter_1\section_1/media/image193.png" style="width:600px" />

> [!NOTE]
>
> **If the Build Output window shows Error(s) after compilation, double-click the error to jump to the corresponding line, fix the issue, and recompile. Warning(s) can be ignored.**

### 1.13.7 Program Download via USB

After compiling the project, the generated hex file can be flashed to the STM32 controller. The following hardware and software are required for this process.

- **Hardware and Software Requirements**

Software: FlyMcu, located in [2. Softwares \\ 4. USB Serial Download \\ FlyMcu](https://drive.google.com/drive/folders/19HBp8kvz7DTKwvJjJG5w-3jh-ggYk4pc).

<img src="../_static/media/chapter_1\section_1/media/image194.png" style="width:100px" />

**Hardware: Type-C cable and STM32 controller.**

1\. Use a Type-C cable to connect your computer to the STM32 controller.

<img src="../_static/media/chapter_1\section_1/media/image195.png" style="width:400px" />

- **Programming Steps**

The following steps illustrate the procedure, using the program **RosRobotControllerM4-ros** as an example:

**(1) Hardware Connection**

Connect the Type-C cable to the STM32 controller’s Type-C port (UART1) and the computer’s USB port: UART1

<img src="../_static/media/chapter_1\section_1/media/image196.png" style="width:400px" />

**(2)Basic Configuration**

Open the FlyMcu software, click **EnumPort** in the top menu, select the correct port, and set the baud rate (bps) to 115200:

<img src="../_static/media/chapter_1\section_1/media/image197.png"  class="common_img" />

In the software interface, click the **STM ISP** option and configure it as shown below:

<img src="../_static/media/chapter_1\section_1/media/image198.png" style="width:600px" />

At the bottom of the interface, select **Reset@DTR Low(<-3V), ISP@RTS High**.

<img src="../_static/media/chapter_1\section_1/media/image199.png" style="width:600px" />

**(3) Firmware Download**

In FlyMcu, click the button in the red frame to select the hex file to flash.

<img src="../_static/media/chapter_1\section_1/media/image200.png" style="width:600px" />

Return to the previous interface and click the **Start ISP (P)** button to begin flashing the selected hex file to the STM32 controller.

<img src="../_static/media/chapter_1\section_1/media/image201.png" style="width:600px" />

The process is flashing.

<img src="../_static/media/chapter_1\section_1/media/image202.png" style="width:600px" />

Once complete, a confirmation message will appear in the sidebar as shown in the image.

<img src="../_static/media/chapter_1\section_1/media/image203.png" style="width:600px" />

> [!NOTE]
>
> **The STM32 controller comes preloaded with the factory program for communication with the Jetson Nano controller and associated sensor modules. This program can be reflashed using the [RosRobotControllerM4.hex](https://drive.google.com/drive/folders/1p0jbaTP2Rz_vNw9noJqti7kszAbOuTUw) file.**



## 1.14 System Software Architecture

### 1.14.1 Introduction to ROS1 File Directory

> [!NOTE]
>
> **This section is intended for the Jetson Nano controller.**

Click <img src="../_static/media/chapter_1\section_1/media/image204.png" width="43" height="34" style="display:inline;vertical-align:middle;"/> to open the command terminal, enter the command **ls**, and press **Enter** to list the files in the **home** directory:

```
ls
```

<img src="../_static/media/chapter_1\section_1/media/image205.png" width="553" height="87" />

Folder descriptions:.

| Directory Name| Function|
|----------|----------|
| wifi-manager| Wi-Fi management tool|
| software| Directory for installed software|
| ros_ws| Workspace containing all feature functionalities|
| Third_party| Directory for additional ROS2 packages, such as trained YOLOv8 models|
| Music| Music files|
| Pictures| Directory storing images|
| Public| User-defined folder|
| Templates| User-defined template folder|
| Videos| Video storage|

Enter the command **cd ros_ws/** and press **Enter** to access the robot workspace directory, then type **ls** to list the files and folders inside.

```bash
cd ros_ws/
ls
```

Folder descriptions:.

| **Directory/File Name**| **Description**|
|----------|----------|
| build| Build space which stores cache files generated during compilation|
| command| Contains command files for implementing various functions, making them easy to locate.|
| devel| Contains compiled target files and executables|
| logs| Folder for storing log files|
| src| Source code folder for function packages|

Enter the command **cd ros_ws/src** and press **Enter** to access the robot workspace directory, then type **ls** to list the files and folders inside.

```bash
cd ros_ws/src
ls
```

Folder descriptions:.

| Directory Name| **Type Description**| **Function**|
|----------|----------|----------|
| app| App feature directory| Gesture control, Lidar, line tracking, etc.|
| interfaces| Communication interface files| ROS message and service communication files|
| slam| Mapping features| Map generation and saving|
| bringup| System services| Launches the app, wireless controller control, etc. |
| multi| Multi-robot features| Includes multi-robot mapping, multi-robot navigation, and more|
| third_party| Third-party package directory| Includes ROS packages for AprilTag, LiDAR, depth cameras, and more|
| calibration| Calibration parameters| IMU, linear velocity, angular velocity calibration|
| navigation| Navigation features| Waypoint publishing, RViz navigation, etc.|
| xf_mic_asr_offline| Voice control features| Voice-controlled functions|
| driver| Driver files| Kinematics, communication between Jetson Nano and STM32|
| peripherals| Peripheral setup| Different Lidar models, wireless controller, keyboard control, etc. |
| example| Example projects| Creative demos: gesture control, posture control, color tracking, etc.|
| simulations| Directory for simulation files| Including Gazebo, URDF, and others|

* **Introduction to Function File Directory**

The following explains the application files using /ros_ws/src/app as an example.

1. Navigate to the directory containing the gameplay files, where two folders can be found: launch and scripts.

```
cd /ros_ws/src/app
```

<img src="../_static/media/chapter_1\section_1/media/image208.png"  />

2. The **launch** folder contains the launch files, while **scripts** holds the source code for the feature.

<img src="../_static/media/chapter_1\section_1/media/image209.png"  />

<img src="../_static/media/chapter_1\section_1/media/image210.png"  />

### 1.14.2 Introduction to ROS2 File Directory

Open the ROS2 terminal, enter the command line, type **ls**, and press **Enter** to view the files in the **home** directory.

```
ls
```

Folder descriptions:.

| Directory Name| Function|
|----------|----------|
| wifi_manager| Wi-Fi management tool|
| software| Directory for installed software|
| ros2_ws| Workspace containing all feature functionalities|
| third_party| Directory for additional ROS2 packages, such as trained YOLOv8 models|
| Music| Music files|
| Pictures| Directory storing images|
| Public| User-defined folder|
| Templates| User-defined template folder|
| Videos| Video storage|

Enter the command **cd ros2_ws/** and press **Enter** to access the robot workspace directory, then type **ls** to list the files and folders inside.

```
cd ros2_ws/
ls
```

Folder descriptions:.

| **Directory/File Name**| **Description**|
|----------|----------|
| build| Build space which stores cache files generated during compilation|
| command| Contains command files for implementing various functions, making them easy to locate.|
| install| Contains compiled target files and executables|
| log| Folder for storing log files|
| src| Source code folder for function packages|

Enter the command **cd ros2_ws/src** and press **Enter** to access the robot workspace directory, then type **ls** to list the files and folders inside.

```
cd ros2_ws/src
ls
```

Folder descriptions:.

| **Directory Name**| **Type Description**| **Function**|
|----------|----------|----------|
| app| App feature directory| Gesture control, Lidar, line tracking, etc.|
| interfaces| Communication interface files| ROS message and service communication files|
| slam| Mapping features| Map generation and saving|
| bringup| System services| Launches app, wireless controller control, etc.|
| multi| Multi-robot features| Includes multi-robot mapping, multi-robot navigation, and more|
| large_models_examples| Directory for large model examples| Contains the large model feature examples|
| large_models| Directory for large model communication interfaces| ROS message and service communication files|
| calibration| Calibration parameters| IMU, linear velocity, angular velocity calibration|
| navigation| Navigation features| Waypoint publishing, RViz navigation, etc.|
| xf_mic_asr_offline| Voice control features| Voice-controlled functions|
| xf_mic_asr_offline_msgs| Directory for voice control communication interfaces| Contains ROS message and service communication files|
| driver| Driver files| Kinematics, communication between the main controller and the STM32 controller |
| peripherals| Peripheral setup| Different Lidar models, wireless controller control, keyboard control, etc.|
| example| Example projects| Creative demos: gesture control, posture control, color tracking, etc.|
| simulations| Directory for simulation files| Including Gazebo, URDF, and others|

* **Introduction to Function File Directory**

Taking /ros2_ws/src/app as an example, the feature files are organized as follows:

1. Navigate to the directory containing the gameplay files, where two folders can be found: launch and app.

```
cd ros2_ws/src/app
```

<img src="../_static/media/chapter_1\section_1/media/image303.png"  />

2. The **launch** folder contains the launch files, while the **app** holds the source code for the feature.

<img src="../_static/media/chapter_1\section_1/media/image304.png"  />

<img src="../_static/media/chapter_1\section_1/media/image305.png"  />



## 1.15 Image Flashing

### 1.15.1 Preparation

* **Hardware:**

For Jetson Nano and Raspberry Pi 5 versions, prepare an SD card with a capacity suitable for the image. In the example shown on the left, a 64GB SD card is used. Also, prepare a card reader and a computer, with Windows 10 recommended as the operating system.

<img src="../_static/media/chapter_1\section_1/media/image286.png" style="width:300px" class="common_img"/>

<img src="../_static/media/chapter_1\section_1/media/image287.png" style="width:300px" class="common_img"/>

For Jetson Nano and Raspberry Pi 5 versions, prepare an SD card with a capacity suitable for the image. In the example shown on the left, a 64GB SD card is used. Also prepare a card reader and a computer, with Windows 10 recommended as the operating system.

<img src="../_static/media/chapter_1\section_1/media/image288.png" class="common_img"  />

* **Software:**

A disk initialization tool called **DiskGenius.exe** and an image burning tool called **Win32DiskImager**. This section uses these two tools as examples.

> [!NOTE]
> 
> * **Before burning the image, you can use the disk initialization tool provided in [2. Software\\3. Image Burning Tools\\1. Disk Formatting Tool](https://drive.google.com/drive/folders/1L2Vvp3L7Gn9SZW7mF1CWgF8cQ0JlaG4g) to delete all extra partitions on the SD card.**
> 
> * **After the image is written, the system may prompt you with multiple partitions appearing as separate drives. Do not format them, simply cancel the prompts.**

<img src="../_static/media/chapter_1\section_1/media/image289.png" class="common_img"  />

### 1.15.2 SD Card / SSD Formatting

> [!NOTE]
>
> **If the SD card or SSD is blank, formatting is not required.**

1) Remove the SD card from Jetson Nano, Jetson Orin Nano, or Raspberry Pi 5. For SSDs, remove them from the Jetson Orin Nano or the Jetson Orin NX.

**Jetson Nano / Jetson Orin Nano**

<img src="../_static/media/chapter_1\section_1/media/image290.png" style="width:600px" class="common_img"  />

**Raspberry Pi 5**

<img src="../_static/media/chapter_1\section_1/media/image291.png" style="width:600px" class="common_img"  />

Jetson Orin NX

<img src="../_static/media/chapter_1\section_1/media/image292.png" style="width:600px" class="common_img"  />

2) In the provided materials, locate the compressed package under **2. Softwares \\ 3. Image Flashing Tools \\ 1. Disk Formatting Tool**, extract it, and open **DiskGenius.exe** to format the SD card or SSD. Make sure you select the correct drive. Choosing the wrong drive may result in formatting your computer’s hard disk.
3) After inserting the SD card or SSD into the computer, you will see an additional drive letter appear besides those of your computer’s own drives.

<img src="../_static/media/chapter_1\section_1/media/image293.png" style="width:600px" class="common_img"  />

4) Right-click the SD card drive and select **Delete All Partitions**.

<img src="../_static/media/chapter_1\section_1/media/image294.png" style="width:600px" class="common_img"  />

5) As shown in the figure below:

<img src="../_static/media/chapter_1\section_1/media/image295.png" style="width:600px" class="common_img"  />

6) Once deleted, create a new partition so the computer can recognize the SD card. Confirm any pop-up prompts.

<img src="../_static/media/chapter_1\section_1/media/image296.png" style="width:600px" class="common_img"  />

7) Then click **Save All**.

<img src="../_static/media/chapter_1\section_1/media/image297.png" style="width:600px" class="common_img"  />

8) When complete, the SD card or SSD is formatted successfully.

<img src="../_static/media/chapter_1\section_1/media/image298.png" style="width:600px" class="common_img"  />

### 1.15.3 Image Flashing

1) Open Win32DiskImager and click the icon <img src="../_static/media/chapter_1\section_1/media/image299.png" class="common_img"  style="width:70px;display:inline;vertical-align:middle;"/> to select the image file. Download and extract it beforehand, and the image shown in the examples is for reference only. In the **Device** field, select the SD card or SSD drive letter, then click **Write** to start burning the image.

<img src="../_static/media/chapter_1\section_1/media/image300.png" style="width:600px" class="common_img"  />

> [!NOTE]
>
> **The image file path must contain only English characters.**

2) If prompted with the figure below, click **Yes** to continue.

<img src="../_static/media/chapter_1\section_1/media/image301.png" style="width:600px" class="common_img"  />

3) Once you see **Write Successful**, the image has been written successfully. If an error occurs, disable firewall software, reinsert the SD card or SSD, and repeat the steps in this section.

<img src="../_static/media/chapter_1\section_1/media/image302.png" style="width:200px" class="common_img"  />

> [!NOTE]
>
> **Once the image has been successfully written, any prompt asking to format the partition can be ignored.**

4) After the burning is complete, reinsert the SD card or SSD into the controller. Power it on, wait a few moments, and the system will boot successfully.