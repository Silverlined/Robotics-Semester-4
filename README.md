# Robot_API Documentation
An interface class providing access to all the necessary robot functionalities.

## Contents
1. [API Reference](#API-Reference)
2. [Getting Started](#Getting-Started)
2. [Extra Material](#Extra-Material)

## API Reference
### Methods:

    createRobot() 
        Description
        ----------
        Creates an instance of the robot class.

        Usage
        ----------
            my_robot = Robot_API.createRobot()
</t>

    getTimestamp()
        Description
        ----------
        Returns the elapsed time in seconds since initialization of the Robot.
        Fractions of a second may be present if the system clock provides them.

        Usage
        ----------
            elapsed_time = my_robot.getTimestamp()
</t>

    getDistance(sensor)
        Description
        ----------
        Returns the calculated distance for the given ultrasound sensor.

        Parameters
        ----------
        sensor : str
            The name of the ultrasound sensor as defined in the config .yaml file.

        Usage
        ----------
            distance = my_robot.getDistance("front")
</t>

    getEncoder(sensor)
        Description
        ----------
        Returns data from an encoder

        Parameters
        ----------
        sensor : str
            The name of the encoder as defined in the config .yaml file

        Usage
        ----------
            encoder_reading = my_robot.getEncoder("left")
</t>

    getAnalogPinValue(pin)
        Description
        ----------
        Returns data from an analog pin

        Parameters
        ----------
        pin : int
            The analog pin number as an integer
            A0 -> 14
            A1 -> 15
            ...
            A7 -> 21

        Usage
        ----------
            A0_pin_value = my_robot.getAnalogPinValue(14)
</t>

    setAnalogPinValue(pin, value)
        Description
        ----------
        Sets the output PWM signal of a pin and returns 'True' if successful

        Parameters
        ----------
        pin : int
            This should be a PWM capable pin
            Select from pin numbers: 3, 5, 6, 9, 10, or 11
        value : int
            The value can be a number from 0-255

        Usage
        ----------
            status = my_robot.setAnalogPinValue(3, 127)
            print(status)
</t>

    getDigitalPinValue(pin)
        Description
        ----------
        Returns state of a digital pin.

        Parameters
        ----------
        pin : int
            The digital pin number as an integer
            (Hint: Follow the pinout diagram of the microcontroller)
            D2 -> 2
            D3 -> 3
            ...
            D12 -> 12
            ...
            A5 -> 19

        Usage
        ----------
            A4_pin_value = my_robot.getDigitalPinValue(18)
</t>

    setDigitalPinValue(pin, value)
        Description
        ----------
        Sets the output state of a digital pin and returns 'True' if successful

        Parameters
        ----------
        pin : int
            This can be any pin from 0 to 19, excluding 20 (A6) and 21 (A7).
        value : int
            The value can be either 0 or 1

        Usage
        ----------
            status = my_robot.setDigitalPinValue(18, 1)
            print(status)
</t>

    setLED(value)
        Description
        ----------
        Sets the output PWM signal of the LED and returns 'True' if successful.
        The LED pin is predifined in the config .yaml file.

        Parameters
        ----------
        value : int
            The value can be a number from 0-255

        Usage
        ----------
            status = my_robot.setLED(255)
            print(status)
</t>

    setServoAngle(angle)
        Description
        ----------
        Sets the angle of the servo motor and returns 'True' if successful.
        The servo pins are predifined in the config .yaml file.

        Parameters
        ----------
        angle : int
            The value can be a number from 0-360

        Usage
        ----------
            status = my_robot.setServoAngle(180)
            print(status)
</t>

    setMotorSpeed(motor, value)
        Description
        ----------
        Sets the speed of a motor and returns 'True' if successful.
        The servo pins are predifined in the config .yaml file.

        Parameters
        ----------
        motor : str
            The name of the motor as defined in the config .yaml file
        value: int
            The value can be any number between 0 - 100
            0 - Stop
            100 - Max. Speed

        Usage
        ----------
            status = my_robot.setMotorSpeed("left", 100)
            print(status)
</t>

    stop()
        Description
        ----------
        Stops both motors.
        
        Usage
        ----------
            my_robot.stop()

## Getting Started
In this project, you will be using a robot, called Zoef, designed at TU Delft. The interface uses ROS to communicate to different parts of the robot and Telemetrix4Arduino to communicate to the microcontroller.

### Installing the Zoef Image
To install the Zoef interface you will need to:

1. **Download** the image, called zoef_RASPBERRY.zip, on your computer from the following [link](https://surfdrive.surf.nl/files/index.php/s/fXCFNj4U762kTT6).
2. **Flash** the image onto the SD card using [Etcher](https://www.balena.io/etcher/). (Note: ignore failure error after validation)
3. **Insert** the newly flashed SD card into the Raspberry Pi and **power it on**.

### Connecting to the Raspberry Pi
To connect the board to your computer, it is possible to use SSH via PuTTY or Command Prompt on Windows 10 using the IP address of the Raspberry Pi with username: **zoef** and password: **zoef_zoef**. At first, the board won’t be connected to your network so you have 3 possibilities:
- Connect to the **Zoef WiFi**: After booting up, the board will create a WiFi network called Zoef_”XXXXX” with password: zoef_zoef. When connected to that network, you can SSH using the IP address - `zoef@192.168.42.1`
> Note: When connected to the *Zoef WiFi*, you won’t have access to the internet.
- **Wired connection**: Use a network cable to connect the Raspberry Pi to your local network (via your router or directly to your computer) and SSH into the IP address assigned to your board
> Note: To find out the assigned IP address to your board, you have several options: Check the control panel of your router for connected clients, scan your network via zenmap/nmap, or use the mobile app called Fing. Alternatively, use the command ip address on your Raspberry which will require an extra HDMI monitor.
- **Wireless connection**: It is possible to configure the Raspberry Pi to connect to your WLAN via the web interface of Zoef. To do that, while being connected to the Zoef_”XXXXX” network, open your browser and type in the IP address of the board - 192.168.42.1
The interface is in Dutch, but that should not be a problem for you. 
Navigate to the “Netwerk” section and fill in your Wi-Fi credentials. The board will then restart and connect to your Wi-Fi.

> *Some of the following steps require Internet access, so make sure to connect your board to your network via Ethernet or Wi-Fi.*

### Fix common problems
- Unable to resolve host:
```
sudo nano /etc/hosts
# Append the following lines:
127.0.0.1     localhost
127.0.0.1     zoef
127.0.1.1     Zoef_XXXXXX (Substitute XXXXXX for the ID of your robot)

cat /etc/hostname (Shows the ID)
```
- Temporary failure in name resolution:
```
cd ~
wget -L https://raw.githubusercontent.com/Silverlined/Robotics-Semester-4/master/systemd_services/fix_resolv.sh
cd /etc/systemd/system/
wget -L https://raw.githubusercontent.com/Silverlined/Robotics-Semester-4/master/systemd_services/fix_resolv.service
sudo systemctl start fix_resolv.service
sudo systemctl enable fix_resolv.service 
```
### Uploading Telemetrix to the Arduino
Once you have installed the image, you will have all the files needed for the robot to work at `~/zoef_ws/src/zoef_ros_package`. In order to control the Arduino via ROS, the Telemetrix4Arduino code first needs to be uploaded to the microcontroller:
> Note: Telemetrix4Arduino is used to collect data from the microcontroller and transmit it to the Raspberry. In this way, the microcontroller can be controlled remotely.
```
cd /usr/local/src/zoef/zoef_arduino
./run.sh build_nano Telemetrix4Arduino
./run.sh upload_nano Telemetrix4Arduino
```
> *run.sh is a bash script which makes use of arduino-cli to compile and upload the Telemetrix4Arduino sketch to an Arduino Nano.*

### Configuring sensor & actuator connections
In order to control the Arduino via ROS it is also needed to specify how the robot hardware is connected to the microcontroller: 
```
cd /usr/local/src/zoef/zoef_ros_package/config
rm zoef_nano_config.yaml (Remove the old file)
wget -L https://raw.githubusercontent.com/Silverlined/Robotics-Semester-4/master/ros_workspace/config/zoef_nano_config_L9110.yaml
```
> Note: This file gives information to ROS about the pinout to different sensors and actuators. 

### Setting up ROS communication

1. Replace hw_control.launch file:
```
cd /usr/local/src/zoef/zoef_ros_package/launch
rm hw_control.launch
wget -L https://raw.githubusercontent.com/Silverlined/Robotics-Semester-4/master/ros_workspace/launch/hw_control.launch
```
> Note: This is a ROS configuration file that initiates a parameter server and starts all ROS nodes. It also tells ROS to get information from the zoef_nano_config_L9110.yaml

2. Replace ROS_telemetrix_api.py file:
```
cd /usr/local/src/zoef/zoef_ros_package/scripts
rm ROS_telemetrix_api.py
wget -L https://raw.githubusercontent.com/Silverlined/Robotics-Semester-4/master/ros_workspace/scripts/ROS_telemetrix_api.py
sudo chmod +x ROS_telemetrix_api.py (Make it executable)
```
> Note: This file establishes a connection between the different ROS nodes and Telemtrix4Arduino. It is the core file that specifies the inner workings of the system and initiates control over the microcontroller. 

3. Install missing libraries:
```
python3.8 -m pip install bitstring
```

### Controlling the Robot
Once you have set up everything, there are several ways to control and test your robot. The first thing to try is to control the LED of the Arduino through one of the predefined ROS services:

To initiate ROS we can use the launch file. Open a new terminal, execute the launch file and keep it running. 

1. Initiate ROS (from the new SSH session):
```
roscd zoef_ros_package/launch
roslaunch hw_control.launch
```

2. Turn ON the LED (from the old SSH session):
```
rosservice call /zoef/set_pin_value "{pin: '13', type: 'digital', value: 1}"
```

3. Turn OFF the LED (from the old SSH session):
```
rosservice call /zoef/set_pin_value "{pin: '13', type: 'digital', value: 0}"
```
> If this works, then you can move on to controlling the robot with Python.

4. Download Robot_API.py file:
```
mkdir /home/zoef/python_controllers
cd /home/zoef/python_controllers
wget -L https://raw.githubusercontent.com/Silverlined/Robotics-Semester-4/master/Robot_API.py
```
> Once you have the Robot_API.py file, you can import it and use it in any of your new scripts.

*e.g.*
```
#!/usr/bin/env python3.8
import Robot_API

robot = Robot_API.createRobot()

while True:    
print(robot.getEncoder("left"))
print(robot.getEncoder("right"))
robot.setMotorSpeed("left", 100)
robot.setMotorSpeed("right", 100)
...
```

## Extra Material
[Control of Mobile Robots](https://youtube.com/playlist?list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr)

<img src="res/Arduino-Nano-Pinout.png" width="720">
