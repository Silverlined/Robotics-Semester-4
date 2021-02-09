#!/usr/bin/env python3.8
import time
import rospy
import rosservice
import signal
import sys
import math
import atexit

import message_filters
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Empty
from zoef_msgs.msg import *

from zoef_msgs.srv import *
from std_srvs.srv import *

zoef = {}


class Robot:
    """
    An interface class providing access to all the necessary robot functionalities

    Methods
    -------
    createRobot()
        Creates an instance of the robot class
    getTimestamp()
        Returns the elapsed time in seconds since initialization of the Robot
    getDistance(sensor)
        Returns the calculated distance for the given ultrasound sensor
    getEncoder(sensor)
        Returns data from an encoder
    getEncoderTicks(sensor, time_delta)
        Returns the number of ticks of an encoder for a given amount of time
    getAnalogPinValue(pin)
        Returns data from an analog pin
    setAnalogPinValue(pin, value)
        Sets the output PWM signal of a pin and returns 'True' if successful
    getDigitalPinValue(pin)
        Returns the state of a digital pin
    setDigitalPinValue(pin, value)
        Sets the output state of a digital pin and returns 'True' if successful
    setLED(value)
        Sets the output PWM signal of the LED and returns 'True' if successful
    setServoAngle(angle)
        Sets the angle of the servo motor and returns 'True' if successful
    setMotorSpeed(motor, value)
        Sets the speed of a motor and returns 'True' if successful
    stop()
        Stops both motors
    """

    def __init__(self):
        # Stop robot when exited
        atexit.register(self.stop)

        # Start timing
        self.begin_time = time.time()
        self.last_call = 0

        # Service for motor speed
        if rospy.has_param("/zoef/motor"):
            motors = rospy.get_param("/zoef/motor")
            self.motor_services = {}
            for motor in motors:
                self.motor_services[motor] = rospy.ServiceProxy(
                    "/zoef/set_" + motors[motor]["name"] + "_speed",
                    SetMotorSpeed,
                    persistent=True,
                )

        #        self.text_publisher = rospy.Publisher('display_text', String, queue_size=10)
        #        self.velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        rospy.init_node("zoef_python_api", anonymous=False)

        ## Sensors
        ## The sensors are now just using a blocking service call. This is intentionally
        ## since one first needs to learn to program in a synchronous way without events.
        ## Event based programming is already possible using the ROS topics for the
        ## same sensors. At a later stage we will expose this as well to this API and
        ## maybe even to blockly.

        # Services for distance sensors
        if rospy.has_param("/zoef/distance"):
            distance_sensors = rospy.get_param("/zoef/distance")
            self.distance_services = {}
            for sensor in distance_sensors:
                self.distance_services[sensor] = rospy.ServiceProxy(
                    "/zoef/get_distance_" + distance_sensors[sensor]["name"],
                    GetDistance,
                    persistent=True,
                )

        # Services for encoder sensors
        if rospy.has_param("/zoef/encoder"):
            encoder_sensors = rospy.get_param("/zoef/encoder")
            self.encoder_services = {}
            for sensor in encoder_sensors:
                self.encoder_services[sensor] = rospy.ServiceProxy(
                    "/zoef/get_encoder_" + encoder_sensors[sensor]["name"],
                    GetEncoder,
                    persistent=True,
                )

        self.get_pin_value_service = rospy.ServiceProxy(
            "/zoef/get_pin_value", GetPinValue, persistent=True
        )
        self.set_pin_value_service = rospy.ServiceProxy(
            "/zoef/set_pin_value", SetPinValue, persistent=True
        )
        self.set_led_value_service = rospy.ServiceProxy(
            "/zoef/set_led_value", SetLEDValue, persistent=True
        )
        self.set_servo_angle_service = rospy.ServiceProxy(
            "/zoef/set_servo_angle", SetServoAngle, persistent=True
        )

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def getTimestamp(self):
        """Returns the elapsed time in seconds since initialization of the Robot 
        
        Fractions of a second may be present if the system clock provides them
        """

        return time.time() - self.begin_time

    def getTimeSinceLastCall(self):
        last_call = self.last_call
        self.last_call = time.time()
        if last_call == 0:
            return 0
        else:
            return time.time() - last_call

    def getDistance(self, sensor):
        """Returns the calculated distance for the given ultrasound sensor

        Parameters
        ----------
        sensor : str
            The name of the ultrasound sensor as defined in the config .yaml file

        Usage
        ----------
        distance = my_robot.getDistance("front")
        """

        dist = self.distance_services[sensor]()
        return dist.data

    def getEncoder(self, sensor):
        """Returns data from an encoder

        Parameters
        ----------
        sensor : str
            The name of the encoder as defined in the config .yaml file

        Usage
        ----------
        encoder_reading = my_robot.getEncoder("left")
        """

        value = self.encoder_services[sensor]()
        return value.data

    # TODO: still needs to be tested. Unclear functionality.
    def getEncoderTicks(self, sensor, time_delta):
        """Returns the number of ticks of an encoder for a given amount of time

        Parameters
        ----------
        sensor : str
            The name of the encoder as defined in the config .yaml file
        time_delta: int
            Maybe getTimeSinceLastCall?

        Usage
        ----------
        encoder_ticks_for_1 = my_robot.getEncoderTicks("left", 1)
        """

        encoder = self.motor_services[sensor](time_delta)
        return ecnoder.ticks

    def getAnalogPinValue(self, pin):
        """Returns data from an analog pin

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
        """

        value = self.get_pin_value_service(pin, "analog")
        return value.data

    def setAnalogPinValue(self, pin, value):
        """Sets the output PWM signal of a pin and returns 'True' if successful

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
        """

        value = self.set_pin_value_service(pin, "analog", value)
        return value.status

    def getDigitalPinValue(self, pin):
        """Returns state of a digital pin

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
        """

        value = self.get_pin_value_service(pin, "digital")
        return value.data

    def setDigitalPinValue(self, pin, value):
        """Sets the output state of a digital pin and returns 'True' if successful

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
        """

        value = self.set_pin_value_service(pin, "digital", value)
        return value.status

    def setLED(self, value):
        """Sets the output PWM signal of the LED and returns 'True' if successful

        The LED pin is predifined in the config .yaml file

        Parameters
        ----------
        value : int
            The value can be a number from 0-255

        Usage
        ----------
        status = my_robot.setLED(255)
        print(status)
        """

        value = self.set_led_value_service(value)
        return value.status

    def setServoAngle(self, angle):
        """Sets the angle of the servo motor and returns 'True' if successful

        The servo pins are predifined in the config .yaml file

        Parameters
        ----------
        angle : int
            The value can be a number from 0-360

        Usage
        ----------
        status = my_robot.setServoAngle(180)
        print(status)
        """

        value = self.set_servo_angle_service(angle)
        return value.status

    # def displayText(self, text):
    #     rospy.loginfo(text)
    #     self.text_publisher.publish(text)

    def setMotorSpeed(self, motor, value):
        """Sets the speed of a motor and returns 'True' if successful

        The servo pins are predifined in the config .yaml file

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
        """

        motor = self.motor_services[motor](value)
        return motor.status

    def stop(self):
        """Stops both motors
        
        Usage
        ----------
        my_robot.stop()
        """

        self.setMotorSpeed("left", 0)
        self.setMotorSpeed("right", 0)

    def signal_handler(self, sig, frame):
        self.stop()
        sys.exit()


# We need a special function to initiate the Robot() because the main.py need to call the
# init_node() (see: https://answers.ros.org/question/266612/rospy-init_node-inside-imported-file/)
def createRobot():
    """Creates an instance of the robot class
    
    Usage
    ----------
        my_robot = Robot_API.Robot.createRobot()
    """

    global zoef
    zoef = Robot()
    return zoef
