#!/usr/bin/env python3.8

from concurrent.futures import ThreadPoolExecutor
import asyncio
import os
import os.path
import sys
import time
import math
import rospy
import signal
import aiorospy
import io
from telemetrix_aio import telemetrix_aio

# Import ROS message types
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from zoef_msgs.msg import *

# Import ROS services
from zoef_msgs.srv import *

from bitstring import BitArray
import textwrap

executor = ThreadPoolExecutor(10)


devices = rospy.get_param("/zoef/device")
analog_offset = 0
pin_map = {}
max_pwm_value = 255


def get_pin_numbers(component):
    devices = rospy.get_param("/zoef/device")
    device = devices[component["device"]]
    pins = {}
    mcu = device["mcu"]
    pins = component["pins"]

    # convert pin naming to numbers
    pin_numbers = {}
    for item in pins:
        pin_numbers[item] = pins[item]
    return pin_numbers


# Abstract Sensor class
class SensorMonitor:
    def __init__(self, board, sensor, publisher):
        self.board = board
        self.pins = get_pin_numbers(sensor)
        self.publisher = publisher
        self.max_freq = 10
        if "max_frequency" in sensor:
            self.max_freq = sensor["max_frequency"]
        self.differential = 0
        if "differential" in sensor:
            self.differential = sensor["differential"]
        self.loop = asyncio.get_event_loop()
        self.last_publish_time = -1
        self.last_publish_value = {}
        rospy.loginfo(
            "Sensor initialized on topic %s (max_freq: %d, differential: %d)",
            self.publisher.name,
            self.max_freq,
            self.differential,
        )

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        return header

    # NOTE: although there are no async functions in this
    # the function needs to be async since it is called
    # inside a callback of an awaited part of telemetrix
    async def publish_imp(self, data):
        self.publisher.publish(data)
        self.last_publish_value = data

    async def publish(self, data):
        if self.max_freq == -1:
            await self.publish_imp(data)
        else:
            now_millis = int(round(time.time() * 1000))

            # always publish the first message (TODO: and maybe messages that took too long 2x 1/freq?)
            if self.last_publish_time == -1:
                await self.publish_imp(data)
                self.last_publish_time = now_millis

            # from then on publish if needed based on max_freq
            if now_millis - self.last_publish_time >= 1000.0 / self.max_freq:
                await self.publish_imp(data)
                self.last_publish_time += (
                    1000.0 / self.max_freq
                )  # Note: this should not be set to now_millis. This is due to Nyquist.


class DistanceSensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher(
            "/zoef/distance/" + sensor["name"], Range, queue_size=1, latch=True
        )
        srv = rospy.Service(
            "/zoef/get_distance_" + sensor["name"], GetDistance, self.get_data
        )
        super().__init__(board, sensor, pub)
        self.last_publish_value = Range()

    def get_data(self, req):
        return GetDistanceResponse(self.last_publish_value.range)

    async def start(self):
        print(self.pins)
        await self.board.set_pin_mode_sonar(
            self.pins["trigger"], self.pins["echo"], self.publish_data
        )

    async def publish_data(self, data):
        # Although the initialization of this Range message
        # including some of the values could be placed in the
        # constructor for efficiency reasons. This does
        # for some reason not work though.
        range = Range()
        range.radiation_type = range.ULTRASOUND
        range.field_of_view = math.pi * 5
        range.min_range = 0.02
        range.max_range = 1.5
        range.header = self.get_header()
        range.range = data[2]
        await self.publish(range)


class EncoderSensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher(
            "/zoef/encoder/" + sensor["name"], Encoder, queue_size=1, latch=True
        )
        srv = rospy.Service(
            "/zoef/get_encoder_" + sensor["name"], GetEncoder, self.get_data
        )
        super().__init__(board, sensor, pub)
        self.ticks_per_wheel = sensor["ticks_per_wheel"]
        self.max_freq = -1
        self.last_publish_value = Encoder()

    def get_data(self, req):
        return GetEncoderResponse(self.last_publish_value.value)

    async def start(self):
        await self.board.set_pin_mode_encoder(
            self.pins["pin"], 2, self.ticks_per_wheel, self.publish_data
        )

    async def publish_data(self, data):
        encoder = Encoder()
        encoder.header = self.get_header()
        encoder.value = data[2]
        await self.publish(encoder)


class PWMMotor:
    def __init__(self, board, pins):
        self.board = board
        self.pins = pins
        self.prev_motor_speed = 0
        self.loop = asyncio.get_event_loop()
        self.initialized = False

    # Ideally one would initialize the pins in the constructor. But
    # since some mcu's have some voltage on pins when they are not
    # initialized yet icw some motor controllers that use the
    # difference between the pins to determine speed and direction
    # the motor will briefly move when initializing. This is unwanted.
    # When setting this on the mcu itself the this will be done fast
    # enough. But using telemetrix is a bit too slow fow this. We
    # therefore set the pin type on first move, and do this in a way
    # where is creates a movement in teh same direction.

    ### Support for the L9110S Motor Driver Module
    async def init_motors(self, speed):
        if not self.initialized:
            if speed > 0:
                await self.board.set_pin_mode_analog_output(self.pins["1a"])
                await self.board.set_pin_mode_analog_output(self.pins["1b"])
            if speed < 0:
                await self.board.set_pin_mode_analog_output(self.pins["1b"])
                await self.board.set_pin_mode_analog_output(self.pins["1a"])
            self.initialized = True

    async def set_speed(self, speed):
        if self.prev_motor_speed != speed:
            if speed == 0:
                await self.board.analog_write(self.pins["1a"], 0)
                await self.board.analog_write(self.pins["1b"], 0)
            elif speed > 0:
                await self.init_motors(speed)
                await self.board.analog_write(self.pins["1a"], 0)
                await self.board.analog_write(
                    self.pins["1b"],
                    int(min(speed, 100) / 100.0 * max_pwm_value),
                )
            elif speed < 0:
                await self.init_motors(speed)
                await self.board.analog_write(self.pins["1b"], 0)
                await self.board.analog_write(
                    self.pins["1a"],
                    int(min(abs(speed), 100) / 100.0 * max_pwm_value),
                )
            self.prev_motor_speed = speed


### Support for the L298N Motor Driver Module
#  async def init_motors(self, speed):
#      if not self.initialized:
#        await self.board.set_pin_mode_digital_output(self.pins["in1"])
#        await self.board.set_pin_mode_digital_output(self.pins["in2"])
#        await self.board.set_pin_mode_analog_output(self.pins["en"])
#        self.initialized = True

#  async def set_speed(self, speed):
#      if (self.prev_motor_speed != speed):
#        if (speed == 0):
#          await self.board.digital_write(self.pins["in1"], 0)
#          await self.board.digital_write(self.pins["in2"], 0)
#          await self.board.analog_write(self.pins["en"], 0)
#        elif (speed > 0):
#          await self.init_motors(speed)
#          await self.board.digital_write(self.pins["in1"], 0)
#          await self.board.digital_write(self.pins["in2"], 1)
#          await self.board.analog_write(self.pins["en"], int(min(speed, 100) / 100.0 * max_pwm_value))
#        elif (speed < 0):
#          await self.init_motors(speed)
#          await self.board.digital_write(self.pins["in1"], 1)
#          await self.board.digital_write(self.pins["in2"], 0)
#          await self.board.analog_write(self.pins["en"], int(min(abs(speed), 100) / 100.0 * max_pwm_value))
#        self.prev_motor_speed = speed


async def set_motor_speed_service(req, motor):
    await motor.set_speed(req.speed)
    return SetMotorSpeedResponse(True)


async def handle_set_led_value(req):
    led = rospy.get_param("/zoef/led")
    await board.analog_write(
        get_pin_numbers(led)["pin"],
        int(min(req.value, 100) / 100.0 * max_pwm_value),
    )
    return SetLEDValueResponse(True)


async def handle_set_servo_angle(req):
    servo = rospy.get_param("/zoef/servo")
    await board.servo_write(get_pin_numbers(servo)["pin"], req.angle)
    return SetServoAngleResponse(True)


import nest_asyncio

nest_asyncio.apply()

# TODO: This needs a full refactor. Probably needs its own class
# with a member storing all settings of the pins (analog/digital)
# and whether or not a callback needs to be called.
# It pwill prbably only need one callback function anyway, pushing
# the values into the member variable.
import nest_asyncio

nest_asyncio.apply()

pin_values = {}

# TODO: and this one probably needs to keep track of
# time as well, making sure that I can not call
# this one more often than another pin.
async def data_callback(data):
    global pin_values
    print(data)
    pin_number = data[1]
    if data[0] == 3:
        pin_number += analog_offset
    pin_values[pin_number] = data[2]


def handle_get_pin_value(req):
    global pin_values
    if not req.pin in pin_values:
        if req.type == "analog":
            print("heieer analog")
            loop.run_until_complete(
                board.set_pin_mode_analog_input(
                    req.pin - analog_offset, callback=data_callback
                )
            )
        if req.type == "digital":
            print("digitlaal")
            loop.run_until_complete(
                board.set_pin_mode_digital_input(
                    req.pin, callback=data_callback
                )
            )

    print(pin_values)
    while not req.pin in pin_values:
        time.sleep(0.00001)
    value = pin_values[req.pin]
    return GetPinValueResponse(value)


# TODO: check on existing pin configuration?
async def handle_set_pin_value(req):
    # Map pin to the pin map if it is in there, or to
    # an int if raw pin number
    if req.pin in pin_map:
        pin = pin_map[req.pin]
    else:
        pin = int(req.pin)

    if req.type == "analog":
        # This should be a PWM capable pin. Therefore we do not need to
        # account for the analog_offset. We do need to account for the
        # max pwm_value though.
        capped_value = min(req.value, max_pwm_value)
        await board.set_pin_mode_analog_output(pin)
        await board.analog_write(pin, capped_value)
    if req.type == "digital":
        await board.set_pin_mode_digital_output(pin)
        await board.digital_write(pin, req.value)
    return SetPinValueResponse(True)


# Shutdown procedure
async def shutdown(signal, loop, board):
    # Shutdown teh telemtrix board
    await board.shutdown()

    # Stop the asyncio loop
    loop.stop()

    # Shutdown all tasks
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    await asyncio.gather(*tasks, return_exceptions=True)

    # Exit
    exit(0)


# Initialize the actuators. Each actuator will become a service
# which can be called.
def actuators(loop, board, device):
    servers = []

    # TODO: support multiple leds
    if rospy.has_param("/zoef/led"):
        led = rospy.get_param("/zoef/led")
        loop.run_until_complete(
            board.set_pin_mode_analog_output(get_pin_numbers(led)["pin"])
        )
        server = aiorospy.AsyncService(
            "/zoef/set_led_value", SetLEDValue, handle_set_led_value
        )
        servers.append(loop.create_task(server.start()))

    # TODO: support multiple servo's
    if rospy.has_param("/zoef/servo"):
        servo = rospy.get_param("/zoef/servo")
        loop.run_until_complete(
            board.set_pin_mode_servo(get_pin_numbers(servo)["pin"])
        )
        server = aiorospy.AsyncService(
            "/zoef/set_servo_angle", SetServoAngle, handle_set_servo_angle
        )
        servers.append(loop.create_task(server.start()))

    if rospy.has_param("/zoef/motor"):
        motors = rospy.get_param("/zoef/motor")
        motors = {k: v for k, v in motors.items() if v["device"] == device}
        for motor in motors:
            motor_obj = PWMMotor(board, get_pin_numbers(motors[motor]))
            l = lambda req, m=motor_obj: set_motor_speed_service(req, m)
            server = aiorospy.AsyncService(
                "/zoef/set_" + motor + "_speed", SetMotorSpeed, l
            )
            servers.append(loop.create_task(server.start()))

    # Set a raw pin value
    server = aiorospy.AsyncService(
        "/zoef/set_pin_value", SetPinValue, handle_set_pin_value
    )
    servers.append(loop.create_task(server.start()))

    return servers


# Initialize all sensors based on their definition in ROS param
# server. For each sensor a topic is created which publishes
# the data.
def sensors(loop, board, device):
    tasks = []
    max_freq = -1

    # initialze distance sensors
    if rospy.has_param("/zoef/distance"):
        distance_sensors = rospy.get_param("/zoef/distance")
        distance_sensors = {
            k: v for k, v in distance_sensors.items() if v["device"] == device
        }
        for sensor in distance_sensors:
            distance_publisher = rospy.Publisher(
                "/zoef/" + sensor, Range, queue_size=1, latch=True
            )
            monitor = DistanceSensorMonitor(board, distance_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))
            if (
                "max_frequency" in distance_sensors[sensor]
                and distance_sensors[sensor]["max_frequency"] >= max_freq
            ):
                max_freq = distance_sensors[sensor]["max_frequency"]
            else:
                max_freq = 10

    # Initialize encoder sensors
    if rospy.has_param("/zoef/encoder"):
        encoder_sensors = rospy.get_param("/zoef/encoder")
        encoder_sensors = {
            k: v for k, v in encoder_sensors.items() if v["device"] == device
        }
        for sensor in encoder_sensors:
            monitor = EncoderSensorMonitor(board, encoder_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))
            # encoder sensors do not need a max_frequency. They are interrupts on
            # on the mcu side.

    # Get a raw pin value
    # TODO: this still needs to be tested. We are waiting on an implementation of ananlog_read()
    # on the telemetrix side
    rospy.Service("/zoef/get_pin_value", GetPinValue, handle_get_pin_value)
    # server = aiorospy.AsyncService('/zoef/get_pin_value', GetPinValue, handle_get_pin_value)
    # tasks.append(loop.create_task(server.start()))

    # For now, we need to set the analog scan interval to teh max_freq. When we set
    # this to 0, we do get the updates from telemetrix as fast as possible. In that
    # case the aiorospy creates a latency for the analog sensors (data will be
    # updated with a delay). This also happens when you try to implement this with
    # nest_asyncio icw rospy services.
    # Maybe there is a better solution for this, to make sure that we get the
    # data here asap.
    if max_freq <= 0:
        loop.run_until_complete(board.set_analog_scan_interval(0))
    else:
        loop.run_until_complete(
            board.set_analog_scan_interval(int(1000.0 / max_freq))
        )

    return tasks


def send_sigint():
    os.kill(os.getpid(), signal.SIGINT)


if __name__ == "__main__":
    loop = asyncio.get_event_loop()

    # Catch signals to exit properly
    # We need to do it this way instead of usgin the try/catch
    # as in the telemetrix examples
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
    for s in signals:
        loop.add_signal_handler(
            s, lambda: asyncio.ensure_future(shutdown(s, loop, board))
        )

    # Initialize the telemetrix board
    board = telemetrix_aio.TelemetrixAIO(loop=loop)

    # Initialize the ROS node as anonymous since there
    # should only be one instnace running.
    rospy.init_node("zoef_telemetrix", anonymous=False, disable_signals=False)
    rospy.on_shutdown(send_sigint)

    # Start all tasks for sensors and actuators
    device = "zoef"
    sensor_tasks = sensors(loop, board, device)
    actuator_tasks = actuators(loop, board, device)
    all_tasks = sensor_tasks + actuator_tasks
    for task in all_tasks:
        loop.run_until_complete(task)

    # Should not be: loop.run_forever() or rospy.spin()
    # Loop forever and give async some time to process
    # The sleep time should be lower than 1/max_freq
    # Since telemetrix updates at a max of 1ms, 0.00001
    # should be enough.
    while True:
        loop.run_until_complete(asyncio.sleep(0.00001))

