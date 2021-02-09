# Robot_API Documentation
An interface class providing access to all the necessary robot functionalities.

## Contents
1. [API Reference](#API-Reference)
2. [Extra Material](#Extra-Material)

## API Reference
### Methods:

    createRobot() 
        Description
        ----------
        Creates an instance of the robot class.

        Usage
        ----------
            my_robot = Robot_API.Robot.createRobot()
</t>

    getTimestamp()
        Description
        ----------
        Returns the elapsed time in seconds since initialization of the Robot.
        Fractions of a second may be present if the system clock provides them.

        Usage
        ----------
            my_robot = Robot_API.Robot.createRobot()
            elapsed_time = getTimestamp()
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

## Extra Material
<img src="res/Arduino-Nano-Pinout.png" width="720">