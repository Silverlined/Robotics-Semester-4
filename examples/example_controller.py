import Robot_API.Robot

my_robot = Robot_API.Robot.createRobot()
speed_left = 0
speed_right = 0

while True:
    elapsed_time = my_robot.getTimestampt()
    distance_front = my_robot.getDistance("front")
    encoder_left = my_robot.getEncoder("left")
    encoder_right = my_robot.getEncoder("right")

    if speed_left == 0 and speed_right == 0:
        speed_left = 20
        speed_right = 20
        my_robot.setMotorSpeed("left", speed_left)
        my_robot.setMotorSpeed("right", speed_right)

    if encoder_left < encoder_right:
        speed_left += 1
        my_robot.setMotorSpeed("left", speed_left)
    if encoder_right < encoder_left:
        speed_right += 1
        my_robot.setMotorSpeed("right", speed_right)
