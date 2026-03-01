from controller import Robot
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Devices
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# Line sensors
sensors = []
sensor_names = ['gs0','gs1','gs2','gs3','gs4','gs5','gs6','gs7']

for name in sensor_names:
    s = robot.getDevice(name)
    s.enable(timestep)
    sensors.append(s)

# Odometry variables
xw = 0.0
yw = 0.028
theta = 0.0

# Robot constants
R = 0.0205   # wheel radius
L = 0.052    # axle length

BASE_SPEED = 4.0

while robot.step(timestep) != -1:

    values = [s.getValue() for s in sensors]

    error_line = (values[7] - values[0]) / 1000.0
    error_line = np.clip(error_line, -2, 2)

    left_speed = BASE_SPEED - error_line
    right_speed = BASE_SPEED + error_line

    motor_left.setVelocity(left_speed)
    motor_right.setVelocity(right_speed)

    # Odometry calculations
    v_l = motor_left.getVelocity() * R
    v_r = motor_right.getVelocity() * R

    v = (v_l + v_r) / 2.0
    omega = (v_r - v_l) / L

    dt = timestep / 1000.0

    theta += omega * dt
    xw += v * np.cos(theta) * dt
    yw += v * np.sin(theta) * dt

    error_pos = np.sqrt(xw**2 + yw**2)

    print(f"x={xw:.3f}, y={yw:.3f}, theta={theta:.3f}, error={error_pos:.3f}")
