"""PID torque controller for the single-link Webots arm."""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

motor = robot.getDevice("arm_motor")
sensor = robot.getDevice("arm_sensor")
sensor.enable(timestep)

# Torque control lets the Physics mass and joint damping influence the response.
motor.setPosition(float("inf"))
motor.setVelocity(0.0)

KP = 4.0
KI = 0.15
KD = 0.12
TARGET_ANGLE = 0.9  # rad
TORQUE_LIMIT = 8.0  # N m

integral = 0.0
previous_error = 0.0

while robot.step(timestep) != -1:
    angle = sensor.getValue()
    error = TARGET_ANGLE - angle
    integral = max(-2.0, min(2.0, integral + error * dt))
    derivative = (error - previous_error) / dt
    previous_error = error

    torque = KP * error + KI * integral + KD * derivative
    torque = max(-TORQUE_LIMIT, min(TORQUE_LIMIT, torque))
    motor.setTorque(torque)

    if int(robot.getTime() * 10) % 10 == 0:
        print(
            f"time={robot.getTime():5.2f} s  angle={angle: .3f} rad  "
            f"torque={torque: .3f} N m"
        )
