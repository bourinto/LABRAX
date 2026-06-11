# Only python 2.7
import math
import time

from drivers import Mission, Security


DIVE_TIME = 12
PITCH_TARGET_DIVE = 70.0
PITCH_REG_SCALE = 5.0

REGULATION_TIME = 68

TARGET_DEPTH = 3.0

# IRL parameters
NOMINAL_SPEED = -0.5
DEPTH_KP = 0.4
DEPTH_KI = 0
DEPTH_KD = 10.0

# Simulation parameters
# NOMINAL_SPEED = 0.5
#DEPTH_KP = 0.4
#DEPTH_KI = 0
#DEPTH_KD = 10.0


if __name__ == '__main__':
    with Mission(__file__) as mission:
        time.sleep(1.0)
        security = Security()

        security.init(DIVE_TIME)
        while security.check():
            imu = mission.imu

            ut = -1.0
            uy = 0.0
            up = mission.pitch_reg(PITCH_TARGET_DIVE, PITCH_REG_SCALE, imu.pitch)

            mission.send(ut, uy, up, imu=imu)

        security.init(30.0)
        while security.check(exit=True) and mission.imu.depth > TARGET_DEPTH + 0.5:
            mission.send(0.0, 0.0, 0.0)

        integral = 0.0
        prev_error = 0.0
        security.init(REGULATION_TIME)
        while security.check(exit=True):
            imu = mission.imu

            error = imu.depth - TARGET_DEPTH

            integral = integral + error
            derivative = (error - prev_error)
            regulation = DEPTH_KP * error + DEPTH_KI * integral + DEPTH_KD * derivative

            ut = -0.5 + regulation
            uy = -1.0
            up = -1.0

            prev_error = error

            mission.send(ut, uy, up, imu=imu)

        mission.run_for(1.0, 1.0, 0.0, 0.0)

    print('Mission done. Log saved to: %s' % mission.output_path)
