# Only python 2.7
import math
import time

from drivers import Mission, Security


DIVE_TIME = 8.0
PITCH_TARGET_DIVE = 70.0
PITCH_REG_SCALE = 5.0

REGULATION_TIME = 999.0

TARGET_DEPTH = 3.0

DEPTH_KP = 2.0
DEPTH_KI = 0.0
DEPTH_KD = 0.0


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

        mission.wait_pitch()

        integral = 0.0
        prev_error = 0.0
        security.init(REGULATION_TIME)
        while security.check(exit=True):
            imu = mission.imu

            error = imu.depth - TARGET_DEPTH

            integral = min(integral + error, 1.0)
            derivative = (error - prev_error)
            regulation = DEPTH_KP * error + DEPTH_KI * integral + DEPTH_KD * derivative

            ut = min(0.0, regulation)
            uy = -1.0
            up = -1.0

            prev_error = error

            mission.send(ut, uy, up, imu=imu)

        mission.run_for(1.0, 1.0, 0.0, 0.0)

    print('Mission done. Log saved to: %s' % mission.output_path)
