# Only python 2.7
import math
import time

from drivers import Mission, Security


DIVE_TIME = 9.0
PITCH_TARGET_DIVE = 70.0
PITCH_REG_SCALE = 5.0

TURN_TIME = 25.0
YAW_TRESHOLD = 355


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

        security.init(TURN_TIME
                      )
        prev_yaw = None
        while security.check(exit=True):
            imu = mission.imu

            ut = -1.0
            uy = -1.0
            up = mission.pitch_reg(0.0, PITCH_REG_SCALE, imu.pitch)
            mission.send(ut, uy, up, imu=imu)

            yaw = mission.yaw(imu)
            if yaw:
                if prev_yaw and prev_yaw < YAW_TRESHOLD and yaw > YAW_TRESHOLD:
                    break
                prev_yaw = yaw

        mission.run_for(1.0, 1.0, 0.0, 0.0)

    print('Mission done. Log saved to: %s' % mission.output_path)
