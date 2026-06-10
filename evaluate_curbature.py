# Only python 2.7
import math
import time

from drivers import Mission, Security


DIVE_TIME = 8.0
PITCH_TARGET_DIVE = 70.0
PITCH_REG_SCALE = 5.0

TURN_TIME = 35.0

direction_input = raw_input('Direction: ').strip().lower()
if direction_input == 'f' or direction_input == 'forward':
    direction = 1.0
elif direction_input == 'b' or direction_input == 'backward':
    direction = -1.0
else:
    raise ValueError('direction must be f, forward, b, or backward')

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

        prev_yaw = None
        YAW_TRESHOLD = mission.yaw()
        security.init(TURN_TIME)
        while security.check(exit=True):
            imu = mission.imu

            ut = direction * 1.0
            uy = -1.0
            up = - direction * mission.pitch_reg(0.0, PITCH_REG_SCALE, imu.pitch)
            mission.send(ut, uy, up, imu=imu)

            yaw = mission.yaw(imu)
            if yaw:
                if prev_yaw and ((prev_yaw < YAW_TRESHOLD and yaw > YAW_TRESHOLD) or (prev_yaw > YAW_TRESHOLD and yaw < YAW_TRESHOLD)):
                    break
                prev_yaw = yaw

        mission.run_for(1.0, -direction, 0.0, 0.0)

    print('Mission done. Log saved to: %s' % mission.output_path)
