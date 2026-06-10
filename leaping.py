# Only python 2.7
import time

from drivers import Mission, Security


DIVE_TIME = 10.0
PITCH_TARGET_DIVE = 85.0
PITCH_REG_SCALE = 7.0


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
        
        
        mission.run_for(1.0, 1.0, 0.0, 1.0)

        security.init(6.0)
        while security.check():
            imu = mission.imu

            ut = 1.0 
            uy = 0.0
            up = -mission.pitch_reg(PITCH_TARGET_DIVE, PITCH_REG_SCALE, imu.pitch)

            mission.send(ut, uy, up, imu=imu)


        
        mission.run_for(2.0, -1.0, 0.0, 0.0)

    print('Mission done. Log saved to: %s' % mission.output_path)
