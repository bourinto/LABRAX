# Only python 2.7
import time

from drivers import Mission


DIVE_TIME = 7.0
CRUISE_TIME = 5.0
BRAKING_TIME = 2.0


if __name__ == '__main__':
    with Mission(__file__) as mission:
        time.sleep(1.0)
        mission.run_for(DIVE_TIME, -1.0, 0.0, -1.0)
        mission.run_for(3.0, -1.0, 0.0, 1.0)
        mission.wait_pitch()
        mission.run_for(CRUISE_TIME, 1.0, 0.0, -0.1)
        mission.run_for(BRAKING_TIME, -1.0, 0.0, 0.0)

    print('Mission done. Log saved to: %s' % mission.output_path)
