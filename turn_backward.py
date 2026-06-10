# Only python 2.7
import time

from drivers import Mission


DIVE_TIME = 9.0
TURN_TIME = 30.0
BRAKING_TIME = 1.0


if __name__ == '__main__':
    with Mission(__file__) as mission:
        time.sleep(1.0)
        mission.run_for(DIVE_TIME, -1.0, 0.0, -1.0)
        mission.wait_pitch()
        mission.run_for(TURN_TIME, -1.0, -1.0, 0.7)
        mission.run_for(BRAKING_TIME, 1.0, 0.0, 0.0)

    print('Mission done. Log saved to: %s' % mission.output_path)
