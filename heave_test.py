# Python 2.7 only
import time

from drivers import Mission

DIVE_TIME = 8.5
LOG_TIME = 25.0


if __name__ == '__main__':
    with Mission(__file__) as mission:
        time.sleep(1.0)
        mission.run_for(DIVE_TIME, -0.5, 0.0, -1.0)
        mission.run_for(LOG_TIME, 0.0, 0.0, 0.0)

    print('Mission done. Log saved to: %s' % mission.output_path)
