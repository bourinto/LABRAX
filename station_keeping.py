# Only python 2.7
import math
import time

from drivers import DeadReckoning, Mission, Security, wrap_angle_deg

CYCLE_NB = 3

LONG_DIVE_TIME = 11.0
SHORT_DIVE_TIME = 8.0

PITCH_TARGET_DIVE = 70.0
PITCH_REG_SCALE = 5.0

WAIT_FLAT_PITCH = 10.0

CRUISE_TIME = 50.0
HEADING_TARGET = 357
HEADING_THRESHOLD = 10.0
BRAKING_TIME = 1.5

PASSIVE_ASCEND_TIME = 30.0
DEPTH_THRESHOLD = 5.0


if __name__ == '__main__':
    with Mission(__file__, include_dr=True) as mission:
        time.sleep(1.0)

        dr = DeadReckoning()
        dr.reset()
        for cycle in range(CYCLE_NB):
            dive_duration_s = LONG_DIVE_TIME if cycle == 0 else SHORT_DIVE_TIME
            print('--- Cycle %d/%d (reverse dive %.1fs) ---' % (cycle + 1, CYCLE_NB, dive_duration_s))

            security = Security()
            security.init(dive_duration_s)
            while security.check():
                imu = mission.imu
                dvl = mission.dvl

                ut = -1.0

                heading = imu.heading
                heading_error = wrap_angle_deg(heading - HEADING_TARGET) if heading else 0.0
                uy = math.tanh(heading_error * 2.0 / 15.0)
                up = mission.pitch_reg(PITCH_TARGET_DIVE, PITCH_REG_SCALE, imu.pitch)

                dr.update(imu, dvl)
                mission.send(ut, uy, up, imu=imu, dvl=dvl, x_dr=dr.x, y_dr=dr.y)

            security.init(WAIT_FLAT_PITCH) # We could have used mission.wait_pitch() but we want to keep dvl readings for dead reckoning
            while security.check():
                imu = mission.imu
                dvl = mission.dvl

                ut = 0.0
                uy = 0.0
                up = 1.0

                dr.update(imu, dvl)
                mission.send(ut, uy, up, imu=imu, dvl=dvl, x_dr=dr.x, y_dr=dr.y)

                pitch = imu.pitch
                if pitch and math.fabs(pitch) < 8:
                    break

            security.init(CRUISE_TIME)
            while security.check(exit=True):
                imu = mission.imu
                dvl = mission.dvl

                heading = imu.heading
                heading_error = wrap_angle_deg(heading - HEADING_TARGET) if heading else 0.0
                x_pos = dr.x
                y_pos = dr.y
                if heading_error is None:
                    ut = 0.0
                    uy = 0.0
                    up = 0.0
                else:
                    ut = 0.2
                    up = 0.0
                    uy = -math.tanh(heading_error * 2.0 / 15.0)

                dr.update(imu, dvl)
                mission.send(ut, uy, up, imu=imu, dvl=dvl, x_dr=x_pos, y_dr=y_pos)

                if imu.depth > DEPTH_THRESHOLD or (x_pos >= 0 and heading_error is not None and math.fabs(heading_error) < HEADING_THRESHOLD):
                    break
                

            mission.run_for(BRAKING_TIME, -1.0, 0.0, 0.0, dr=dr)

            security.init(PASSIVE_ASCEND_TIME)
            while security.check(exit=True):
                imu = mission.imu
                dvl = mission.dvl

                ut = 0.0
                uy = 0.0
                up = 0.0

                dr.update(imu, dvl)
                mission.send(ut, uy, up, imu=imu, dvl=dvl, x_dr=dr.x, y_dr=dr.y)

                if imu.depth > DEPTH_THRESHOLD:
                        break


    print('Mission done. Log saved to: %s' % mission.output_path)
