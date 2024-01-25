from Shrink_and_expand import One_cycle_Shrink_Expand
from running_std import read_IMU
from running_std import IMU_reset

if __name__ == '__main__':
    while True:
        try:
            if read_IMU():
                One_cycle_Shrink_Expand()
                IMU_reset()
        except KeyboardInterrupt:
            print('Stop!')
            break
