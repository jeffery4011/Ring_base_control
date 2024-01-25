from Shrink_and_expand import One_cycle_Shrink_Expand
from running_std import read_IMU
from running_std improt IMU_reset

if __name__ == '__main__':
    while True:
        if read_IMU():
            One_cycle_Shrink_Expand()
            IMU_reset()