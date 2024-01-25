from Shrink_and_expand import One_cycle_Shrink_Expand
from running_std import read_IMU
from running_std import IMU_reset
from Shrink_and_expand import All_stop
import time

if __name__ == '__main__':
    diffusion_limit = 10
    pt_time = time.time()
    while True:
        try:
            if read_IMU() and (time.time()-pt_time>diffusion_limit):
                One_cycle_Shrink_Expand()
                IMU_reset()
                pt_time = time.time()
        except KeyboardInterrupt:
            print('Stop!')
            All_stop()
            break
