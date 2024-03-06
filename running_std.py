from __future__ import print_function
import qwiic_icm20948
import time
import sys
import numpy

episode_num = 150
std_episode_num = 150
reading_array = numpy.array([])
std_array = numpy.array([])
IMU = qwiic_icm20948.QwiicIcm20948()

if IMU.connected == False:
    print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
        file=sys.stderr)
    print('Error!')

IMU.begin()
reading_index = 0
std_index = 0
std_thre = 0.1

data_gathered = list()

def read_IMU():
    global IMU
    global episode_num
    global std_episode_num
    global reading_array
    global std_array
    global reading_index
    global std_index
    global std_thre
    
    global data_gathered
    
    
    if IMU.dataReady():
        #print(reading_array.shape)
        IMU.getAgmt()
        new_data = numpy.array([[IMU.axRaw,IMU.ayRaw,IMU.azRaw]])/16.384*0.001
        if reading_array.shape[0] ==0:
            reading_array = new_data
            reading_index = reading_index+1
            return False
        if reading_array.shape[0]<episode_num:
            reading_array = numpy.concatenate((reading_array,new_data))
            reading_index = reading_index+1
            return False
        else:
            # print('std calculated')
            # print(std_array.shape)
            reading_array[(reading_index % episode_num),:] = new_data
            reading_index = reading_index+1
           
            std = numpy.std(numpy.linalg.norm(reading_array,axis = -1))
            print('std_value')
            print(std)
            data_gathered.append(std)
            if std_array.shape[0]<std_episode_num:
                std_array = numpy.append(std_array,(std<std_thre))
                std_index = std_index+1
                return False
            else:
                std_array[std_index%std_episode_num] = (std<std_thre)
                std_index=std_index+1
                return numpy.sum(std_array)==std_episode_num


    else:
        print('IMU Data Error')

def IMU_reset():
    global reading_array
    global std_array
    global reading_index
    global std_index
    reading_index = 0
    std_index = 0
    reading_array = numpy.array([])
    std_array = numpy.array([])



if __name__ == '__main__':
    IMU_debug = True
    new_data = list()
    while True:
        try:
            if IMU_debug:
                new_data.append(read_IMU())
            else:
                read_IMU()
        #print(read_IMU())
        except KeyboardInterrupt:
            print('Interrupted!')
            if IMU_debug:
                saved_data = numpy.array(new_data)
                numpy.save('IMU_online1.npy',saved_data)
                saved_std = numpy.array(data_gathered)
                numpy.save('IMU_std_realtime.npy',saved_std)
                print('Data saved!')
            break
	
