from time import time,sleep
import threading
from farmware_tools import get_config_value,device
#import cv2
#from cv2 import floodFill
import numpy as np
#from numpy.fft import fft,fft2,fftshift,ifft2,ifftshift
import os

_veri_pin = 63
TOOLMOUNTED = 0
TOOLUNMOUNTED = 1
weeder=(33,554,-401)
gripper_pin = 4
gripper_down = 0
gripper_up = 1

def get_hole_coords(tray_mat,seedling_num):
    ymat = seedling_num % 12
    xmat = int(seedling_num / 12)
    pos = tray_mat[ymat,xmat]
    x = pos[0]
    y = pos[1]
    return x,y

class RepeatedTimer:
    def __init__(self, interval, function):
        self.function = function
        self.interval = interval
        self._timer=None
        self.keep=True
        self.start()
    def start(self):
        self.function()
        self._timer=threading.Timer(self.interval,self.start)
        self._timer.start()
    def cancel(self):
        self._timer.cancel()

def checktool():
  status = device.get_pin_value(_veri_pin)
  device.log(message='status: {}'.format(status), message_type='success')
  return

def pickup_gripper():
  move_absolute(weeder,(0,0,0),100)
  move_absolute(weeder,(100,0,0),100)
  move_absolute(weeder,(100,0,100),100)
  move_absolute(weeder,(100,0,260),100)
  device.write_pin(gripper_pin,gripper_up,0)
    
def move_absolute(position,offset,speed):
  as_position= device.assemble_coordinate(position[0],position[1],position[2])
  as_offset=device.assemble_coordinate(offset[0],offset[1],offset[2])
  device.move_absolute(as_position, speed=speed, offset=as_offset)


#OBTAINING CONSTANT DATA: HOMO KERNEL, CALIBRATION PARAMETERS, DESCRIPTORS
dir_path = os.path.dirname(os.path.realpath(__file__))
device.log(message='Libraries ok', message_type='success')
fw_name="Taking_photo"
#tray_num = get_config_value(fw_name,config_name="Tray",value_type=int)

device.set_pin_io_mode(0,_veri_pin)

#butt_kernel = np.load(dir_path+'/'+'kernel_butt.npy')
#descriptors = np.load(dir_path+'/'+'all_descriptors.npy')
#tvec = np.load(dir_path+'/'+'tvec.npy')
#rmatrix = np.load(dir_path+'/'+'rmatrix.npy')
#intrinsics = np.load(dir_path+'/'+'nintrinsics.npy')
matrix=np.load(dir_path+'/'+'array1.npy')
matrix2=np.load(dir_path+'/'+'array2.npy')
matrix3=np.load(dir_path+'/'+'array3.npy')
matrix4=np.load(dir_path+'/'+'array4.npy')


device.set_pin_io_mode(1,gripper_pin)
pickup_gripper()

for i in range(72):
  x,y = get_hole_coords(matrix,i)
  move_absolute((x-22,y-10,-205),(0,0,0),100)
  move_absolute((x-22,y-10,-270),(0,0,0),100)
  move_absolute((x,y,-270),(0,0,0),100)
  move_absolute((x,y,-291),(0,0,0),100)
  device.wait(500)
  device.write_pin(gripper_pin,gripper_down,0)
  device.wait(500)
  move_absolute((x,y,-215),(0,0,0),100)
  device.write_pin(gripper_pin,gripper_up,0)
  device.wait(500)

device.log(message='Process_finished', message_type='success')  

