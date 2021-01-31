from scipy.optimize import root,fsolve,least_squares
import numpy as np
import math
import time
import cv2
import sys
import traceback
import serial
import serial.tools.list_ports
ang_list = [[[90,-90],[2467,564]],    #1640 -> 0度 615(-90度)~2500(89.5度)
          [[80,-50.04],[500,1867]],   #1363   500(80度)~1341(0度)
          [[82,-83],[2500,500]],#12#4.41    #1549  2500(85度)~500(-86度)
          [[88,-91],[516,2500]]]   #1427  #523(90度)~2500(-87度)
ang,pwm_period = list(zip(*ang_list))
max_ang,min_ang = list(zip(*ang))
max_pwm,min_pwm = list(zip(*pwm_period))
a=92 #89
b=105
c=148
d=175
bending_height = 0 #單位:mm
pre_angle = (0,0,0)
small_r = 20
"""
i0 = 13.95 #底座上面第一節 -90~(90-22) 防止撞到
i1 = 16.65 #底座上面第二節 -90~90
i2 = -0.9 #底座上面第三節 -90~90 
i3 = -6.57 #底座 -90~90
"""
#i4
#i5#950最小1500最大
d2r = math.pi/180
def get_hand(t0,t1,t2,t3):
  ang0 = t0
  ang1 = t1
  ang2 = t2
  ang3 = t3
  alpha = ang0 * d2r
  beta = (-ang0+ang1) * d2r
  gamma = (-ang0+ang1+ang2) *d2r
  delta = ang3 * d2r
  z=(a+b*math.cos(alpha)+c*math.sin(beta)+d*math.sin(gamma)) - bending_height
  r=b*math.sin(alpha)+c*math.cos(beta)+d*math.cos(gamma)
  #print(r)
  x=r*math.cos(delta) + small_r*math.sin(delta)
  y=r*math.sin(delta) - small_r*math.cos(delta)
  return [x,y,z]
def hand_func(vars,*args, **kwargs):
  temp = get_hand(vars[0],vars[1],vars[2],args[3])
  p = [(temp[0]-args[0]),(temp[1]-args[1]),(temp[2]-args[2])]
  #print('p',p)
  return p

def solve_angle(x,y,z):
  delta_theta = math.atan2(x,-y)
  r = (x**2+y**2)**(1/2)
  theta = math.atan2(r,small_r)/d2r
  R = small_r/math.cos(theta*d2r)
  delta_theta /= d2r #-90~90 deg
  delta = delta_theta - theta
  t3 = delta
  print('ang3',t3)
  sol3_root = least_squares(fun=hand_func,loss='soft_l1',tr_solver='exact',jac='3-point',x0=pre_angle,args=(x, y, z, t3),bounds=(min_ang[0:3],max_ang[0:3]))
  return [*sol3_root.x,t3]
def deg2period(num,ang):
  return int((max_pwm[num]-min_pwm[num])/(max_ang[num]-min_ang[num])*(ang-min_ang[num])+min_pwm[num])
    
points = (241,-61,16*3)
flag = True
pwm_t3 = ''
if flag:
  ang = solve_angle(*points)
  speed = '500'
  cmd_p0 = f'#0P{deg2period(0,ang[0])}S{speed}#1P{deg2period(1,ang[1])}S{speed}#2P{deg2period(2,ang[2])}S{speed}#3P{deg2period(3,ang[3])}S{speed}#5P1550S{speed}T2000\r\n'
  print('pwm3',deg2period(3,ang[3]))
else:
  ang = solve_angle(*points)
  speed = '500'
  cmd_p0 = f'#0P{deg2period(0,ang[0])}S{speed}#1P{deg2period(1,ang[1])}S{speed}#2P{deg2period(2,ang[2])}S{speed}#3P{pwm_t3}S{speed}#5P1550S{speed}T2000\r\n'
ports = serial.tools.list_ports.comports()
serial = serial.Serial(ports[-1][0], 115200, timeout=5)
serial.write(str.encode(cmd_p0))
