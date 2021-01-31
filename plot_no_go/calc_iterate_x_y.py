import math
from scipy.optimize import root,fsolve,least_squares
import time
import multiprocessing as mp
import yaml
import plot
a=85 #89
b=105
c=148
d=172
bending_height = 0 #單位:mm

p0 = 5.4#-3.6    #1460 -> 0度
p1 = -12.3   #1363 
p2 = 4.41#12#4.41    #1549
p3 = -6.57   #1427
small_r = 30
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
  ang0 = t0+p0
  ang1 = t1+p1
  ang2 = t2+p2
  ang3 = t3+p3
  alpha = ang0 * d2r
  beta = (ang0+ang1) * d2r
  gamma = (-ang0-ang1+ang2) *d2r
  delta = ang3 * d2r
  #print(alpha,beta,gamma,delta)
  z=(a+b*math.cos(alpha)+c*math.sin(beta)-d*math.sin(gamma)) - bending_height
  r=-b*math.sin(alpha)+c*math.cos(beta)+d*math.cos(gamma)
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
    #print('delta_theta',delta_theta,'theta',theta,'delta',delta)
    pre_angle = (0,0,0)
    t3 = delta - p3
    #print(t3)
    sol3_root = least_squares(fun=hand_func,loss='soft_l1',tr_solver='exact',jac='3-point',x0=pre_angle,args=(x, y, z, t3),bounds=((-85,-65,-85),(85,85,85)))
    return [*sol3_root.x,t3]
def get_distance(p1,p2):
  d = 0
  for x0,x1 in zip(p1,p2):
    d += (x0-x1)**2
  return d**(1/2)


def calc_data(z):
  cannot_go = []
  
  for x in range(0,412):
    for y in range(-181,401-181):
      ang = solve_angle(x,y,z)
      x_i,y_i,z_i = get_hand(*ang)
      if get_distance((x,y,z),(x_i,y_i,z_i))>1:
        cannot_go += [(x,y,z)]
    print(z,x)

  with open(f'{z}.yaml','w') as f:
    yaml.dump(cannot_go,f)

if __name__=='__main__':
  aps = []
  for z in range(16,200,8):
    ap = mp.Process(target=calc_data, args=(z,))
    ap.start()
    aps += [ap]
  for ap in aps:
    ap.join()

  print('finish')

  plot.main()