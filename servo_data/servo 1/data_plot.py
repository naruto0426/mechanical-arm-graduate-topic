import yaml
import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize as op
with open(r'data.yaml') as file:
  dict1 = yaml.load(file, Loader=yaml.FullLoader)
min_x =700
max_x = 2500
xs = np.array(dict1['xs'])
ys = np.array(dict1['ys'])
tp = np.logical_and(xs>=min_x,xs<=max_x)
xs_new = xs[tp]
ys_new = ys[tp]
tp_top = xs<min_x
tp_bt = xs>max_x
xs_top = xs[tp_top]
ys_top = ys[tp_top]
xs_bt = xs[tp_bt]
ys_bt = ys[tp_bt]
tp1 = ys_top>0
tp2 = ys_bt<0
tp1_not = np.logical_not(tp1)
tp2_not = np.logical_not(tp2)

xs_top_new = np.append(xs_top[tp1],xs_top[tp1_not])
ys_top_new = np.append(ys_top[tp1],ys_top[tp1_not]+180)
xs_bt_new = np.append(xs_bt[tp2],xs_bt[tp2_not])
ys_bt_new = np.append(ys_bt[tp2],ys_bt[tp2_not]+0)

xs_new1 = np.append(np.append(xs_new,xs_top_new),xs_bt_new)
ys_new1 = np.append(np.append(ys_new,ys_top_new),ys_bt_new)


def mean(data):
	return sum(data)/len(data)
all_xs = np.unique(xs_new1)

for x in all_xs:
	flags = (xs_new1==x)
	vs,count = np.unique(np.around(ys_new1[flags]*10),return_counts=True)
	ys_new1[flags] = vs[np.argmax(count)]/10


"""
for i in range(10):
	xs_new1 = np.append(xs_new1,2500)
	ys_new1 = np.append(ys_new1,175)
	xs_new1 = np.append(xs_new1,679)
	ys_new1 = np.append(ys_new1,-68)
	xs_new1 = np.append(xs_new1,555)
	ys_new1 = np.append(ys_new1,-90)
"""
val = np.polyfit(ys_new1,xs_new1,5)  #y=a*x+b
c = np.polyval(val,[0])-[1372]
val[-1] = val[-1]-mean(c)
ys_tp = np.arange(min(ys_new1),max(ys_new1),0.1)
#ys_tp = np.arange(-90,90,0.1)
pred = np.polyval(val,ys_tp)
print(np.polyval(val,[-90,0,78,175]))
#plt.scatter(xs,ys,color='blue',s=1)
plt.scatter(xs_new1-mean(c),ys_new1,color='blue',s=1)
plt.plot(pred,ys_tp,color='red')
print('[',end='')
print(*val,sep=',',end='')
print(']')
plt.show()
