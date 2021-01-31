import math
import time
import multiprocessing as mp
import yaml
import plotly.graph_objects as go

def main():
  xs =[]
  ys =[]
  zs =[]
  for z in range(16,200,8):
    with open(f'{z}.yaml','r') as f:
      cannot_go=yaml.load(f,Loader=yaml.FullLoader)
    x,y,z = list(zip(*cannot_go))
    print(z[0],len(cannot_go))
    xs += x
    ys += y
    zs += z

  marker_data = go.Scatter3d(
      x=xs, 
      y=ys, 
      z=zs, 
      mode='markers'
  )

  fig=go.Figure(data=marker_data)
  fig.show()
if __name__=='__main__':
  main()