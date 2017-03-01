
import rospy
from daedalus_control.srv import *
import cma
import math
import numpy as np

gait = [ 1., 1., 1., -0.5, 0, 0, 0, 0, 0, 0.78539, 0, 0, 0, 0.5, 0, 0, 0, 0]
count = 10

lb = [0]*4 + 20*[-math.pi] + [0]
ub = [2]*4 + 20*[math.pi] + [1]


rospy.init_node('optimize-3kp-W')

evaluate = rospy.ServiceProxy('evaluate', Eval)
# count = 1000.
def delegate(gait):
  # global count
  # count-=1
  print("#######")
  # return max(count, 0)
  

  T = sum(gait[0:3])

  a = np.array(gait[0:3])/T

  sgait = np.zeros(25)

  sgait[0] = T
  sgait[1] = a[0]
  sgait[2] = 0.001
  sgait[3] = a[1]
  # sgait[1:4] = a[0:3]
  sgait[4:9] = gait[3:8]
  sgait[9:14] = gait[8:13]
  sgait[14:19] = gait[8:13]
  sgait[19:24] = gait[13:18]
  sgait[24] = 1

  t = np.zeros(4)
  t[0:3] = sgait[1:4] * T
  t[3] = T - sum(t[0:3])

  for i in range(20):
    j = i + 4
    k = (i + 5) % 20 + 4

    dth = abs(sgait[j]-sgait[k])
    c = int(i/5)
    dt = t[c]

    if(dth/dt > 2*math.pi):
      return 1000

  print(sgait)
  resp = evaluate(sgait, 20)
  obj = resp.objective
  print(resp)
  return -obj


cma.fmin(delegate, gait, 0.1, eval_initial_x=True, options={ 'verb_disp': '1', 'verb_plot': '1'  })  # {'boundary_handling': 'BoundTransform ','bounds': [lb, ub] })
# resp = evaluate(gait, count)

# print resp