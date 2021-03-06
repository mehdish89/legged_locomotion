
import rospy
from daedalus_control.srv import *
import cma
import math
import numpy as np

gait = [ 1., 1., 1., 1., -0.5, 0, 0, 0, 0, -0.5, 0.78539, 0, 0, 0, 0.5, 0.78539, 0, 0, 0, 0.5, 0, 0, 0, 0, 0.5 ]
count = 10

lb = [0]*4 + 20*[-math.pi] + [0]
ub = [2]*4 + 20*[math.pi] + [1]


rospy.init_node('optimize-4kp-A')

evaluate = rospy.ServiceProxy('evaluate', Eval)
# count = 1000.
def delegate(gait):
  # global count
  # count-=1
  print("#######")
  # return max(count, 0)
  

  T = sum(gait[0:4])

  a = np.array(gait[0:4])/T

  sgait = np.array(gait)

  sgait[0] = T
  sgait[1:4] = a[0:3]

  print(sgait)
  resp = evaluate(sgait, 20)
  obj = resp.objective
  print(resp)
  return -obj


cma.fmin(delegate, gait, 0.2, eval_initial_x=True, options={ 'verb_disp': '1', 'verb_plot': '1'  })  # {'boundary_handling': 'BoundTransform ','bounds': [lb, ub] })
# resp = evaluate(gait, count)

# print resp