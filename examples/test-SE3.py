
from openravepy import *
from numpy import *

from toppso3.SE3RRT import *
from toppso3 import Utils
from toppso3 import lie

import time

import TOPP
from TOPP import TOPPpy
from TOPP import Trajectory
from pylab import *
import scipy.optimize
from mpl_toolkits.mplot3d import Axes3D

import pdb

ion()



env = Environment()
# This model was downloaded from http://nasa3d.arc.nasa.gov/models/printable
env.Load("../MESSENGER/messengerWithEnvSE3.xml")
env.SetViewer('qtcoin')

robot = env.GetBodies()[0]


phi = pi

R0 = eye(3)
q0 = quatFromRotationMatrix(R0)
omega0 = zeros(3)

q1 = array([cos(phi/2.),0,0,sin(phi/2.)])
omega1 = zeros(3)

taumax = ones(3)
vmax = ones(6)
fmax = ones(3)

tran0 = array([0,0,0])
vtran0 = zeros(3)

tran1 = array([0,0.03,0])
vtran1 = zeros(3)


# pdb.set_trace()
# ################################## BiRRT planner #################################

vertex_beg = Vertex(Config(q0,tran0, omega0, vtran0), FW)
vertex_end = Vertex(Config(q1,tran1, omega1, vtran1), BW)
biRRTinstance = RRTPlanner(vertex_beg, vertex_end, robot)

allottedtime = 600
biRRTinstance.Run(allottedtime)

Rlist = biRRTinstance.GenFinalRotationMatrixList()
TrajRotlist = biRRTinstance.GenFinalTrajList()
lietraj = lie.LieTraj(Rlist,TrajRotlist)