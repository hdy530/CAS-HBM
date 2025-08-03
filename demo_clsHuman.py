# (c) Copyright 2024, Fraunhofer IFF

# This file is part of iff-cas-modHumanBody.

# iff-cas-modHumanBody is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# iff-cas-modHumanBody is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.

# You should have received a copy of the GNU Affero General Public License
# along with iff-cas-modHumanBody.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np                                                  # proprietary license, see https://github.com/numpy/numpy/blob/main/LICENSE.txt
np.set_printoptions(suppress=True) # better matrix output in console
import clsHumanModel as clsHM
import clsMultiBodyModel as clsMBM
import clsHumanModelUtilities as clsHMU

import math                                                         # CC BY-NC 4.0 license, see https://github.com/pnavaro/math-python/blob/master/LICENCE.md 
from scipy.spatial.transform import Rotation as rot                 # BSD license, see https://github.com/scipy/scipy/blob/main/LICENSE.txt 

# base transformation
visu_rpy = np.array( [0, 0, 90] )
visu_xyz = np.array( [0, 0,  0] )

deg          = math.pi/180
T0           = np.eye( 4 )
T0[0:3, 0:3] = rot.from_euler( 'zyx', visu_rpy*deg ).as_matrix()
T0[0:3, 3]   = visu_xyz
# print(T0)

# step 1) initialize human model
human = clsHM.clsHumanModel("bodyParams.xml", 'male', 77.2, 1.6, stlver="basic")

# step 2) create an object that represents a set of predefined body postures
objPos = clsHMU.clsHMBodyPoseReader( "posture.xml" )
# load specific body posture from file into the model
q = objPos.getPostureByName( "ifa_2" )

# step 3) create an object that represents the body locations defined in IS0/TS 15066
mybodyloc = clsHMU.clsHMBodyLocationReader( "body.xml" )

# define direction of the collision
u        = np.array( [0, 1, 0] )

# get position and frame id of a body location
bpid = 15 # body location id
currBodyPart     = mybodyloc.GetBodyLocationById( bpid )
pn, pi, frameid  = human.GetBodyPartPosition( currBodyPart, q )
print(frameid)

# step 4) plot the situation (red line highlight the collision direction, back spot highlights the body location)
human.PlotModel( q, u, pi, frameid, T0=T0 )

# step 4) get effective mass
# meff ~ effective mass for the body part selected
# Meff ~ effective mass for the body part selected and the body 
#        parts that lie in the same direction of the collision 
#        vector (relevant if the arm is placed against other body)
#        parts)
meff, Meff = human.EffectiveMass(q, u, pi, frameid, T0=T0)

print(meff)
print(Meff)
