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

from dotmap import DotMap #using dictionary with .  dot-operator # MIT license, see https://github.com/drgrib/dotmap/blob/master/LICENSE.txt
from lxml import etree    #reading xml-file                      # BSD license, see https://github.com/lxml/lxml/blob/master/LICENSES.txt 
import numpy as np                                               # proprietary license, see https://github.com/numpy/numpy/blob/main/LICENSE.txt
from pathlib import Path                                         # Apache 2.0 license, see https://github.com/chigopher/pathlib/blob/master/LICENSE 
import math                                                      # CC BY-NC 4.0 license, see https://github.com/pnavaro/math-python/blob/master/LICENCE.md 


class clsXmlReader():
    xmlFile  = None
    treeObj  = None
    currNode = None

    def __init__(self, file):
        self.xmlFile = Path( file )
        if not self.xmlFile.is_file():
            # throw exception
            raise NameError('xml file is not available')
        else:
            filePath     = self.xmlFile.resolve().as_posix()
            self.treeObj = etree.parse( filePath ) # __init__()
    
    def find( self, path ):
        # get current posture
        return self.treeObj.find( path ) # find()

    def findall( self, path ):
        # get current posture
        return self.treeObj.findall( path ) # findall()

    def setNode( self, path ):
        self.currNode = self.find( path )

        if( self.currNode is None ):
            return False
        else:
            return True # setNode()

    def resetNode( self ):
        self.currNode = None # resetNode()

    def setToParent( self ):
        self.currNode = self.currNode.getparent() # !!! what if no parent is available?
    
    def getAttribVal( self, name, valType="string", optval=None, path=None ):
        if( self.currNode is None ):
            raise NameError('node was not set')

        if( not path is None ):
            subNode = self.currNode.find( path )
            if( not ( subNode is None ) ):
                mynode = subNode
            else:
                val = optval
        else:
            mynode = self.currNode

        if( name in mynode.attrib ):
            val = self.str2any( mynode.get( name ), valType )
        else:
            val = optval

        return val # getAttribVal()

    def countNodes( self, nodeName ):
        return len( self.treeObj.findall( nodeName ) )
    
    def getNodeVal( self, path, valType="string", optval=None ):
        if( self.currNode is None ):
            raise NameError('node was not set')

        subNode = self.currNode.find( path )

        if( not ( subNode is None ) ):
            val = self.str2any( subNode.text, valType )
        else:
            val = optval

        return val # getNodeVal()

    def str2any( self, strVal, valType ):
        if( valType == "np"):
            val = self.str2np( strVal )
        elif( valType == "float" ):
            val = float( strVal )
        elif( valType == "int" ):
            val = int( strVal )
        else:
            val = strVal

        return val # str2any()
    
    def str2np(self, str):
        # convert str to numpy array
        if (str == 'NaN') | (str == ''):
            # str is NaN or empty
            vec = np.nan
        elif str[-1] == 'i':
            # complex number - numpy uses j instead of i
            str = str.replace('i', 'j')  
            vec = np.array([ complex(v) for v in str.split() ])
        else:
            # regular float array
            vec = np.array([ float(v) for v in str.split() ])

        return vec # str2np()

class clsHMBodyPoseReader(clsXmlReader):
    deg     = np.pi/180

    def __init__(self, xmlFile):
        # read xml file
        clsXmlReader.__init__( self, xmlFile )
        
        return # __init__

    # def getJointangleAll(self):
    #     # return all jointangles
    #     return self.q

    # def getJointangleIdx(self,idx):
    #     #return jointangle at index idx
    #     if idx in range(0,len(self.q)):
    #         return self.q[idx]
       
    def getPostureById(self, postureid):
        # get current posture
        path    = f"//posture[@id='{ postureid }']"    
        return self.readPostureFromXML( path ) # getPostureById()
    
    def getPostureByName(self, name):
        # get current posture
        path    = f"//posture[@posturename='{ name }']"        
        return self.readPostureFromXML( path ) # getPostureByName()

    def readPostureFromXML( self, path ):
        #parse etree
        q = []

        posture = self.find( path )

        for nodeJointAngle in posture:
            # read all jointangle q from current posture
            unit = nodeJointAngle.get("unit")
            qi   = float( nodeJointAngle.text )

            if( unit == "degree" ):
                # convert to radians
                q.append( qi*self.deg )
            else:
                q.append( qi )
   
        return q # getPosture()

class clsHMBodyLocationReader:
    xmlObj = None

    def __init__(self, xmlFile):
        # read xml file
        self.xmlObj = clsXmlReader( xmlFile )
        
        return # __init__

    def getBodyLocationByName(self, name):
        #create Dotmap of child (specific body area)
        path    = f"//bodylocation[@name='{name}']"
        return self.getBodyLocation(path)

    def GetBodyLocationById(self, id):
        #create Dotmap of child (specific body area)
        if not isinstance( id, list ):
            idlist = [ id ]
        else:
            idlist = id

        bodyparts = []
        for currid in idlist:
            path = f"//bodylocation[@id='{currid}']"
            bodyparts.append( self.getBodyLocation(path) )
        
        if not isinstance( id, list ):
            return bodyparts[0]
        else:
            return bodyparts

    def getBodyLocation(self, path):
        bodyregion = DotMap()
        bodyloc    = DotMap()
        xyz        = DotMap()
        limits     = DotMap()

        #create Dotmap of child (specific body area)
        if not self.xmlObj.setNode(path):
            return None

        bodyloc.id   = self.xmlObj.getAttribVal( "id", "int" )
        bodyloc.name = self.xmlObj.getAttribVal( "name", "string", "unknown")
        
        # poi
        xyz.val     = self.xmlObj.getAttribVal( "xyz", "np")
        xyz.base    = self.xmlObj.getAttribVal( "xyzbase", "string", "unkown")
        xyz.frameid = self.xmlObj.getAttribVal( "frameid", "int" ) - 1

        # limits
        limits.qs.pressure = self.xmlObj.getNodeVal( "limits/quasistatic/pressure", "float", 0)
        limits.qs.force    = self.xmlObj.getNodeVal( "limits/quasistatic/force", "float", 0)
        limits.tr.pressure = self.xmlObj.getNodeVal( "limits/transient/pressure", "float", 0)
        limits.tr.force    = self.xmlObj.getNodeVal( "limits/transient/force", "float", 0)

        #create Dotmap of parent (bodyregion)
        path = path + "/.."
        if not self.xmlObj.setNode(path):
            return None

        bodyregion.id      = self.xmlObj.getAttribVal( "id", "int" )
        bodyregion.name    = self.xmlObj.getAttribVal( "name", "string" )

        # add poi and body region information to body location
        bodyloc.bodyregion = bodyregion
        bodyloc.xyz        = xyz
        bodyloc.limits     = limits

        return bodyloc # getBodyLocation()


class clsHMJointAndFrameMapper(clsXmlReader):
    targetQ = None

    def __init__(self, xmlFile):
        # read xml file
        clsXmlReader.__init__( self, xmlFile )

        # init joints of target model
        self.targetQ = self.__initJoints()
        
        return # __init__
       
    def mapJointAnglesForward(self, dictJoints):
        if not type( dictJoints ) is dict:
            raise NameError('input parameter is not of type "dict"')

        for currKey in dictJoints.keys():
            # get joint position
            jointPos          = float( dictJoints[ currKey ] )
            # get mapping info
            jointId, jointDir, jointOff = self.__getJointMapInfoForward( currKey )
            # set joint pos of target model
            idx               = jointId - 1
            self.targetQ[idx] = jointDir*jointPos + jointOff

        return self.targetQ

    def mapBodyLocForward( self, listBodyLoc ):
        if not type( listBodyLoc ) is list:
            raise NameError('input parameter is not of type "dict"')
        
        retlist = []

        for i, item in enumerate( listBodyLoc ):
            # get mapping info
            retlist.extend( self.__getBodyLocMapInfoForward( item ) )
        
        return retlist
            
    def __initJoints( self ):
        ndof = self.countNodes( "//joints/joint" )
        return np.zeros( ndof )

    # def __initBodyLocs( self ):
    #     nbloc = self.countNodes( "//joint" )
    #     return [None]*nbloc

    def __getBodyLocMapInfoForward(self, targetBodyId):
        # path to joint
        path = f"//bodylocation[name='{ targetBodyId }']"

        nodes  = self.findall( path )
        bodyid = [0]*len( nodes )

        for i, currNode in enumerate( nodes ):
            # get body id of human model
            bodyid[i]  = int( currNode.get( "id" ) )

        return bodyid
    
    def __getJointMapInfoForward(self, targetJointId):
        # path to joint
        path = f"//joint[name='{ targetJointId }']"
        # set node that represents joint
        if not self.setNode( path ):
            return 0, 0, 0

        # get moving direction
        jointDir = self.getNodeVal( "dir", valType="float", optval=0 )
        # get joint id of human model
        jointId  = self.getAttribVal( "id", valType="int", optval=0)

        # path to offset angle
        unit  = self.getAttribVal( "unit", valType="string", optval=0, path="./off")
        jointOff = self.getNodeVal( "off", valType="float", optval=0 )
        if( unit == "degree" ):
            jointOff *= math.pi/180
        elif( unit != "radiant" ):
            jointOff = 0

        return jointId, jointDir, jointOff


# class clsHMJointAndFrameMapper:
#     qbill = None
#     header = None
#     data = None
#     timesteps = None
#     idx = None
#     q = None

#     xmlObj = None

#     def __init__(self, xmlFile):
#         # read xml file
#         self.xmlObj = clsXmlReader( xmlFile )

#     def __init__(self, file):
#         # open csv file of bill
#         f = open(file)
#         csvfile = csv.reader( f )
#         self.header = next(csvfile, None)
#         self.data = list(csvfile)

#         # match idx of bill time to csv header
#         idx_time = self.header.index('simulation_step')

#         # match idx of bill angles to csv header
#         len_qbill = 11
#         self.idx = np.zeros(len_qbill).astype(int)
#         self.idx[0] = self.header.index('Bill_leftAnkleJoint')
#         self.idx[1]= self.header.index('Bill_rightAnkleJoint')
#         self.idx[2]= self.header.index('Bill_leftKneeJoint')
#         self.idx[3]= self.header.index('Bill_rightKneeJoint')
#         self.idx[4] = self.header.index('Bill_leftLegJoint')
#         self.idx[5] = self.header.index('Bill_rightLegJoint')
#         self.idx[6]= self.header.index('Bill_leftShoulderJoint')
#         self.idx[7] = self.header.index('Bill_rightShoulderJoint')
#         self.idx[8] = self.header.index('Bill_neck')
#         self.idx[9]= self.header.index('Bill_leftElbowJoint')
#         self.idx[10] = self.header.index('Bill_rightElbowJoint')
    
#         # write data into arrays
#         idx_steps = len(self.data)
#         self.timesteps = np.zeros(idx_steps)
#         self.qbill = np.zeros([idx_steps,len_qbill])

#         for idx_row in range(0,idx_steps):
#             str = self.data[idx_row][idx_time]
#             self.timesteps[idx_row] = float(str)

#             for idx_col in self.idx:
#                 str = self.data[idx_row][idx_col]
#                 self.qbill[idx_row,idx_col-1] = float(str)

#     def BillToHuman( self, human, row ):

#         qBill = self.qbill

#         q = np.zeros( human.model.NB )

#         # match angles of bill to human model
#         q[6]  = qBill[row,self.idx[4]-1]    #   K7_hip_left_z       =   Bill_leftLegJoint
#         q[13] = qBill[row,self.idx[5]-1]    #   K14_hip_right_z     =   Bill_rightLegJoint
#         q[27] = -qBill[row,self.idx[6]-1]   #   K28_shoulder_left_z =   Bill_leftShoulderJoint
#         q[20] = -qBill[row,self.idx[7]-1]   #   K21_shoulder_right_z= - Bill_rightShoulderJoint
#         q[18] = qBill[row,self.idx[8]-1]    #   K19_head_y          =   Bill_neck
#         q[3]  = qBill[row,self.idx[2]-1]    #   K4_knee_left_z      =   Bill_leftKneeJoint
#         q[10] = qBill[row,self.idx[3]-1]    #   K11_knee_right_z    =   Bill_rightKneeJoint
#         q[30] = -qBill[row,self.idx[9]-1]   #   K31_elbow_left_z    = - Bill_leftElbowJoint
#         q[23] = -qBill[row,self.idx[10]-1]  #   K24_elbow_right_z   = - Bill_rightElbowJoint
#         q[0]  = -qBill[row,self.idx[0]-1]   #   K1_ankle_left_z     =   Bill_leftAnkleJoint
#         q[7]  = -qBill[row,self.idx[1]-1]   #   K8_ankle_right_z    =   Bill_rightAnkleJoint

#         return q

#     def BaseTransformation( self, human, q):

#         # get actual position of the torso
#         EE_torso = 16   # Torso
#         Xn_torso = human.model.endEffector[EE_torso]
#         X_torso, _, _ = human.objMBM.fkin( q, EE_torso, Xn_torso)

#         # get position (especially in Y-direction) of torso in neutral position    
#         q0 = np.zeros( human.model.NB ) 
#         _, _, p0 = human.objMBM.fkin( q0, EE_torso, Xn_torso) 
#         # and calculate the transformation
#         Xy, _ = human.objMBM.jacJoint("Py", p0[1]) 

#         # get xtree of right foot
#         idx_right_foot = 7
#         X_right_foot = human.model.Xtree[idx_right_foot ]

#         # calculate new base so that torso is vertical and placed at p0
#         newbase = np.matmul( np.linalg.inv( X_torso), X_right_foot  ) 
#         newbase = np.matmul(newbase, Xy)

#         # overwrite xtree of right foot in model and submodel of the human
#         human.submodels[1].Xtree[0] = newbase
#         human.model.Xtree[idx_right_foot] = newbase

#         return human

# pos = clsHMBodyPoseReader('posture.xml')
# q1 = pos.getPosture(1)
# q1 = pos.getPosture(3)

# mybody = clsHMBodyLocationReader('body.xml')
# specbody = mybody.getBodyLocation(1)
# for e in specbody: print(e,'->',specbody[e])
