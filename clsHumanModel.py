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

import numpy as np                    # proprietary license, see https://github.com/numpy/numpy/blob/main/LICENSE.txt
from numpy import sin, cos
import clsMultiBodyModel as clsMBM
import clsMatBase as clsMB

import math                           # CC BY-NC 4.0 license, see https://github.com/pnavaro/math-python/blob/master/LICENCE.md 

# from xml.dom import minidom as xmldom
from dotmap import DotMap #using dictionary with . dot-operator # MIT license, see https://github.com/drgrib/dotmap/blob/master/LICENSE.txt
from lxml import etree    #reading xml-file                     # BSD license, see https://github.com/lxml/lxml/blob/master/LICENSES.txt 
from pathlib import Path                                        # Apache 2.0 license, see https://github.com/chigopher/pathlib/blob/master/LICENSE 

from stl import mesh # pip install numpy-stl                    # BSD license, see https://pypi.org/project/numpy-stl/

# plotting
import pkg_resources                                            # MIT license, see https://github.com/pypa/pkg_resources/blob/main/LICENSE
pkg_resources.require("plotly==4.14.3")
import plotly.graph_objects as gopy                             # MIT license, see https://github.com/plotly/plotly.py/blob/master/LICENSE.txt

from clsMatBase import clsMatBase as clsMB

class clsHumanModel():
    human = None

    model = DotMap()
    models = None
    gender = None
    weight = None
    height = None
    xmlFile = None
    bodyParams = None

    stlver  = 'pro'
    stlroot = '.'

    TOL = 1e-5

    # visu     = DotMap()
    # visu.rpy = np.array( [0, 0, 90] )
    # visu.xyz = np.array( [0, 0, 0] )

    dyn = None

    def __init__(self, xmlFile, gender, weight, height, stlver='basic', stlroot='.'):
        # prepare internal model description
        
        self.xmlFile = Path( xmlFile )
        if not self.xmlFile.is_file():
            # throw exception
            raise NameError('xml file is not available')

        self.model.NB           = None # numbers of elements / dof
        self.model.parent       = [] # parent element in tree
        self.model.jtype        = [] # joint type
        self.model.Xtree        = [] # tree
        self.model.I            = [] # ?
        self.model.endEffector  = [] # end effector something ?
        self.model.linkLength   = [] # length of the limb
        self.model.mass         = [] # mass of link
        self.model.inertia      = [] # inertia of link
        self.model.cog          = [] # center of gravity of link
        self.model.name         = [] # name of joint
        self.model.subID        = [] # id of sub kinematic

        self.model.mesh          = DotMap() # 3d mesh
        self.model.mesh.stl      = []
        self.model.mesh.faces    = []
        self.model.mesh.vertices = []

        self.submodels = [self.model.copy(), self.model.copy()] #make 2 copies

        self.objMBM = clsMBM.clsMultiBodyModel( self.model )

        # anthropometric data
        self.gender = gender
        self.weight = weight
        self.height = height
        
        # import xml file with body parameters as etree
        filePath = self.xmlFile.resolve().as_posix()
        self.modelDescrXml = etree.parse( filePath )

        self.stlver  = stlver
        self.stlroot = stlroot

        # create human model
        self.__createModel()
        return # __init__
    
    def __createModel(self):
        # read model description from xml dom
        path = f"//parameters[@gender='{self.gender}']"
        subModelDescr = self.modelDescrXml.find( path )                 
        
        # number separate models
        nmodel = len( subModelDescr )
        # number of joints in each model             
        ndof   = [ len(e) for e in subModelDescr ]                 
        # number of all joints in both models
        self.model.NB = sum( ndof )
        
        # offset index to switch between separate models and full model
        ioff = 0                                            

        for currSubModel, j in zip( self.submodels, range( nmodel ) ): #iterate over both partial kinematics
        # for j in range( nmodel ): #iterate over both partial kinematics
            # get description for current model
            xmlCurrSubModel = subModelDescr[j]
            # update dof           
            currSubModel.NB = ndof[j]

            if j > 0:
                # offset to go thorough second model
                ioff = ioff + ndof[j-1]
            
            # iterate over the separate kinematics and read joint parameters
            for xmlJoint, i in zip( xmlCurrSubModel, range( ndof[j] ) ):                                
                # joint name
                jname = xmlJoint.get( 'name' )

                currSubModel.name.append( jname )
                self.model.name.append( jname )
                
                # joint type
                jtype, R = self.__getJointType( xmlJoint )

                currSubModel.jtype.append( jtype )
                self.model.jtype.append( jtype )

                # fill the Xtree
                r = self.__getAbsoluteNumVal( xmlJoint, 'origin', 'xyz' )
                r = self.__getAbsoluteNumVal( xmlJoint, 'origin', 'xyz' )
                X = self.objMBM.plux( R, r )

                currSubModel.Xtree.append( X )
                self.model.Xtree.append( X )

                # append inertia properties to model
                fullInertiaTensor, mass, cog, inertiaTensor = self.__getInertiaProperties( xmlJoint )
                
                currSubModel.I.append( fullInertiaTensor )
                currSubModel.mass.append( mass )
                currSubModel.inertia.append( inertiaTensor )
                currSubModel.cog.append( cog )

                self.model.I.append( fullInertiaTensor )
                self.model.mass.append( mass )
                self.model.inertia.append( inertiaTensor )
                self.model.cog.append( cog )

                # append link length
                length = self.__getAbsoluteNumVal( xmlJoint, 'link', 'length' )
                
                currSubModel.linkLength.append( length )
                self.model.linkLength.append( length )

                # transformation for end effector
                ree = self.__getAbsoluteNumVal( xmlJoint, 'endeffector', 'xyz' )
                Xee = self.objMBM.plux( np.identity(3), ree )

                currSubModel.endEffector.append( Xee )
                self.model.endEffector.append(Xee)

                # append parent id
                # convert to int and substract 1 (python is 0-index based)
                tmp = self.__getAbsoluteNumVal( xmlJoint, 'parent', 'joint' )
                parentid = int( tmp ) - 1

                currSubModel.parent.append( parentid )
                
                # increase parentid for sub model
                if ( j > 0 ) and ( parentid >= 0 ):
                    #offset for 2nd kinematic tree
                    parentid = parentid + ioff
                self.model.parent.append(parentid)

                # load mesh if available
                vertices = None
                faces    = None

                mystl   = self.__getSTL( xmlJoint )
                if( mystl is not None ):
                    p, q, r = mystl.vectors.shape #(p, 3, 3)

                    mystl.vectors = np.around( mystl.vectors, 5 )
            
                    # extract unique vertices from all mesh triangles
                    vertices, ixr = np.unique( mystl.vectors.reshape(p*q, r), return_inverse=True, axis=0 )

                    # get face indeces
                    I = np.take( ixr, [3*k for k in range(p)] )
                    J = np.take( ixr, [3*k+1 for k in range(p)] )
                    K = np.take( ixr, [3*k+2 for k in range(p)] )

                    # concantenate vectors
                    faces  = np.c_[I, J, K]

                self.model.mesh.stl.append( mystl )
                self.model.mesh.vertices.append( vertices )
                self.model.mesh.faces.append( faces )
        return # createModel()

    def __distanceToBodyParts( self, q, u, pi, frameid ):
        # transform pi into base frame
        pn = self.objMBM.transformLocalPointToBase( q, frameid, pi )

        Dist = DotMap()
        Dist.frameid = []
        Dist.faceid  = []
        Dist.value   = []
        Dist.Pin     = []
        Dist.N       = 0

        for i in range( self.model.NB ):
            if( self.model.mesh.vertices[i] is None ):
                continue

            Vert  = self.objMBM.transformPointList( q, i, self.model.mesh.vertices[i], transpose=True )
            Faces = self.model.mesh.faces[i]

            # sort points
            points = []
            for j in range( 3 ):
                idx = Faces[:,j]
                points.append( Vert[idx,:] )

            idx = np.argsort( np.sum( points[0], axis=1) )

            # origin
            O = points[0][idx,:] 

            # edges
            U = points[1][idx,:] - O
            V = points[2][idx,:] - O
            
            faceid, value, Pin = self.__rayIntersectsTriangle( i, pn, u, U, V, O, Faces, Vert)

            if( len( value ) == 0 ):
                continue
            
            Dist.frameid.append( i )
            Dist.faceid.append( faceid )
            Dist.value.append( value )
            Dist.Pin.append( self.objMBM.transformPointList( q, i, Pin, backwards=True) )
            
        Dist.N = len( Dist.frameid )

        return Dist
    
    def __rayIntersectsTriangle( self, i, mray, nray, U, V, O, Faces, Vert, plotWithPlotly=False):

        # normal vectors
        N      = np.cross( U, V )
        nfaces = len( N )

        Mray  = np.tile( mray, ( nfaces, 1 ) )
        Nray  = np.tile( nray, ( nfaces, 1 ) )

        rnum  = np.sum( np.multiply( N, O - Mray ), axis=1 )
        rden  = np.matmul( nray, N.T )
        idxnz = np.abs( rden ) > 1e-5

        U     = U[idxnz,:]
        V     = V[idxnz,:]
        O     = O[idxnz,:]
        N     = N[idxnz,:]

        Mray  = Mray[idxnz,:]
        Nray  = Nray[idxnz,:]

        rnum  = rnum[idxnz]
        rden  = rden[idxnz]

        r     = np.divide( rnum, rden )

        Pin = Mray + np.multiply( np.tile(r, (3, 1) ).T, Nray )
        W   = Pin - O

        uv = np.sum( np.multiply( U, V ), axis=1 )        
        wv = np.sum( np.multiply( W, V ), axis=1 )        
        wu = np.sum( np.multiply( W, U ), axis=1 )        
        vv = np.sum( np.power( V, 2 ), axis=1 )        
        uu = np.sum( np.power( U, 2 ), axis=1 )

        pden = np.power( uv, 2 ) - np.multiply( uu, vv )

        S = np.divide( np.multiply( uv, wv ) - np.multiply( vv, wu ), pden )
        T = np.divide( np.multiply( uv, wu ) - np.multiply( uu, wv ), pden )

        idxIn = ( S >= 0 ) & ( T >= 0) & ( (S + T) <= 1 )

        if np.any( idxIn ):
            idx = np.arange( nfaces )
            idx = idx[idxnz]

            if plotWithPlotly:
                fig = gopy.Figure()

                # add mesh 2 plotly figure
                self.__addMesh2Fig( fig, Faces, Vert )

                self.__addSphere2Fig( fig, mray, 0.01, "black" )
                self.__addLine2Fig( fig, np.stack( (mray, mray + nray*0.1), axis=0), "black" )
                self.__addLine2Fig( fig, np.stack( (mray - 0.5*nray, mray), axis=0), "red"   )

                idxPin = np.where( idxIn )[0]
                for k in idxPin:
                    self.__addSphere2Fig( fig, Pin[k,: ], 0.01, "green" )

                fig.show()
                

            return idx[ idxIn ], r[ idxIn ], Pin[ idxIn,: ]
        else:
            return [], [], []

    def __getSTL(self, xmlJoint):
        mymesh = None
        
        # nmodel   = xmlJoint.xpath( 'count( stl/model )' )
        stlNodes = xmlJoint.findall( 'stl/model' )
        if stlNodes is None:
            return None

        for currNode in stlNodes:
            if( currNode.get( 'type' ) != self.stlver ):
                continue

            # load mesh
            file   = currNode.get( 'file' )
            path   = f"{self.stlroot}/stl/{self.gender}/{self.stlver}/{file}"
            mymesh = mesh.Mesh.from_file( path )

            # get scaling parameters
            scalingbase = currNode.get( 'scalingbase' )
            if( scalingbase == "linklength" ):
                baseVal = self.__getAbsoluteNumVal( xmlJoint, 'link', 'length' )
            elif( scalingbase == "bodyheight" ):
                baseVal = self.height

            scaling = baseVal*self.__str2np( currNode.get( 'scaling' ) )
            # scale mesh
            mymesh = self.__scaleMesh( mymesh, scaling )

            # get transformation values
            xyz = self.__str2np( currNode.get( 'xyz' ) )
            rpy = self.__str2np( currNode.get( 'rpy' ) )      
            # transform mesh
            mymesh = self.__transformMesh( mymesh, xyz, rpy )

            break
        return mymesh

    def GetTransformationMatrix( self, q, iframe, T0=np.eye( 4 ) ):
        _, R, p = self.objMBM.fkin( q, iframe )
        Ti      = self.objMBM.Rp2T( R, p )
        return np.matmul( T0, Ti )
    
    def PlotModel( self, q, u=None, pi=None, frameid=None, bodyparts=[], T0=np.eye( 4 ) ):
        # create empty plotly figure
        fig = gopy.Figure()

        # update base (if necessary)
        self.objMBM.setBase( q )

        for i, faces, vertices in zip( range( self.model.NB ), self.model.mesh.faces, self.model.mesh.vertices ):
            if( faces is None ):
                continue

            # get transformation
            T = self.GetTransformationMatrix( q, i )

            # T0visu  = self.objMBM.xyzrpy2T( self.visu.xyz, self.visu.rpy, unit='deg')
            # Tbase   = np.matmul( T0visu, T0 )
            tmpVert = self.objMBM.transformPointList( q, i, vertices, T0=T0 )

            # add mesh 2 plotly figure
            self.__addMesh2Fig( fig, faces, tmpVert )
            
            # stop control (for debugging)
        
        if ( not u is None ) and ( not pi is None ) and ( not frameid is None ):
            pn = self.objMBM.transformLocalPointToBase( q, frameid, pi, T0=T0)
            # un = self.objMBM.applyTransformation( np.eye(4), u)
            un = u

            self.__addSphere2Fig( fig, pn, 0.01, "black" )
            self.__addLine2Fig( fig, np.stack( (pn, pn + un*0.1), axis=0), "black" )
            self.__addLine2Fig( fig, np.stack( (pn - 0.5*un, pn), axis=0), "red"   )
        
        if not isinstance( bodyparts, list ):
            bodyparts = [ bodyparts ]

        for bp in bodyparts:
            if not isinstance( bp, DotMap):
                break
            
            pn, _, _ = self.__calcBodyPartPosition( bp, q, T0=T0 )
            self.addSphere2Fig( fig, pn, 0.01 )

        fig.update_layout( scene=dict(
                 aspectmode = 'data'
         ))

        fig.show()

        return

    def __addLine2Fig( self, fig, Data, color ):
        fig.add_scatter3d(x=Data[:,0], y=Data[:,1], z=Data[:,2], 
            mode="lines",
            showlegend=False,
            line=dict(
                color = color,
                width = 5 )
            )

    def GetBodyPartPosition( self, bp, q, T0=np.eye( 4 ) ):
        baseVal = self.__getBaseValue( bp.xyz.frameid, bp.xyz.base )
        offVal  = self.__getOffsetValue( bp.xyz.frameid, bp.xyz.base, dim=3)
            
        # _, R, p = self.objMBM.fkin( q, bp.xyz.frameid)
        # T       = self.objMBM.Rp2T( R, p )
        T       = self.GetTransformationMatrix( q, bp.xyz.frameid )

        pi      = ( bp.xyz.val + offVal )*baseVal
        pn      = self.objMBM.applyTransformation( np.matmul(T0, T), pi )
        
        return pn, pi, bp.xyz.frameid

    @staticmethod
    def __addMesh2Fig( fig, faces, vertices ):
        fig.add_mesh3d(
            x = vertices[:,0],
            y = vertices[:,1],
            z = vertices[:,2], 
            i = faces[:,0], 
            j = faces[:,1], 
            k = faces[:,2],
            showscale = False,
            opacity = 0.25 )

        return

    @staticmethod
    def __addSphere2Fig( fig, p, radius, color="red" ):
        phi        = np.linspace(       0, 2*np.pi,   num=10)
        theta      = np.linspace(-np.pi/2,   np.pi/2, num=10)
        
        phi, theta = np.meshgrid( phi, theta)

        x = p[0] + cos(theta) * sin(phi) * radius
        y = p[1] + cos(theta) * cos(phi) * radius
        z = p[2] + sin(theta) * radius

        fig.add_mesh3d(x= x.flatten(), y=y.flatten(), z=z.flatten(), alphahull=1, color=color)
        # EOF addSphere2Fig()

    def __scaleMesh(self, mymesh, scaling ):
        if isinstance(scaling, float):
            s = [scaling]*3
        else:
            s = scaling
        
        mymesh.x *= s[0]
        mymesh.y *= s[1]
        mymesh.z *= s[2]
        
        return mymesh # scaleMesh()
    
    def __transformMesh(self, mymesh, xyz, rpy):
        mymesh.x += xyz[0]
        mymesh.y += xyz[1]
        mymesh.z += xyz[2]

        for i, angle in enumerate( rpy ):
            vec = [0.0]*3
            vec[i] = 0.5
            mymesh.rotate( vec, math.radians( angle ) )

        return mymesh

    def __getJointType(self, xmlJoint):
        # determine joint type and rotational offset
        jtype = xmlJoint.get( 'type' )

        if ( jtype == 'revolute' ):
            vec = self.__str2np( xmlJoint.find( 'axis' ).get( 'xyz' ) )
            q0  = float( xmlJoint.find( 'axis' ).get( 'offset' ) )
            
            # determine jtype
            R = np.identity(3)
            if all( vec == R[0] ):
                # rotation about x
                jtype = 'Rx'
                R = self.objMBM.rx( q0 )
            elif all( vec == R[1] ):
                # rotation about y
                jtype = 'Ry'
                R = self.objMBM.ry( q0 )
            elif all( vec == R[2] ):
                # rotation about z
                jtype = 'Rz'
                R = self.objMBM.rz( q0 )
            else:
                raise NameError( 'found unknown axis type' )
        else:
            raise NameError( 'found unkonwn joint type' )
        
        return jtype, R
        # EOF getJointType()

    def __getAbsoluteNumVal(self, xmlJoint, elmName, attName):
        # get abolute value of an specific attribute

        # 1) read relative value
        relval = self.__str2np( xmlJoint.find( elmName ).get( attName ) )
        # 2) read name of the base   
        basename = xmlJoint.find( elmName ).get( attName + 'base' )

        if basename is not None:
            # get value of base
            baseVal = self.__getBaseValue( xmlJoint, basename )/100
        else:
            baseVal = 1
        
        return relval*baseVal
        # EOF getAbsoluteNumVal()
    
    def __getBaseValue(self, source, basename):
        base = 1

        # get base value
        if   ( basename == 'bodyheight' ):
            base = self.height
        elif ( basename == 'bodyweight' ):
            base = self.weight
        elif ( basename == 'linklength' ):
            if isinstance( source, etree._Element):
                xmlJoint = source
                base = self.__getAbsoluteNumVal( xmlJoint, 'link', 'length' )
            elif isinstance( source, int):
                # determine base value from model
                i = source
                base = self.model.linkLength[i]
            else:
                raise ValueError("unknown source type")

        return base 
        # EOF getBaseValue()
    
    def __getOffsetValue(self, source, basename, dim=None):
        off = None
        if not dim is None:
            off = np.array( [0.0]*dim )

        # get base value
        if ( basename == 'abs_linkend' ):
            i      = source
            _, off = self.objMBM.deplux( self.model.Xtree[i] )

        return off
        # EOF getBaseValue()

    def __getInertiaProperties(self, xmlJoint):
        # get inertia properties for a given joint node
        
        # mass
        mass = self.__getAbsoluteNumVal( xmlJoint, 'cog', 'mass' )
        
        # cog (center of gravity)
        cog = self.__getAbsoluteNumVal( xmlJoint, 'cog', 'xyz')

        # inertia
        linkLen = self.__getAbsoluteNumVal( xmlJoint, 'link', 'length' )

        # radius of gyration (can be complex)
        rog = np.zeros( (3,3), dtype = np.complex_ )                           
        rog[0,0] = self.__getAbsoluteNumVal( xmlJoint, 'radiusOfGyration', 'xx' )
        rog[1,1] = self.__getAbsoluteNumVal( xmlJoint, 'radiusOfGyration', 'yy' )
        rog[2,2] = self.__getAbsoluteNumVal( xmlJoint, 'radiusOfGyration', 'zz' )
        rog[0,1] = self.__getAbsoluteNumVal( xmlJoint, 'radiusOfGyration', 'xy' ) # can be complex!
        rog[0,2] = self.__getAbsoluteNumVal( xmlJoint, 'radiusOfGyration', 'xz' ) # can be complex!
        rog[1,2] = self.__getAbsoluteNumVal( xmlJoint, 'radiusOfGyration', 'yz' ) # can be complex!
        rog[1,0] = rog[0,1]
        rog[2,0] = rog[0,2]
        rog[2,1] = rog[1,2]

        # calculate inertia tensor from rog
        I = np.real( np.square( linkLen/100*rog )*mass )

        # calculate mass matrix from interia properties
        B = self.objMBM.compInertia( mass, cog, I)

        return B, mass, cog, I
        # EOF getInertiaProperties()

    def EffectiveMass(self, q, u, pi, frameid, T0=np.eye(4) ):
        # check q
        assert len(q) == self.model.NB, "length of configuration vector q does not match the dof of the human body model loaded"
        # check frameid
        assert ( frameid >= 0 ) and ( frameid < self.model.NB ), "local frame is not available in the model"
        # check pi
        assert self.objMBM.isvector( pi ), "vector to the contact point is not valid"
        # check u
        assert self.objMBM.isvector( u ), "direction vector is not valid"

        # normalize u
        u     = np.array( u)
        # back transformation of u (into the model base frame)        
        R0, _ = self.objMBM.T2Rp( T0 )
        u     = np.dot( R0.T, u )
        u     = self.objMBM.normalizeVector( u )

        # effective mass of body part
        meff = self.objMBM.effectiveMass( q, frameid, pi, u )

        # calculate distance to all other body parts
        Meff = []
        def0 = 0
        Dist = self.__distanceToBodyParts( q, u, pi, frameid )
        for i in  range( Dist.N ):
            if( frameid == Dist.frameid[i] ):
                def0 = np.max( Dist.value[i] )
                break
        
        dist = []
        if( def0 > 0 ):
            for i in  range( Dist.N ):
                if( frameid == Dist.frameid[i] ):
                    continue

                if( np.min( Dist.value[i] ) < 0 ):
                    continue

                jmin = np.argmin( Dist.value[i] )
                if( ( Dist.value[i][ jmin ] - def0 ) < self.TOL ):
                    # append distance
                    dist.append( Dist.value[i][ jmin ] )

                    pi     = Dist.Pin[i][jmin,:]
                    iframe = Dist.frameid[i]
                    # collision here
                    Meff.append( self.objMBM.effectiveMass( q, iframe, pi, u ) )

        if( len( dist ) > 0 ):
            idx  = np.argsort( dist )
            Meff = np.take( Meff, idx )

        return meff, Meff

    
    def __str2np(self, str):
        # convert str to numpy array
        if str[-1] == 'i':
            # compley number - numpy uses j instead of i
            str = str.replace( 'i', 'j')  
            vec = np.array( [ complex(v) for v in str.split() ] )
        else:
            # regular float array
            vec = np.array( [ float(v) for v in str.split() ] )

        if( len( vec ) == 1 ):
            vec = vec.item(0)

        return vec
        # EOF str2np()

    def __printMe(self, model):
        # print model to console
        print('---\nPrint model')
        for key in model:
            if isinstance( model[key], list ):
                print( key, '->' )
                modelLen = len( model[key] )
                for item, idx in zip( model[key], range( modelLen ) ):
                    print( key, '[', idx, '] = \n', item )
            else:
                print( key, '->', model[key] )
            print('---')
        
        # EOF printMe()
