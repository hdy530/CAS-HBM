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

import numpy as np                                  # proprietary license, see https://github.com/numpy/numpy/blob/main/LICENSE.txt
import math                                         # CC BY-NC 4.0 license, see https://github.com/pnavaro/math-python/blob/master/LICENCE.md 
from scipy.spatial.transform import Rotation as rot # BSD license, see https://github.com/scipy/scipy/blob/main/LICENSE.txt 
from clsMatBase import clsMatBase as clsMB

class clsMultiBodyModel( clsMB ):
    model = None
    ir    = 13   # index of right leg end
    il    = 6    # index of left leg end

    TOL   = 1e-3

    def __init__(self, model):
        self.model = model

        return
        # EOF __init__

    def jacJoint(self, jtype, q):
        S = np.zeros( 6 )
        x = np.zeros( 3 )

        if jtype == "Rx":
            Xj = self.rotx( q )
            i = 0
        elif jtype == "Ry":
            Xj = self.roty( q )
            i = 1
        elif jtype == "Rz":
            Xj = self.rotz( q )
            i = 2
        elif jtype == "Px":
            x[0] = q
            Xj = self.xlt( x )
            i = 3
        elif jtype == "Py":
            x[1] = q
            Xj = self.xlt( x )
            i = 4
        elif jtype == "Pz":
            x[2] = q
            Xj = self.xlt( x )
            i = 5

        S[i] = 1

        return Xj, S
        # EOF jacobianJoint()

    def rotx(self, th):
        c = np.cos( th )
        s = np.sin( th )

        R = np.array(
            [[ 1,  0,  0,  0,  0,  0],
             [ 0,  c,  s,  0,  0,  0],
             [ 0, -s,  c,  0,  0,  0],
             [ 0,  0,  0,  1,  0,  0],
             [ 0,  0,  0,  0,  c,  s],
             [ 0,  0,  0,  0, -s,  c]])

        return R
        # EOF rotx()
    
    def roty(self, th):
        c = np.cos( th )
        s = np.sin( th )

        R = np.array(
            [[ c,  0, -s,  0,  0,  0],
             [ 0,  1,  0,  0,  0,  0],
             [ s,  0,  c,  0,  0,  0],
             [ 0,  0,  0,  c,  0, -s],
             [ 0,  0,  0,  0,  1,  0],
             [ 0,  0,  0,  s,  0,  c]])

        return R
        # EOF roty()

    def rotz(self, th):
        c = np.cos( th )
        s = np.sin( th )

        R = np.array(
            [[ c,  s,  0,  0,  0,  0],
             [-s,  c,  0,  0,  0,  0],
             [ 0,  0,  1,  0,  0,  0],
             [ 0,  0,  0,  c,  s,  0],
             [ 0,  0,  0, -s,  c,  0],
             [ 0,  0,  0,  0,  0,  1]])

        return R
        # EOF rotz()

    def rx(self,th):
        c = np.cos(th)
        s = np.sin(th)
        
        R = np.array(
            [[ 1,  0,  0],
             [ 0,  c,  s],
             [ 0, -s,  c]])

        return R
        # EOF rx()

    def ry(self,th):
        c = np.cos(th)
        s = np.sin(th)

        R = np.array(
            [[ c,  0, -s],
             [ 0,  1,  0],
             [ s,  0,  c]])

        return R
        # EOF ry()

    def rz(self,th):
        c = np.cos(th)
        s = np.sin(th)

        R = np.array(
            [[ c,  s,  0],
             [-s,  c,  0],
             [ 0,  0,  1]])

        return R
        # EOF rz()

    
    def xlt(self, r):
        x = r[0]
        y = r[1]
        z = r[2]

        X = np.array(
            [[ 1,  0,  0,  0,  0,  0],
             [ 0,  1,  0,  0,  0,  0],
             [ 0,  0,  1,  0,  0,  0],
             [ 0,  z, -y,  1,  0,  0],
             [-z,  0,  x,  0,  1,  0],
             [ y, -x,  0,  0,  0,  1]])

        return X
        # EOF xlt

    def rpy2R( self, rpy, unit='rad' ):
        if( unit == 'rad' ):
            R   = rot.from_euler( 'zyx', rpy )
        else:
            deg = math.pi/180
            R   = rot.from_euler( 'zyx', rpy*deg )
        
        return R.as_matrix()

    def xyzrpy2T( self, xyz, rpy, unit='rad' ):
        return self.Rp2T( self.rpy2R( rpy, unit=unit ), xyz )

    def rpy2T( self, rpy, unit='rad' ):
        R = self.rpy2R( rpy, unit=unit)

        return self.Rp2T( R, np.array( [0, 0, 0]) )

    def compInertia(self, m, cog, I):
        C = self.skewMatrix( cog )

        M11 = np.add( I, m*np.matmul( C, C.T ) )  
        M12 = m*C
        M21 = m*C.T
        M22 = m*np.identity( 3 )

        IC = np.concatenate( (np.concatenate((M11, M12), 1), np.concatenate( (M21, M22), 1)), 0 )

        return IC
        # EOF compInertia()

    def skewMatrix(self, r):
        # create skew matrix from vector
        x = r[0]
        y = r[1]
        z = r[2]

        S = np.array(
            [[  0, -z,  y], 
             [  z,  0, -x],
             [ -y,  x,  0 ]])

        return S
        # EOF skewMatrix()

    def skewVector(self, S):
        # create skew  vector from matrix
        x = S[2,1] - S[1,2]
        y = S[0,2] - S[2,0]
        z = S[1,0] - S[0,1]

        r = np.array( [x, y, z] )

        return 0.5*r
        # EOF skewVector()

    def plux(self, E, r):
        # compose Plucker coordinate transform
        X11 =  E
        X12 =  np.zeros( (3,3) )
        X21 = -np.matmul(E, self.skewMatrix(r) )
        X22 =  E

        X1 = np.concatenate( ( X11, X12 ), 1)
        X2 = np.concatenate( ( X21, X22 ), 1)

        X = np.concatenate( ( X1, X2 ), 0 )

        return X
        # EOF plux()

    def deplux(self, X):
        # decompose Plucker coordinate transform
        R =  X[ 0:3, 0:3 ]

        S =  np.matmul(R.T, X[3:6,0:3] )
        r = -self.skewVector( S )

        return R, r
        # EOF deplux()

    def inertiaMatrix(self, q):
        n = self.model.NB
        B = np.zeros( ( n, n ) )

        Xup = []
        S   = []
        IC  = []

        for i in range(0, n):
            XJ, Stmp = self.jacJoint(self.model.jtype[i], q[i])
            S.append( Stmp )
            Xup.append( np.matmul( XJ, self.model.Xtree[i] ) )

            # composite inertia matrix
            ICi = self.compInertia( self.model.mass[i], self.model.cog[i], self.model.inertia[i] )
            IC.append( ICi )

        for i in range( n-1, -1, -1):
            j = self.model.parent[i]          
            if j >= 0:
                Xtmp = Xup[i].T @ IC[i] @ Xup[i]
                IC[j] = np.add( IC[j], Xtmp )
        
        for i in range(0, n):
            fh     = np.matmul( IC[i], S[i] )
            B[i,i] = np.matmul( S[i].T, fh )
            
            j = i
            while self.model.parent[j] >= 0:
                fh     = np.matmul( Xup[j].T, fh ) 
                j      = self.model.parent[j]
                B[i,j] = np.matmul( S[j].T, fh ) 
                B[j,i] = B[i,j]
    
        return B 
        # EOF inertiaMatrix()

    def simIter(self, i):
        n = 0
        while (i >= 0):
            i  = self.model.parent[i]
            n += 1

        return n

    def jacobian(self, q, i, Xn):
        n = self.model.NB
        J = np.zeros( (6, n) )

        _, _, pn = self.fkin(q, i, Xn)

        while ( i >= 0 ):
            _, Ri, pi = self.fkin( q, self.model.parent[i], self.model.Xtree[i] )
            _, S      = self.jacJoint( self.model.jtype[i], q[i] )

            ti = np.matmul(Ri, S[0:3]) # ggf matdot
            ri = np.cross( ti, np.subtract( pn, pi ) )

            J[0:3, i] = ti
            J[3:6, i] = ri
            
            i = self.model.parent[i]
        
        return J

    def fkin(self, q, i, Xn=None):
        if( Xn is None ):
            Xn = self.plux( np.eye(3), np.array( [0, 0, 0] ) )

        X = Xn.copy()

        while ( i >= 0 ):
            Xi, _ = self.jacJoint( self.model.jtype[i], q[i] )
            Xup   = np.matmul( Xi, self.model.Xtree[i] )

            X     = np.matmul( X, Xup )
            i     = self.model.parent[i]
        
        RT, p = self.deplux( X ) # note, X contains the transposed rotations matrix

        return X, RT.T, p
    
    def setBase( self, q ):
        # get matrix to end of leg
        Xeel = self.model.endEffector[ self.il ]
        Xeer = self.model.endEffector[ self.ir ]

        # do forward kinematic
        Xl, _, _   = self.fkin( q, self.il, Xeel )
        Xr, _, _   = self.fkin( q, self.ir, Xeer )

        # calculate matrix to update base
        X    = np.matmul( np.linalg.inv( Xl ), Xr )
        
        # update new base
        self.model.Xtree[0] = np.matmul( X, self.model.Xtree[0] )

        return

    def jointTransformation(self, q):
        # jacobian for the right leg
        Jr = self.jacobian(q, self.ir, self.model.endEffector[ self.ir ] )
        Jr = Jr[:, 7:14]
        self.debug_printMatrix( Jr )
        
        # jacobian for the left leg
        Jl = self.jacobian(q, self.il, self.model.endEffector[ self.il ] )
        Jl = Jl[:, 0:7]
        self.debug_printMatrix( Jl )

        J = np.concatenate( (Jr, -Jl), axis=1)
        self.debug_printMatrix( J )

        Js, idxs = self.licols( J )     # linearily independent columns
        self.debug_printMatrix( Js )

        # extract jacobian with dependent joints
        # idxs  = np.int( idxs )
        ns    = idxs.shape[0]
        n     = J.shape[1]
        idxn  = np.arange( n )

        idxg  = np.setdiff1d( idxn, idxs )
        ng    = idxg.shape[0]

        Jg    = J[:, idxg]
        self.debug_printMatrix( Jg )

        Jsinv = np.linalg.pinv( Js )
        self.debug_printMatrix( Jsinv )

        H     = np.matmul( Jsinv, Jg )
        self.debug_printMatrix( H )

        Pi_sub = np.zeros( (n, ng) )

        for i in range(0, n):
            idxg_log = idxn[i] == idxg
            idxs_log = idxn[i] == idxs

            if np.any( idxg_log ):
                Pi_sub[i, idxg_log] = 1
            elif np.any( idxs_log ):
                Pi_sub[i, :] = H[idxs_log, :]
            else:
                raise NameError('rats! something went wrong here...')
        self.debug_printMatrix( Pi_sub )

        # update sizes
        n  = self.model.NB
        ng = n - ns
        Pi = np.zeros( (n, ng) )

        for i in range(0,n):
            for j in range(0,ng):
                if ( np.any( i == idxs ) and (j < 8 ) ):
                    mm = i == idxs
                    Pi[i,j] = H[mm,j]
                elif ( np.any( i == idxg ) and (j < 8 ) ):
                    nn = np.argwhere(i == idxg)
                    if (nn == j):
                        Pi[i,nn] = 1
                elif ( j == (i -6 ) ):
                    Pi[i,j] = 1    
                
        Pi2 = Pi
            

        Pi = np.zeros( (n, ng) )

        nr, nc = Pi_sub.shape

        Pi[0:nr, 0:nc] = Pi_sub
        Pi[nr:n, nc:ng] = np.eye( n - nr )
        self.debug_printMatrix( Pi )

        return Pi


    @staticmethod
    def isvector( v ):
        if not isinstance( v, np.ndarray ):
            v = np.array( v )
            
        S = v.shape

        res = False
        if len( S ) == 1:
            res = True

        return res
    
    @staticmethod
    def Rp2T( R, p ):
        T = np.eye( 4 )
        T[0:3, 0:3] = R
        T[0:3, 3]   = p
        return T

    @staticmethod
    def T2Rp( T ):
        R = T[0:3, 0:3]
        p = T[0:3, 3]
        return R, p

    def normalizeVector( self, v ):
        if not self.isvector( v ):
            return v
        
        if not isinstance( v, np.ndarray ):
            v = np.array( v )
        
        vn = np.linalg.norm( v )

        if np.abs( vn ) > self.TOL:
            v = v/np.linalg.norm( v )

        return v

    def transformPointList( self, q, i, P, T0=np.eye(4), rotonly=False, backwards=False, transpose=False ):
        if P is None:
            return P
        
        npoints = len( P )
        if( npoints == 0 ):
            return P
        
        _, R, p = self.fkin( q, i )

        if backwards:
            R = R.T
            p = np.dot( -R, p )

        if transpose:
            R = R.T

        if rotonly:
            T       = self.Rp2T( R, np.array( [0, 0, 0] ) )
        else:
            T       = self.Rp2T( R, p )

        P1 = np.c_[ P, np.ones( npoints ) ]
        P1 = np.matmul( np.matmul(T0, T),  P1.T ).T

        return P1[:,0:3]
    
    def applyTransformation( self, Ti, vi ):
        vi1  = np.r_[ vi, 1.0 ]
        vn1 = np.matmul( Ti, vi1.T ).T

        return vn1[0:3]

    def transformLocalPointToBase( self, q, frameid, pi, T0=np.eye(4) ):
        Xpi      = self.plux( np.eye( 3 ), pi )
        _, _, pn = self.fkin( q, frameid, Xn=Xpi)

        return self.applyTransformation( T0, pn )

    def effectiveMass( self, q, i, poi, u ):
        Pi   = self.jointTransformation( q )
        B    = self.inertiaMatrix( q )
        self.debug_printMatrix( B )

        R    = np.eye( 3 )
        Xpoi = self.plux( R, poi )
        # self.debug_printMatrix( Xpoi )

        J    = self.jacobian( q, i, Xpoi )
        # print(i, J)
        # print(f"J: {J.shape}")
        # self.debug_printMatrix( J )

        Bg   = np.matmul( Pi.T, np.matmul( B, Pi ) )
        Jg   = np.matmul( J, Pi )

        La_inv = np.matmul( np.matmul( Jg, np.linalg.inv(Bg) ), Jg.T )
        # self.debug_printMatrix( La_inv )
        print(f"La_inv: {La_inv.shape}")

        La_invt = La_inv[3:6, 3:6]
        # print(f"La_invt: {La_invt.shape}")

        u = np.array( u )
        print(f"u: {u}")

        meff = 1/np.dot(  np.dot( La_invt, u), u )
        
        return meff

    # {@staticmethod
    # def debug_printMatrix(A):
    #     n = 4
    #     base = 10**n

    #     A = np.round( A*base )/base
    #     r, c = A.shape

    #     mystr = '\n['

    #     for i in range(0, r):
    #         if (i > 0):
    #             mystr += ' ['
    #         else:
    #             mystr += '['
            
    #         for j in range(0, c):
    #             item = A[i,j]
    #             mystr += ('{:' + str(n+4) + '}').format(item)

    #         if ( i < (r-1) ):
    #             mystr += ' ],\n'
    #         else:
    #             mystr += ' ]'
        
    #     mystr += ']\n'

    #     # print('\n['.join( [''.join([ '{:6}'.format(item) for item in row]) for row in A] ) )
    #     print( mystr )
    #     return }

            
