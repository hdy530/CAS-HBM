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

import numpy as np        # proprietary license, see https://github.com/numpy/numpy/blob/main/LICENSE.txt 
import scipy.linalg as sp # BSD license, see https://github.com/scipy/scipy/blob/main/LICENSE.txt 

class clsMatBase(object):
    DEBUG_MODE_ON = False

    def __init__(self, debugModeOn=False):
        self.DEBUG_MODE_ON = debugModeOn
        pass
        # EOF __init__

    def licols(self, A):
        # https://math.stackexchange.com/questions/238591/qr-decomposition-with-column-pivoting-errror

        # tolerance
        tol = 1e-10

        _, R, P = sp.qr(A, pivoting=True)

        if (R.ndim > 1):
            diagr = np.abs( np.diag(R) )
        else:
            diagr = R[0]

        i = np.where(diagr >= tol * diagr[0])[0]

        idxP = range(0, i[-1] + 1)
        # P[ idxP ].sort()
        idx = P[idxP]
        idx.sort()

        Ax = A[:, idx]

        return Ax, idx

    def debug_printMatrix(self, A, descr=''):
        if not self.DEBUG_MODE_ON:
            return

        n = 4
        base = 10**n

        if isinstance( A, list ):
            A = np.array( A )
        elif isinstance( A, np.ndarray ):
            pass
        else:
            return

        A = np.round( A*base )/base

        if ( A.ndim == 2 ):
            r, c = A.shape
        elif ( A.ndim == 1 ):
            r = len( A )
            c = 1
        else:
            return

        A = np.reshape( A, (r, c) )

        if len( descr ) > 0:
            mystr = descr + '\n'
        else:
            mystr = ''

        mystr += '\n['

        for i in range(0, r):
            if (i > 0):
                mystr += ' ['
            else:
                mystr += '['
            
            for j in range(0, c):
                item = A[i,j]
                mystr += ('{:' + str(n+4) + '}').format(item)

            if ( i < (r-1) ):
                mystr += ' ],\n'
            else:
                mystr += ' ]'
        
        mystr += ']\n'

        # print('\n['.join( [''.join([ '{:6}'.format(item) for item in row]) for row in A] ) )
        print( mystr )
        return 

            
