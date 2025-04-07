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

from distutils.command.install import INSTALL_SCHEMES # MIT license, see https://pypi.org/project/distutils-licenses/
from logging import raiseExceptions                   # Apache 2.0 license, https://github.com/googleapis/python-logging/blob/main/LICENSE 


import subprocess # MIT license, see https://github.com/hishamhm/subprocess/blob/master/LICENSE
import sys
import os

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])

if( os.name == 'nt' ):
    install( 'numpy' )
    install( 'dotmap' )
    install( 'scipy' )
    install( 'lxml' )
    install( 'pathlib' )
    install( 'stl' )
    install( 'plotly==4.14.3' )
else:
    raise NameError('installer works only in Windows')

