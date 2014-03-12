This Folder contains a couple of sample files created from the same
original data set using the x3p-matlab interface.
This files are untested yet and are intented for testing purposes only.

==============
  CONTENTS
==============

=+============+=========================================
#| Size       | Name
=+============+=========================================
1|  5.583.250 | 1-euro-star.x3p
2| 11.003.848 | 1-euro-star_2_layer.x3p
3|  5.582.871 | 1-euro-star_missing.x3p
4|  5.583.359 | 1-euro-star_offset_rot.x3p
5|  9.186.106 | 1-euro-star_offset_rot_absxy.x3p
6| 20.367.688 | 1-euro-star_offset_rot_absxy_jitter.x3p
7|     16.110 | 1-euro-star_profile.x3p
8|     17.188 | 1-euro-star_profile_2_layer.x3p
9|     15.574 | 1-euro-star_profile_rot_trans.x3p
=+============+=========================================


==============
  DESCRIPTION
==============
The files have the following contents:

1. areal data set in matrix representation with incremental x/y axis.
2. same as 1. with 100 µm offset for second layer
3. same as 1. with missing data area of 20 by 10 pixel in the center of the image
4. same as 1. with a rotation matrix roating 10 deg. around x-axis and an offset vector [x,y,z]=[1e-6,5e-3,1e0] meter
5. same as 4. with absolute x/y axis
6. same as 5. with a random 500nm jitter on x/y point positions added.
7. line profile extracted from 1. at line 500.
8. 2-layer profile with 100 µm z-offset.
9. line profile with rotation and translation matrix.

Dr. Georg Wiora
Head of openGPS Consortium
21-Jun-2011

==============
  LICENSE
==============
The data files and this readme file are part of the openGPS(R)[TM]
software library.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License (LGPL)
as published by the Free Software Foundation; either version 3 of
the License, or (at your option) any later version.
for detail see the files "licence_LGPL-3.0.txt" and
"licence_GPL-3.0.txt".

openGPS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

The name "openGPS" and the logo are registered as 
European trade mark No. 006178354 for
Physikalisch Technische Bundesanstalt (PTB)
http://www.ptb.de/

More information about openGPS can be found at
http://www.opengps.eu/

Copyright 2011, by Dr. Georg Wiora, NanoFocus AG
