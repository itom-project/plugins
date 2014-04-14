% Test and demonstrate vendor specific extension of X3P files
%
%  This file is part of the openGPS(R)[TM] software library.
%
%  This program is free software; you can redistribute it and/or modify
%  it under the terms of the GNU Lesser General Public License (LGPL)
%  as published by the Free Software Foundation; either version 3 of
%  the License, or (at your option) any later version.
%  for detail see the files "licence_LGPL-3.0.txt" and
%  "licence_GPL-3.0.txt".
%
%  openGPS is distributed in the hope that it will be useful,
%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  GNU Lesser General Public License for more details.
%
%  You should have received a copy of the GNU General Public License
%  along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
%  The name "openGPS" and the logo are registered as 
%  European trade mark No. 006178354 for
%  Physikalisch Technische Bundesanstalt (PTB)
%  http://www.ptb.de/
%
%  More information about openGPS can be found at
%  http://www.opengps.eu/

%  Copyright 2009, by Dr. Georg Wiora, NanoFocus AG


% Filename for vendor specific extension
vextfile = 'testimage.png';
% Create an image as test file
testimage = spiral(20);
% Write image as png
imwrite(testimage,vextfile,'png');

% Read a sample file to get a valid meta data set
[z,x,y,pinfo,meta] = openX3P('ISO5436-sample1.x3p');
% Reset creation date
meta.Date = '';
% Write sample back with an extension
pinfo = writeX3P('vendortest.x3p',pinfo.FeatureType,x,y,z,meta,'Vendorspecific',vextfile);
% Delete the input file to demonstrate read back
delete(vextfile);

% Read sample back without extension, no file should be created
[z,x,y,pinfo,meta] = openX3P('vendortest.x3p');

% empty uri
[z,x,y,pinfo,meta] = openX3P('vendortest.x3p','VendorSpecificFilename',vextfile,'VendorSpecificURI','http://wrong.uri.illegal/');
% No extension in file
[z,x,y,pinfo,meta] = openX3P('ISO5436-sample1.x3p','VendorSpecificFilename',vextfile);
% Read sample back using a wrong extension
[z,x,y,pinfo,meta] = openX3P('vendortest.x3p','VendorSpecificFilename','wrong_filename.txt');
% Read back in default mode
[z,x,y,pinfo,meta] = openX3P('vendortest.x3p','VendorSpecificFilename',vextfile);
% Read back to different file
[z,x,y,pinfo,meta] = openX3P('vendortest.x3p','VendorSpecificFilename',vextfile,'VendorSpecificTargetFile','testimage2.png');
