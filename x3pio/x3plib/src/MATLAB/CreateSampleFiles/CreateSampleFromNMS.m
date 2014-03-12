% CREATESAMPLEFILES Create sample x3p files from a given dataset
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

%  Copyright 2011, by Dr. Georg Wiora, NanoFocus AG

%% Read sample file
s=openNMS('1-euro-star.nms')
% Convert to meter
u = s.u/1e6;
v = s.v/1e6;
z = s.z/1e6;


%% Write x3p file
meta.Date = '2008-08-25T14:14:00.0+02:00';
meta.Creator = 'Dr. Georg Wiora, NanoFocus AG';
meta.Instrument_Manufacturer = 'NanoFocus AG';
meta.Instrument_Model = 'µsurf explorer';
meta.Instrument_Serial = 'not available';
meta.Instrument_Version = 'Software V6.7, Instrument Version D';
meta.CalibrationDate = '2008-08-25T13:59:21.4+02:00';
meta.ProbingSystem_Type = 'NonContacting';
meta.ProbingSystem_Identification = 'Olympus 20x0.6';
meta.Comment = 'Confocal Measurement of star structure on a 1 Euro coin. Objective 20x0.6';

pinfo = writeX3P('1-euro-star.x3p','SUR',u,v,z,meta);

%% Add an offset and a rotation
rotx=10; % Rotation angle around x (degree)
R = [1,0,0;...
     0,cosd(rotx),sind(rotx);...
     0,-sind(rotx),cosd(rotx)];

pinfo = writeX3P('1-euro-star_offset_rot.x3p','SUR',u,v,z,meta,'rotation',R,'translation',[1e-6,5e-3,1e0])

%% Create missing points
zmissing = z;
zmissing(450:470,500:510) = NaN;
meta.Comment = 'Confocal Measurement of star structure on a 1 Euro coin. Objective 20x0.6. Missing points created in center of field';
pinfo = writeX3P('1-euro-star_missing.x3p','SUR',u,v,zmissing,meta)

%% Write absolute x and y coordinates
rotx=10; % Rotation angle around x (degree)
R = [1,0,0;...
     0,cosd(rotx),sind(rotx);...
     0,-sind(rotx),cosd(rotx)];

u3d = repmat(u,970,1);
v3d = repmat(v,1,970);

meta.Comment = 'Confocal Measurement of star structure on a 1 Euro coin. Objective 20x0.6. Absolute x/y axis.';
   
pinfo = writeX3P('1-euro-star_offset_rot_absxy.x3p','SUR',u3d,v3d,z,meta,'rotation',R,'translation',[1e-6,5e-3,1e0])

%% Add a small jitter (500 nm) on x and y coordindates to test the interpolation
u3dj = u3d + rand(size(u3d,1),size(u3d,2))*5e-7;
v3dj = v3d + rand(size(u3d,1),size(u3d,2))*5e-7;

meta.Comment = 'Confocal Measurement of star structure on a 1 Euro coin. Objective 20x0.6. Absolute x/y axis and 500nm jitter on x/y coordinates.';

pinfo = writeX3P('1-euro-star_offset_rot_absxy_jitter.x3p','SUR',u3dj,v3dj,z,meta,'rotation',R,'translation',[1e-6,5e-3,1e0])


%% Add a second layer with a 100 µm offset
z2 = repmat(z,[1,1,2]);
z2(:,:,2) = z2(:,:,2) + 100e-6;
meta.Comment = 'Confocal Measurement of star structure on a 1 Euro coin. Objective 20x0.6. Second layer created by z-shift of 100µm.';
pinfo = writeX3P('1-e uro-star_2_layer.x3p','SUR',u,v,z2,meta)

%% Create a profile
prof = z(500,:);
meta.Comment = 'Confocal Measurement of star structure on a 1 Euro coin. Objective 20x0.6. Line profile extracted from original dataset at line 500.';
pinfo = writeX3P('1-euro-star_profile.x3p','PRF',u,v(500),prof,meta)
%% Create a two layer profile
prof2 = repmat(z(:,500),[1,1,2]);
% Add 100 µm offset in z
prof2(:,1,2) = prof2(:,1,2)+1e-4;

meta.Comment = ['Confocal Measurement of star structure on a 1 Euro coin.',...
  'Objective 20x0.6.',...
  'Line profile extracted from original dataset at column 500.',...
  'Second layer created by z-shift of 100µm.'];
pinfo = writeX3P('1-euro-star_profile_2_layer.x3p','PRF',u,v(500),prof2,meta)

%% Create a profile with rotation and offset
prof = z(:,500);
meta.Comment = ['Confocal Measurement of star structure on a 1 Euro coin.',...
  'Objective 20x0.6.',...
  'Line profile extracted from original dataset at line 500.',...
  'Rotation 10 deg around x axis, and offset vector (x,y,z)=(1e-6,5e-3,1e0)'];
pinfo = writeX3P('1-euro-star_profile_rot_trans.x3p','PRF',u(500),v,prof,meta,'rotation',R,'translation',[1e-6,5e-3,1e0])
