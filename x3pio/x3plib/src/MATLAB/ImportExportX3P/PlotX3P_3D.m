function PlotX3P_3D(filename,PlotTitle)
  % PLOTNMS3D Display an X3P Dataset as 3-dimensional surface.
  %
  %  This file is part of the openGPS(R)[TM] software library.
  %
  % Syntax:
  %   PlotX3P_3D(filename,title)
  %
  % Parameters:
  %   filename - File specification for the x3p file to read and display
  %   title    - Plot title to display
  %
  % Example:
  %   PlotX3P_3D('KautschukInfiniteFocus.x3p','Rubber Surface')
  
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

  % Copyright 2009, Dr. Georg Wiora, NanoFocus AG
  % This file has been put under GFDL license by Georg Wiora

  % Read in x3p-File
  [z,x,y,pinfo,meta] = openX3P(filename);
  % This returns a structure with the following fields:
  %   x     - Array of x coordinates in meter
  %   y     - Array of y coordinates in meter
  %   z     - Array of z coordinates in meter
  %   pinfo - Info about data organisation
  %   meta  - Meta data structure of the file

  % Display pinfo and meta data record
  fprintf(1,'Info about data organisation:\n');
  pinfo
  fprintf(1,'\n\nMeta data:');
  meta

  % Test for surface data
  if (strcmpi(pinfo.FeatureType,'SUR')==0) || (pinfo.IsMatrix == 0)
    error(['This example programm can only display surface date, but no profiles or point clouds.', ...
      'Maybe you are able to extend its functionality?']);
  end
  % Create new figure
  figure1 = figure('name',PlotTitle);

  % Create axes
  axes1 = axes('Parent',figure1,'FontWeight','bold','FontSize',14);
  view([-20 46]);
  hold('all');

  % Create surface
  surface('Parent',axes1,'ZData',z*1e6,'YData',x*1e6,'XData',y*1e6,...
    'LineStyle','none','CData',z*1e6);

  % Create xlabel
  xlabel('X/µm','FontWeight','bold','FontSize',14);

  % Create ylabel
  ylabel('Y/µm','FontWeight','bold','FontSize',14);

  % Create zlabel
  zlabel('Z/µm','FontWeight','bold','FontSize',14);

  % Create title
  title(PlotTitle,'Interpreter','none','FontWeight','bold',...
    'FontSize',16);

  % Create light to have a shiny surface. This works best with very smooth
  % surfaces. On rough surfaces it does not look very nice.
  light('Parent',axes1,'Style','local','Position',[-100 -400 100]);

  % Create colorbar
  colorbar('peer',axes1,'FontWeight','bold','FontSize',14);
  end
