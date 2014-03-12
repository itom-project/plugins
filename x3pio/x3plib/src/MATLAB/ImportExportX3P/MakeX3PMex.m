% MakeX3PMex  Compile the Mex files for openGPS X3P import/export
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

%  Copyright 2008, by Dr. Georg Wiora, NanoFocus AG



%% Unload all libraries
clear all libraries

% Fileseparator
fs=filesep;
% go one level up
up=['..',fs];

% Get current directory
mexpath = which('MakeX3PMex');
d = [fileparts(mexpath),filesep];

% Debug flag
% debug='-g '
debug=''

%% Build path names

% Installation path of x3p lib
InstallX3P = [cd(cd([d,up,up,up,up,'install'])),fs]
% Include directory for x3p
IDirX3P = ['-I"',InstallX3P,'include',fs,'" ']
% Library directory for x3p
LDirX3P = ['-L"',InstallX3P,'lib',fs,'" ',...
           '-L"',InstallX3P,'bin',fs,'" ']
% Set library name of x3P DLL
LibX3P= '-lISO5436_2_XML_S '

% Path of x3p dll
LibX3PPath = [InstallX3P,'bin',fs,'ISO5436_2_XML.dll']
% Path of Sample files
X3PSamplePath = [InstallX3P,'SampleFiles',fs,'*.x3p']
           
%%

% Search installation of Codesynthesis XSD-tool and Xerces library
% This will only work on windows
if ispc()
  % Get system PATH environment variable and look for codesynthesys
  sp = getenv('PATH');
  % Separate paths
  spk = strfind(lower(sp), 'codesynthesis');
  % Get first occurence
  if numel(spk)<1
    error('Please specifiy the include and library path of codesynthesys Xerces library and remove this error message!');
    IDirCS='Please specify path to include files here';
    LDirCS='Please specify path to library directory here';
  else
    spk = spk(1);
    % Find separator before location
    st=1;
    for i=spk:-1:1
      if sp(i) == ';'
        st = i+1;
        break;
      end
    end
    % Find separator after location
    en=numel(sp);
    for i=spk:en
      if sp(i) == ';'
        en = i-1;
        break;
      end
    end
    % grep the bin path of codesynthesys
    DirCS = [sp(st:en),fs];
    % Set include and library path
    IDirCS=['-I"',cd(cd([DirCS,up,'include'])),fs,'" ']
    % ToDo: We stick to vc8 here, this may have to be changed
    warning('Assuming you are using Visual C8.0. If this is not the case please change the version of Codysynthesys libraries!');
    LDirCS=['-L"',cd(cd([DirCS,up,'lib',fs,'vc-8.0'])),fs,'" ',...
            '-L"',DirCS,'" ']
    % Path to xerces DLL
    LibXercesPath = [DirCS,'xerces-c_2_7_vc80.dll']
  end
else
  error('Please specifiy the include and library path of codesynthesys Xerces library and remove this error message!');
  IDirCS='Please specify path to include files here';
  LDirCS='Please specify path to library directory here';
end

%% Create compile commands
cmex=['mex ',debug,IDirX3P,LDirX3P,LibX3P,IDirCS,LDirCS,'COMPFLAGS="$COMPFLAGS /Zc:wchar_t" '];
cmexopenX3P=[cmex,'openX3P.cpp ','X3PUtilities.cpp'];
cmexwriteX3P=[cmex,'writeX3P.cpp ','X3PUtilities.cpp'];

%% Check date of mexfiles and compare to source files
  datsrc(1) = dir('openX3P.cpp');
  datsrc(2) = dir('writeX3P.cpp');
  datsrc(3) = dir('X3PUtilities.h');
  datsrc(4) = dir('X3PUtilities.cpp');
  datsrc(5) = dir('MakeX3PMex.m');
  datmex.openX3P_mex = dir(['openX3P.',mexext]);
  datmex.writeX3P_mex = dir(['writeX3P.',mexext]);
  
  % Compare youngest source file to mex date
  if numel(dir([d,'openX3P.',mexext]))==1
    if (max([datsrc([1,3,4,5]).datenum]) >= datmex.openX3P_mex.datenum)
      build_openX3P=true;
    else
      build_openX3P=false;
    end
  else
    build_openX3P=true;
  end
  
  % Compare youngest source file to mex date
  if numel(dir([d,'writeX3P.',mexext])) == 1
    if (max([datsrc([2,3,4,5]).datenum]) >= datmex.writeX3P_mex.datenum)
      build_writeX3P=true;
    else
      build_writeX3P=false;
    end
  else
    build_writeX3P=true;
  end

%% compile
if build_openX3P
  disp(['openX3P.',mexext,' is out of date. Rebuilding.']);
  disp(cmexopenX3P);
  eval(cmexopenX3P)
else
  disp(['openX3P.',mexext,' is up to date. Not touching.']);
end

if build_writeX3P
  disp(['writeX3P.',mexext,' is out of date. Rebuilding.']);
  disp(cmexwriteX3P);
  eval(cmexwriteX3P)
else
  disp(['writeX3P.',mexext,' is up to date. Not touching.']);
end

%% Copy the dlls to the same directory as the mex files
copyfile(LibX3PPath,'.')
copyfile(LibXercesPath,'.')

%% Copy sample files
% Clear all x3p files from current dir
disp('Delete all x3p files in current directory...');
delete('*.x3p');
% Copy sample files to current dir
disp('Copy sample files to current directory...');
copyfile(X3PSamplePath,'.')
