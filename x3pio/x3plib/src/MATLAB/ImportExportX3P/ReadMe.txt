This file is part of the openGPS project.
(C) by Georg Wiora (NanoFocus AG)
This file is licenced under LGPL.

This directory contains MATLAB interface files ("mex" files) that
provide an import and export in x3p dataformat in matlab.

Preconditions for compiling mex files: CodeSynthesis XSD-tool 3.0 has
to be installed.

To build the mex files start matlab, configure your c-compiler by typing

  mex -setup

at the command prompt.

Then start compilation by executing the m-file

  MakeX3PMex

This will copy all libraries and the resulting mex files to the
current directory.

MakeX3PMex is preconfigured for windows systems. You will get a
warning or error message if you have to change the configuration for
your system.