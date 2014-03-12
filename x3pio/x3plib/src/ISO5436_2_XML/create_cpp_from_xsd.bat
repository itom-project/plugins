REM Generate XML-Tree parser from XSD file.
REM (C) by Georg Wiora, NanoFocus AG, 2008
REM This file is part of the openGPS project
REM http://www.opengps.eu/
REM This file is licensed under LGPL 3.0

REM This batch file is not needed for a build with visual studio since
REM all actions are included in the project file. But it shows how to
REM do it.

xsd.exe cxx-tree --prologue-file "xsd_Licence_Header.c" --generate-doxygen --generate-ostream --generate-serialization --char-type wchar_t --generate-comparison --generate-from-base-ctor --namespace-map http://www.opengps.eu/2008/ISO5436_2=OpenGPS::Schemas::ISO5436_2 --export-symbol _OPENGPS_EXPORT --cxx-suffix "_xsd.cxx" --hxx-suffix "_xsd.hxx" --output-dir "." "iso5436_2.xsd"

ECHO Publish Include file "iso5436_2_xsd.hxx"
xcopy /y "iso5436_2_xsd.hxx" "..\..\include\opengps\cxx\"
ECHO Publish xsd file "iso5436_2.xsd"
xcopy /y "iso5436_2.xsd" "..\..\..\install\lib\"
