echo 

REM Setup Environment for running the demo application to the current directory
REM This directory must contain Demo.exe, iso5436_2.xsd, and optionally ISO5436_2.dll.
set OPENGPS_LOCATION=%~dp0
REM Directory path of sample files
set SAMPLEFILES=%~dp0\..\Samplefiles\

REM Run the demo programm
IF EXIST ISO5436_2_XML_Demo_D.exe (
  ISO5436_2_XML_Demo_D.exe %SAMPLEFILES%
) ELSE IF EXIST ISO5436_2_XML_Demo.exe (
  ISO5436_2_XML_Demo.exe %SAMPLEFILES%
) ELSE IF EXIST ISO5436_2_XML_Demo_DS.exe (
  ISO5436_2_XML_Demo_DS.exe %SAMPLEFILES%
) ELSE IF EXIST ISO5436_2_XML_Demo_S.exe (
  ISO5436_2_XML_Demo_S.exe %SAMPLEFILES%
)

