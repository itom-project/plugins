<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>QObject</name>
    <message>
        <location filename="../x3pio.cpp" line="+248"/>
        <source>x3p Import/Export</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>saves dataObject to x3p file</source>
        <translation type="obsolete">Speichert Datenobjekte im X3P-Dateiformat</translation>
    </message>
    <message>
        <location line="+50"/>
        <source>loads dataObject from x3p file</source>
        <translation type="unfinished">Läd Datenobjekte im X3P-Dateiformat</translation>
    </message>
    <message>
        <location line="+568"/>
        <source>data type mismatch</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-616"/>
        <source>This plugin provides methods to save and load dataObjects in/from the file format &apos;x3p&apos;. This format is specified in ISO 25178 - Geometrical product specification (GPS).

The library ISO 5436-2 XML, that is necessary for this plugin and included in the sources,
is licensed under the LGPL license and uses further libraries. For more information about the license
of the library itself see www.opengps.eu</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>saves dataObject to x3p file. x3p defines all axes in meter, if the unit of any axis is m, cm, mm, µm or nm they are correctly converted to m.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+32"/>
        <source>no error details</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+82"/>
        <location line="+21"/>
        <location line="+25"/>
        <location line="+21"/>
        <location line="+25"/>
        <location line="+21"/>
        <location line="+31"/>
        <location line="+28"/>
        <location line="+31"/>
        <location line="+28"/>
        <location line="+32"/>
        <location line="+31"/>
        <source>error writing is5436_2 file: </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+169"/>
        <source>error reading is5436_2 file: </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>warning while opening the file: %s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>error while opening the file: %s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+27"/>
        <source>empty data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>no filename specified</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+32"/>
        <source>unsupported data type. Supported types are (Int8), Int16, Int32, Float32 and Float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+219"/>
        <source>data set is no matrix: </source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>X3pIO</name>
    <message>
        <location line="-872"/>
        <location line="+3"/>
        <source>X3P Files (*.x3p)</source>
        <translation>X3P-Datei (*.x3p)</translation>
    </message>
    <message>
        <location line="+67"/>
        <source>DataObject</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Destination filename</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Save data in binary (1, default) or ascii format - use binary for big objects (&gt; 5000 Points)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+550"/>
        <source>data object must have at least two dimensions</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+123"/>
        <source>x3p stores its data in meter, therefore a scaling factor has to be applied. The format of the stored data is changed to double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+173"/>
        <source>unit &apos;%s&apos; cannot be interpreted. Meter as default unit is assumed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>Empty dataObject</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>source file name</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Unit of x and y axes. x3p assumes to have m as default unit, this can be scaled using other values than m. Default: m (Be careful that other units than &apos;m&apos; lead to a multiplication of all values that might exceed the data type limit.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>Unit of value axis. x3p assumes to have m as default unit, this can be scaled using other values than m. Default: m (Be careful that other units than &apos;m&apos; lead to a multiplication of all values that might exceed the data type limit.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+24"/>
        <source>empty data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>no filename specified</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+59"/>
        <source>error opening file: </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>warning while opening the file: %s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>error while opening the file: %s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+29"/>
        <source>only feature types SUR (surface) or PRF (profile) are supported.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>x3p file does not contain any data</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+24"/>
        <source>x3p file does not contain organized matrix or list data. Unordered list data is not supported.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+65"/>
        <source>x- and y-axes must have an incremental axis type. absolute x- and y-axes not supported.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>x-axis must have an incremental axis type.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
