<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DataObjectArithmetic</name>
    <message>
        <location filename="../dataobjectarithmetic.cpp" line="+109"/>
        <location line="+12"/>
        <location line="+3"/>
        <location line="+13"/>
        <location line="+157"/>
        <location line="+121"/>
        <source>source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-304"/>
        <location line="+16"/>
        <location line="+14"/>
        <source>result of calculation. This param can be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-2"/>
        <location line="+157"/>
        <location line="+123"/>
        <source>Ignore invalid-Values for floating point</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-277"/>
        <location line="+159"/>
        <location line="+4"/>
        <source>Index of the plane, which contains the result.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-162"/>
        <location line="+159"/>
        <location line="+4"/>
        <source>Pixelindex in y-direction.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-162"/>
        <location line="+159"/>
        <location line="+4"/>
        <source>Pixelindex in x-direction.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-153"/>
        <source>1. source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>2. source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>0 if both data objects are not equal, else 1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+78"/>
        <location line="+83"/>
        <location line="+68"/>
        <source>Error: source image is NUL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+429"/>
        <source>2D source image data object (u)int8, (u)int16, int32, float32 or float64 only.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>values &lt; lowThreshold are ignored. lowThreshold is subtracted from each valid value before COG determination.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>values &gt; highThreshold are ignored.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>y-Coordinate of COG (physical unit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>x-Coordinate of COG (physical unit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <location line="+209"/>
        <source>Error: sourceImage is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-205"/>
        <location line="+209"/>
        <source>Error: sourceImage is not initialized</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-42"/>
        <source>source image data (2D or 3D) object for operation (u)int8, (u)int16, int32, float32 or float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>destination image data object for operation, will contain evaluated COG (in physical coordinates)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>destination image data object for operation, will contain maximal intensity</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>if (max-min) along the search direction is lower or equal this pvThreshold (peak-to-valley), no cog is determined and a NaN value is set into the resulting position array (default: this threshold is not considered).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>If != 0.0, values &lt;= (max+min)*dynamicThreshold will be ignored. To only consider values above the FWHM, set this value to 0.5 (default).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>values &lt;= lowerThreshold will not be considered for the cog calculation (default: this threshold is not considered).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The search direction is along each column if 1, else along each row (default, 0)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+35"/>
        <source>Error: destCOG image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Error: destIntensity image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Error: destCOG and destIntensity must not be the same data objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+144"/>
        <source>Center of gravity can only be calculated for (u)int8, (u)int16, (u)int32, float32 or float64 data objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1002"/>
        <location line="+60"/>
        <location line="+87"/>
        <source>data type not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+64"/>
        <source>mean result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>deviation result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-240"/>
        <location line="+59"/>
        <location line="+83"/>
        <location line="+68"/>
        <location line="+60"/>
        <source>Error, object dimensions must be unequal zero</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-155"/>
        <source>Switch complex handling, 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Minimal value, this parameter be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Maximum value, this parameter. This param can be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+113"/>
        <source>Toggles the calculation mode of standard deviation over N or N-1 elements</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-239"/>
        <location line="+263"/>
        <location line="+109"/>
        <location line="+7"/>
        <source>Error: source image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+232"/>
        <source>type not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+35"/>
        <source>y-Coordinate of COG (index)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>x-Coordinate of COG (index)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>Error: source image must not have multiple planes</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>Unknown type or type not implemented for phase shifting evaluation</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DataObjectArithmeticInterface</name>
    <message>
        <location line="-816"/>
        <source>Arithmetic algorithms filters.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="-8"/>
        <source>Operations and arithmetic calculations of dataObject.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>LGPL</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
