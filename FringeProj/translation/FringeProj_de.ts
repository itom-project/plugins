<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>FringeProj</name>
    <message>
        <location filename="../FringeProj.cpp" line="+638"/>
        <source>Calculate the indexmap for graycode image stack</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>Unwrapped phase by Graycode (CiMap)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Creates the X- and Y-Map for the given disparity map. The values consider the given scaling factor and the disparity-dependent shift due to the tilted illumination.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+37"/>
        <source>Continous 3D-image stack (uint8 or uint16)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Pixels with bright image &gt; brightUpperLimit will be set to invalid (-10)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Pixels with dark image &lt; darkLowerLimit will be set to invalid (-10)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>2D-Output object [int16, 2pi-phase-index (&gt;=0) or -10 for invalid]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <source>image memory used by calcPhaseMap4 must be continuous!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+30"/>
        <location line="+7"/>
        <source>error calling CalcCIMap</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>continuous image stack must have format uint8 or uint16</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+118"/>
        <location line="+79"/>
        <source>Contrast threashold (val &lt; threas = invalid)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-271"/>
        <source>Threshold for contrast. Only pixels with ((bright-dark) &gt; contThres) will be considered</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>Intensity values that lie in a band around the mean value (bright+dark)/2 will be ignored. The width of the band is given by safetyFactor*(bright-dark)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+103"/>
        <source>Contrast threshold (val &lt; threas = invalid)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Wrapped phase result (float32, [-pi..pi] or -10 for invalid)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Map with intensity modulation (float32, [0..max. overExp], invalids are not marked here)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+246"/>
        <source>error calling calcPhaseMap4</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-412"/>
        <source>Reconstructs wrapped phase from N phaseshifted images with a shift of 2pi / N. The definition of the phase is equal to calcPhaseMap4.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Reconstructs wrapped phase from four 90degree phase shifted images. The phase value is determined using the Carré algorithm: atan2(I3-I1,I2-I0).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+155"/>
        <source>4 x Y x X image stack (uint8 or uint16) with 4 phase shifted images (90° each)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <location line="+80"/>
        <source>Value for over-exposed pixels or 0 if it should not be considered</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-4"/>
        <source>N x Y x X continous image stack</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Wrapped phase result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Map with intensity modulation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+75"/>
        <source>Highest possible unwrapped phase</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>2D Inputobject from evaluated Graycode (int16)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>2D Raw (wrapped) phase (float32) (NaN is represented -10, else [-pi,pi])</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>2D Modulation map from phase evaluation (float32)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Absolute height value (result) [float32] range: [0,maxPha] or NaN for invalid phases</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+31"/>
        <source>image memory used by unwrapPhaseGray must be continuous and all objects have 2 dimensions only!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>ciMap must have format int16 (short)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>rawPhase must have format float32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>modulationMap must have format float32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>input dataObject differ in size!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+49"/>
        <source>2D disparity map (float32)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>2D x-map (float32)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>2D y-map (float32)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Base-Scaling value (mm/px)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Lateral-shift per disparity value (mm/mm)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>0: lateral shift in y-direction, 1: in x-direction</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>disparity map must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="-536"/>
        <source>Algorithms used for fringe projection (phase shifting and Gray code)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>This DLL contains several reconstruction algorithms for fringe projection.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+602"/>
        <source>creates a 1xN lookup table to convert a gray-code value into its corresponding decimal value. 

The gray-code has a maximal width of graycodeBitWidth. Hence, N corresponds 1 &lt;&lt; (graycodeBitWidth+1) 
To apply this lookup table to a dataObject or numpy array, consider using the numpy method take(lut,array) that returns (lut[array[i]] for i in array).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+113"/>
        <source>generates the graycode pattern that fills up the given data object of type uint8, uint16 or uint32</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
