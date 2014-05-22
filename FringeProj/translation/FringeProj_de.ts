<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.0" language="de">
<context>
    <name>FringeProj</name>
    <message>
        <location filename="../FringeProj.cpp" line="+589"/>
        <source>Calculate the indexmap for graycode image stack</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Reconstructs wrapped phase from Nx phaseshifts</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Reconstructs wrapped phase from 4x phaseshifts</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Unwrapped phase by Graycode (CiMap)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Creates the X- and Y-Map for the given disparity map. The values consider the given scaling factor and the disparity-dependent shift due to the tilted illumination.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+32"/>
        <source>Continous 3D-image stack (uint8 or uint16)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Threshold for contrast. Only pixels with (bright-dark &gt; contThres) will be considered</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
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
        <location line="+23"/>
        <location line="+106"/>
        <location line="+119"/>
        <source>image memory used by calcPhaseMap4 must be continuous!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-198"/>
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
        <location line="+36"/>
        <source>4 x Y x X continuous image stack (uint8 or uint16)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <location line="+119"/>
        <location line="+111"/>
        <source>Contrast threashold (val &lt; threas = invalid)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-228"/>
        <location line="+119"/>
        <source>Value for overexposured pixels</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-117"/>
        <source>Wrapped phase result (float32, [-pi..pi] or -10 for invalid)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Map with intensity modulation (float32, [0..max. overExp], invalids are not marked here)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+37"/>
        <source>wrong number of images! calcPhaseMap4 needs four 90deg phase shifted images!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+23"/>
        <location line="+7"/>
        <location line="+109"/>
        <location line="+7"/>
        <location line="+126"/>
        <source>error calling calcPhaseMap4</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-237"/>
        <source>image stack must have format uint8 or uint16</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+39"/>
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
        <location line="+107"/>
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
        <location line="+18"/>
        <source>input dataObject differ in size!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
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
        <location line="+21"/>
        <source>disparity map must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="-590"/>
        <source>Algorithms used for fringe projection (phase shifting and Gray code)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>This DLL contains several reconstruction algorithms for fringe projection.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>ito internal, do not copy</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInActuator</name>
    <message>
        <location filename="../../../../../../Build/itom/SDK/include/common/addInInterface.cpp" line="+683"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Destructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInAlgo</name>
    <message>
        <location line="+92"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../../../../../../Build/itom/SDK/include/common/addInInterface.h" line="+985"/>
        <source>uninitialized vector for mandatory parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>uninitialized vector for optional parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>uninitialized vector for output parameters!</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInBase</name>
    <message>
        <location filename="../../../../../../Build/itom/SDK/include/common/addInInterface.cpp" line="-577"/>
        <source>function execution unused in this plugin</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+29"/>
        <source>Toolbox</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+147"/>
        <source>Your plugin is supposed to have a configuration dialog, but you did not implement the showConfDialog-method</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInDataIO</name>
    <message>
        <location line="+13"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Destructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>listener does not have a slot </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>this object already has been registered as listener</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>timer could not be set</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+25"/>
        <source>the object could not been removed from the listener list</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+94"/>
        <location line="+20"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <source>not implemented</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
