<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.0" language="de">
<context>
    <name>FittingFilters</name>
    <message>
        <source>fits plane in 2D-dataObject and returns plane-parameters A,B,C (z=A+Bx+Cy)</source>
        <translation type="obsolete">Fittet eine Ebene in 2D-Datenobjekten und gibt Ebenenparameter A, B, C (z=A+Bx+Cy) zurück</translation>
    </message>
    <message>
        <source>subtracts plane from 2D-dataObject given by plane-parameters A,B,C (z=A+Bx+Cy)</source>
        <translation type="obsolete">Subtrahiert eine Ebene vom 2D-Datenobjekt, gegeben durch die Ebenenparameter A, B, C (z=A+Bx+Cy)</translation>
    </message>
    <message>
        <source>fits plane in 2D-dataObject and subtracts this plane from the dataObject -&gt; this is a combination of fitPlane and subtractPlane</source>
        <translation type="obsolete">Fittet eine Ebene im 2D-Datenobjekt und subtrahiert diese Ebene vom Datenobjekt -&gt; dies ist eine Kombination von &apos;fitPlane&apos; und &apos;subtractPlane&apos;</translation>
    </message>
    <message>
        <location filename="../fittingfilters.cpp" line="+1291"/>
        <source>fits 2D-polynomial in 2D-dataObject and returns a double-DataObject with the fitted surface as well as an error value sigma</source>
        <translation>Fittet ein 2D-Polynom n-ter Ordnung in ein 2D-Datenobjekt und gibt ein &apos;Double&apos;-Datenobjekt mit den gefitteten Daten und der mittleren Abweichung (sigma-Wert) zurück</translation>
    </message>
    <message>
        <location filename="../polyfit2d.cpp" line="+40"/>
        <source>uninitialized vector for mandatory, optional or output parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../fittingfilters.cpp" line="-1174"/>
        <location line="+97"/>
        <location line="+109"/>
        <location filename="../polyfit2d.cpp" line="+7"/>
        <source>source image data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-107"/>
        <source>Parameter A of regression plane z = A + Bx + Cy, which is subtracted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter B of regression plane z = A + Bx + Cy, which is subtracted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter C of regression plane z = A + Bx + Cy, which is subtracted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Error: source image must be two-dimensional.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-116"/>
        <location line="+207"/>
        <source>fitting method (leastSquareFit [default], leastSquareFitSVD)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-205"/>
        <source>Parameter A of regression plane z = A + Bx + Cy</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter B of regression plane z = A + Bx + Cy</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter C of regression plane z = A + Bx + Cy</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+117"/>
        <source>source matrix must be of type (u)int8, (u)int16, (u)int32, float32 or float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-100"/>
        <source>the chosen method is unknown</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+75"/>
        <location line="+109"/>
        <source>destination image data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-79"/>
        <location filename="../polyfit2d.cpp" line="+55"/>
        <source>destination matrix is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+60"/>
        <source>Substracted plane with A = %1, B = %2, C = %3</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+170"/>
        <source>Generated object via polyVal with order X = %1, Y = %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+218"/>
        <source>Caluclated polynomical coeffs along z-direction with order Z = %1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../polyfit2d.cpp" line="+42"/>
        <source>2D polynomical fit with order x = %1 and y = %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-94"/>
        <source>destination data object with fitted values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>number of polynoms in x-direction</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>number of polynoms in y-direction</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>if 0 infinite values in input image will be copied to output</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Variance value *sigma* of polynomial fit.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+162"/>
        <source>2:  gradX und gradY =0, Funktion abgebrochen</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>3:  gradX und/oder gradY zu gross</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+31"/>
        <source>5:  Fehler bei Speicherzuweisung fuer Rekursionskoeffizienten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <source>6:  Fehler bei Speicherzuweisung fuer Werte, NormX, NormY oder Sum</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+32"/>
        <source>7:  Fehler bei Speicherzuweisung fuer ZeilenSumme</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+186"/>
        <source>error while allocating memory</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <source>Filter-Plugin for fitting-methods.</source>
        <translation type="obsolete">Filter-Plugin für Fitting-Methoden.</translation>
    </message>
    <message>
        <location filename="../fittingfilters.cpp" line="-624"/>
        <source>Plugin with fitting algorithms.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>licensed under LPGL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>N.A.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInAlgo</name>
    <message>
        <location filename="../../../../../Build/itom/SDK/include/common/addInInterface.h" line="+1075"/>
        <source>uninitialized vector for mandatory parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>uninitialized vector for optional parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>uninitialized vector for output parameters!</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
