<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>PclTools</name>
    <message>
        <location filename="../pclTools.cpp" line="+4727"/>
        <source>Point Cloud (*.pcd *.ply *.vtk *.xyz)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>saves polygonMesh to hard drive (format obj[default], ply, vtk, stl)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+0"/>
        <location line="+3"/>
        <source>Polygon Mesh (*.obj *.ply *.vtk *.stl)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+0"/>
        <source>loads polygonMesh from hard drive and returns it (format obj[default], ply, vtk, stl)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>VTK Image Data File (*.vti)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>estimates normal vectors to the given input point cloud and returns the normal-enhanced representation of the input point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>removes NaN values from input point cloud (input and output can be the same).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>filters a point cloud by giving boundary values to a specific dimension (outside or inside of this field).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>downsamples a point cloud using a voxelized gripd approach.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>uses point neighborhood statistics to filter outlier data.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>randomly reduces the point cloud to a given number of points.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>get the minimum and maximum values on each of the 3 (x-y-z) dimensions in a given pointcloud.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>returns the threshold value at the percentage value in the sorted values of the specific field.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>returns the histogram of the specific field.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>clips points from the point cloud that lie outside a given cylindrical tube.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>determines PCA of point Cloud.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>get sub-mesh from given mesh by the indices of the polygons.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>calculates polygon mesh with triangles only. This is based on the ear-clipping algorithm, the complexity is n^3 and it does not handle holes.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>creates a triangle based, polygonial mesh from an organized point cloud. The triangles are always spanned between neighboured points of the organized cloud.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-4614"/>
        <source>point cloud to save</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>complete filename (type is either read by suffix of filename or by parameter &apos;type&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>mode (b=binary (default) or t=ascii, for type &apos;pcd&apos; and &apos;ply&apos; only)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>type (&apos;pcd&apos; [default],&apos;ply&apos;,&apos;vtk&apos;,&apos;xyz&apos;)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+30"/>
        <source>type was set to &apos;pcd&apos;, since &apos;pcd&apos;,&apos;ply&apos;,&apos;xyz&apos; or &apos;vtk&apos; expected</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>mode is set to &apos;b&apos; since for type &apos;pcd&apos; or &apos;ply&apos; mode is expected to be &apos;t&apos; (ascii) or &apos;b&apos; (binary).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>can not save invalid point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+81"/>
        <location line="+197"/>
        <source>ply-support is not compiled in this version (since this is not supported in PCL1.5.1 or lower</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-138"/>
        <source>pointCloud could not be saved: %s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>error while saving point cloud (internal error of method in point cloud library</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+30"/>
        <source>loaded pointcloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>complete filename (type is read by suffix)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>type (&apos;xyz&apos;, &apos;pcd&apos;,&apos;ply&apos;,&apos;auto&apos; [default, check suffix of filename])</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>filename &apos;%s&apos; does not exist.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>empty filename not allowed.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <location line="+579"/>
        <source>file &apos;%s&apos; could not be opened.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-546"/>
        <location line="+572"/>
        <source>unsupported format &apos;%s&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-448"/>
        <source>The loaded point cloud has an uncompatible format.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>The loaded point cloud does not contain x,y,z-fields.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Error while loading the point cloud. Message: %s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+29"/>
        <source>data object to save (two or three dimensional, uint8 or uint16)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>complete filename, ending .vti will be appended if not available</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>mode (b=binary (default) or t=ascii)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>name of scalar field, e.g. &apos;scalars&apos; (zero values will be transparent), &apos;ImageScalars&apos; (zero values will be displayed)...</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>values &lt;= threshold will be set to 0 (transparent values for scalar field name &apos;scalars&apos;)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+24"/>
        <location line="+15"/>
        <source>dataObject must be uint8 or uint16</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>filename is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+150"/>
        <source>polygon mesh to save</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>complete filename (type is either read by suffix of filename or by parameter &apos;type&apos;)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>type (&apos;obj&apos; [default],&apos;ply&apos;,&apos;vtk&apos;,&apos;stl&apos;)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>If 1 (default), the file is written as binary file, else ascii. If type is &apos;obj&apos;, the file is always an ascii file. (This option is only considered for PCL &gt; 1.8.0).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Precision (default: 5), only valid for &apos;obj&apos;-file types.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+30"/>
        <source>type was set to &apos;obj&apos;, since &apos;obj&apos;,&apos;ply&apos;, &apos;stl&apos; or &apos;vtk&apos; expected</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>invalid polygon mesh cannot be saved</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>error while saving polygon mesh (internal error of method in point cloud library</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+33"/>
        <source>loaded polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>complete filename (type is read by suffix</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>type (&apos;obj&apos;,&apos;vtk&apos;,&apos;stl&apos;,&apos;ply&apos;, &apos;auto&apos; [default, check suffix of filename])</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+29"/>
        <source>filename &apos;%s&apos; does not exist</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Filename is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+51"/>
        <source>error while loading polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>The affine transform is applied to this point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclModelFitGeneric.cpp" line="+77"/>
        <location filename="../pclTools.cpp" line="+21"/>
        <location line="+266"/>
        <location line="+284"/>
        <location line="+125"/>
        <location line="+185"/>
        <location line="+203"/>
        <location line="+189"/>
        <location line="+148"/>
        <location line="+137"/>
        <location line="+136"/>
        <location line="+193"/>
        <location line="+248"/>
        <location line="+126"/>
        <source>point cloud must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclProjectInliers.cpp" line="+121"/>
        <location line="+85"/>
        <location filename="../pclModelFitGeneric.cpp" line="+39"/>
        <location line="+140"/>
        <location line="+328"/>
        <location filename="../pclModelFit.cpp" line="+769"/>
        <location line="+60"/>
        <source>Fit of model type %1 not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclModelFitGeneric.cpp" line="-459"/>
        <source>Angle limit must have 2 entries</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <location line="+36"/>
        <source>(normal-)axis vector must have at 3 entries</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+29"/>
        <location line="+36"/>
        <location line="+74"/>
        <source>Radius limit must have 2 entries</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-99"/>
        <location line="+57"/>
        <location line="+28"/>
        <source>(normal-)axis vector must have 3 entries</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+46"/>
        <location line="+36"/>
        <location line="+36"/>
        <source>Can not fit the supposed type to object without normals defined.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+137"/>
        <source>no model could be fit to given point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>Could not alloced result vector</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclNurbs.cpp" line="+204"/>
        <location filename="../pclModelFit.cpp" line="-662"/>
        <location line="+222"/>
        <location line="+39"/>
        <location line="+38"/>
        <location line="+38"/>
        <location line="+44"/>
        <location line="+42"/>
        <location line="+42"/>
        <location line="+69"/>
        <location filename="../pclTools.cpp" line="-2034"/>
        <source>Input point cloud with normal values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fitted mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>polynomial order of the B-spline surface</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>number of refinement iterations, where for each iteration control-points are inserted, approximately doubling the control points in each parametric direction of the B-spline surface</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>number of iterations that are performed after refinement is completed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>the number of vertices in each parametric direction, used for triangulation of the B-spline surface</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>filename of a *.3dm OpenNURBS file. If given, the B-spline surface and curve are saved as two layers in the file</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>parameters for the B-spline surface fitting (interior_smoothness, interior_weight, boundary_smoothness, boundary_weight)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>parameters for the B-spline curve fitting (addCPsAccuracy, addCPsIteration (int), maxCPs (int), accuracy, iterations (int), closest_point_resolution (int), closest_point_weight, closest_point_sigma2, interior_sigma2, smooth_concavity, smoothness)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclModelFit.cpp" line="-533"/>
        <location filename="../pclTools.cpp" line="+2"/>
        <source>Model type according to enum pcl::SacModel</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <location line="+219"/>
        <location line="+39"/>
        <location line="+38"/>
        <location line="+38"/>
        <source>radius limits [min, max]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-330"/>
        <location line="+334"/>
        <location line="+43"/>
        <source>(normal-)axis to fit to [x, y, z]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-376"/>
        <source>maximum divergence between (normal-)axis and model orientation in radiant</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>if 1: A nonlinear optimization over all 7 parameters is applied (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Vector with the model coefficients according to model definition.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+94"/>
        <source>Input data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Number of random samples. If this number is in the range [256,65355], randomly selected values from the data object are taken into account for the fit, else all values are used for the ransac fit</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+121"/>
        <source>point on axis of symmetry of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>axis of symmetry of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>fitted radius of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+108"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+43"/>
        <source>maximum divergence between (normal-)axis and model oriantation in radiant</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-461"/>
        <location line="+216"/>
        <location line="+39"/>
        <location line="+38"/>
        <location line="+42"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+43"/>
        <source>Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-462"/>
        <location line="+216"/>
        <location line="+39"/>
        <location line="+38"/>
        <location line="+42"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+43"/>
        <source>maximum number of RANSAC iterations [default: 10000]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-462"/>
        <location line="+216"/>
        <location line="+39"/>
        <location line="+38"/>
        <location line="+42"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+43"/>
        <source>distanceThreshold of pcl [default: 0.05]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-246"/>
        <location line="+39"/>
        <source>if 1: nonlinear optimization over al 7 parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-254"/>
        <location line="+216"/>
        <location line="+39"/>
        <location line="+38"/>
        <location line="+42"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+43"/>
        <source>the probability of choosing at least one sample free from outliers. [default: 0.99]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-356"/>
        <source>The api-functions are not defined. Init of filter failed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>Input DataObject must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Could not allocated new pointcloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-129"/>
        <location line="+218"/>
        <location line="+38"/>
        <location line="+38"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+42"/>
        <location line="+44"/>
        <source>number of points considered after filtering outliers (due to RANSAC principle)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclModelFitGeneric.cpp" line="-483"/>
        <source>pclFitCylinder not implemented for PCL 1.6.1 or lower</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+256"/>
        <source>invalid point cloud type not allowed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+209"/>
        <source>point cloud must have normal vectors defined.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclModelFit.cpp" line="-211"/>
        <source>resulting center point of spehre</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>resulting fitted radius of sphere</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+33"/>
        <location line="+42"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+43"/>
        <source>if 1: nonlinear optimization over all parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-166"/>
        <source>resulting center point (xy) of circle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>resulting fitted radius of circle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+41"/>
        <source>resulting center point of the circle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+42"/>
        <source>resulting normal vector</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-41"/>
        <source>resulting fitted radius of the circle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+42"/>
        <source>resulting last value of Hesse Form</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+31"/>
        <location line="+43"/>
        <source>axis to fit to [x, y, z]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-33"/>
        <location line="+43"/>
        <source>resulting point on the line</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-42"/>
        <location line="+43"/>
        <source>resulting oriantation vector</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-15"/>
        <source>opening angle limits in radiant [min, max]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>resulting opening angle in radiant</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+53"/>
        <location filename="../pclTools.cpp" line="-1"/>
        <source>Output point cloud with distances</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+755"/>
        <source>Model type according to enum pcl::SacModel (sphere: 4, cylinder: 5)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-752"/>
        <location filename="../pclTools.cpp" line="+4"/>
        <source>point on cylinder symmetrie axis</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location filename="../pclTools.cpp" line="+1"/>
        <source>symmetrie axis of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location filename="../pclTools.cpp" line="+1"/>
        <source>cylinder radius</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+748"/>
        <source>Input data object (real type)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output distance object (inplace allowed)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>model coefficients (sphere: p_x, p_y, p_z, r; cylinder: p_x, p_y, p_z, v_x, v_y, v_z, r)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+84"/>
        <source>four coefficients are required for a sphere model (p_x, p_y, p_z, r)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>seven coefficients are required for a sphere model (p_x, p_y, p_z, v_x, v_y, v_z, r)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>modelType %i not supported.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclProjectInliers.cpp" line="-119"/>
        <location filename="../pclModelFit.cpp" line="-818"/>
        <source>pclDistanceToModel not implemented for PCL 1.6.1 or lower</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <location filename="../pclModelFit.cpp" line="+6"/>
        <source>input point cloud must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <location filename="../pclModelFit.cpp" line="+6"/>
        <source>output point cloud must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+51"/>
        <location filename="../pclModelFit.cpp" line="+31"/>
        <source>Spherical model must have [x,y,z] and r. [x,y,z] was not defined correctly.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+19"/>
        <location filename="../pclModelFit.cpp" line="+17"/>
        <source>Cylinder model must have 7 parameters, [x,y,z], [dx, dy, dz] and r. [x,y,z] was not defined correctly.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <location filename="../pclModelFit.cpp" line="+11"/>
        <source>Cylinder model must have [x,y,z], [dx, dy, dz] and r. [dx,dy,dz] was not defined correctly.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+39"/>
        <location filename="../pclModelFit.cpp" line="+36"/>
        <location filename="../pclTools.cpp" line="+3048"/>
        <location line="+142"/>
        <source>invalid point cloud type not defined or point cloud invalid</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+75"/>
        <location filename="../pclModelFit.cpp" line="+75"/>
        <source>invalid point cloud type or type not allowed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-163"/>
        <source>Plane model must have [nx,ny,nz] and d. [nx,ny,nz] was not defined correctly.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclTools.cpp" line="-3164"/>
        <source>Valid point cloud whose normals should be estimated</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output point cloud with estimated normals. The type corresponds to the normal-enhanced type of the input cloud.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>the number of k nearest neighbors to use for the feature estimation [default: 50]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>optional camera view point (if given it must have three entries [x,y,z], [default: not used])</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+35"/>
        <source>view point must be empty or a double list with three elements.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>input point cloud must be valid</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+104"/>
        <source>the alpha values of the input point cloud cannot be copied to the output point cloud [not supported]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>type of input point cloud not supported (no normal based input types allowed)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+134"/>
        <location line="+119"/>
        <location line="+165"/>
        <location line="+221"/>
        <location line="+191"/>
        <location line="+152"/>
        <location line="+132"/>
        <location line="+135"/>
        <location line="+198"/>
        <location line="+245"/>
        <location line="+129"/>
        <source>Valid input point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1686"/>
        <location line="+119"/>
        <location line="+165"/>
        <location line="+221"/>
        <location line="+191"/>
        <location line="+152"/>
        <source>Output point cloud with removed NaN values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-728"/>
        <source>valid field name to filter &apos;x&apos;,&apos;y&apos;,z&apos;,&apos;intensity&apos;...</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>minimum value (default: -FLT_MAX)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>maximum value (default: FLT_MAX)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+185"/>
        <source>1: values inside of range will be deleted, else 0 [default]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-157"/>
        <location line="+398"/>
        <source>field with name &apos;%s&apos; does not exist in given point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-263"/>
        <source>minimum values (x,y,z) (default: FLT_MIN)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>maximum values (x,y,z) (default: FLT_MAX)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>translation of box (dx,dy,dz) (default: zero values)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>euler rotation angles (in rad) of box (rx,ry,rz) (default: zero values)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+202"/>
        <source>voxel grid leaf size [lx,ly,lz] (3 elements necessary)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>1: downsample all fields, 0: only xyz [default]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>field name, whose value is filtered between &apos;fieldMin&apos; and &apos;fieldMax&apos; or &apos;&apos; if no field filtering should be applied [default]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>minimum field filtering value (default: FLT_MIN)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>maximum field filtering value (default: FLT_MAX)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>0 [default]: values inside of field range will be deleted, else 1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+21"/>
        <source>leafSize must be a vector with three elements</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+164"/>
        <source>number of nearest neighbors to use for mean distance estimation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>standard deviation multiplier for the distance threshold calculation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>0: the regular filter conditions are applied [default], 1: the inverted conditions are applied</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>filtered points should be kept and set to NaN [1] or removed (potentially breaking organized structure) [default: 0]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+280"/>
        <source>minimum (x,y,z) values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>maximum (x,y,z) values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+133"/>
        <source>field name, whose values are used for the determination of the threshold value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>percentage value [0.0,100.0]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>provide a dataObject if you want to access the sorted values of &apos;fieldName&apos;. The output is then a float32 data object of size [1 x n], where n is the number of valid values in the specific field.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>threshold value (NaN if point cloud was empty or invalid)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+149"/>
        <location line="+232"/>
        <source>Unable to find field name in point type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-188"/>
        <source>field name, whose values are used for the calculation of the histogram</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>lower boundary of the uniformly distributed histogram</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>upper boundary of the uniformly distributed histogram</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>number of discrete fields in the histogram</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>The histogram is a int32 data object with size [1 x steps].</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>provide a dataObject if you want to access the uniform percentage distribution. The resulting data object is of type float32 and has the size [1 x 100]. The value at index j gives the histogram value, where j% of the values lies below that value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+23"/>
        <source>histogram must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+216"/>
        <source>Output point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>point on axis of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>orientation vector of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>array with [min,max] radius</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>point on axis must have 3 elements</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>orientation vector of cylinder must have 3 elements</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>radius must contain two values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+64"/>
        <source>CylinderClipper3D not supported in PCL version &lt; 1.7.0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+25"/>
        <source>results in 3x3 float32 data object with three eigen-vectors</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>mean value</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>eigen values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+120"/>
        <source>Valid target point cloud of type XYZ</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Point cloud of same type than target cloud. This cloud is registered to the target.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>gives the number of closest source points taken into account for registration. By closest source points we mean the source points closest to the target. These points are computed anew at each iteration.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>    is the estimated rigid transform. IMPORTANT: this matrix is also taken as the initial guess for the alignment. If there is no guess, set the matrix to identity!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+82"/>
        <location line="+141"/>
        <source>Valid polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-140"/>
        <location line="+141"/>
        <source>output polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-140"/>
        <source>vector with indices of polygons that will be copied into output mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <location line="+138"/>
        <source>polygon mesh must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-105"/>
        <source>vertice index [%i] is out of range</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+147"/>
        <source>point cloud must be organized and dense</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>point cloud pointer or content must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>number of element of pointcloud must be identical to width*height</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>disperity output must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>intensity output dataObject must differ from disperity dataObject</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>curvature output dataObject must differ from disperity dataObject</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <source>point cloud must be of type a with an intensity or rgba vector to extract intensity</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>point cloud must be of type a with defined normals to extract intensity</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>point cloud type not supported be given</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <location line="+311"/>
        <source>a valid organized point cloud must be given</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-47"/>
        <source>with a PCL &lt; 1.7.0 trianglePixelSizeRows and trianglePixelSizeColumns must be 1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+24"/>
        <source>wrong triangulationType parameter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>the parameters organizedCloud and meshOut must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>the given point cloud must be organized. The height property of an organized point cloud is bigger than one.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+132"/>
        <source>Only tested / implemented for version 1.7.0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <location line="+93"/>
        <source>the parameters meshIn and meshOut must not be NULL.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-88"/>
        <source>the input mesh must be valid.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+33"/>
        <location line="+148"/>
        <source>Input point cloud with normal vector information.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-147"/>
        <location line="+148"/>
        <source>Output polygonal mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-145"/>
        <source>Maximum depth of the octTree to reconstruct. Be careful: High values might require a lot of memory and processing time.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>The minimum depth.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Set the depth at which a block iso-surface extractor should be used to extract the iso-surface.
This parameter must be &gt;= minTreeDepth. 

Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. 
(In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Get the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation.
This parameter must be &gt;= minTreeDepth. 

Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. 
(In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <location line="+152"/>
        <source>Only tested / implemented for PCL &gt;= 1.7.0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-137"/>
        <source>isoDivide must be &gt;= minDepth</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>solverDivide must be &gt;= minDepth</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <location line="+142"/>
        <source>the input point cloud must be valid.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-134"/>
        <location line="+142"/>
        <source>The type of the input point cloud must be XYZNormal, XYZINormal or XYZRGBNormal.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-58"/>
        <source>0: MarchingCubesHoppe, 1: MarchingCubesRBF</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>the iso level of the surface to be extracted.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The grid resolution in x, y, and z (default: 32 each)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>parameter that defines how much free space should be left inside the grid between the bounding box of the point cloud and the grid limits, as a percentage of the bounding box.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Method that sets the distance for ignoring voxels which are far from point cloud. 
If the distance is negative, then the distance functions would be calculated in all voxels; 
otherwise, only voxels with distance lower than dist_ignore would be involved in marching cube. 
Default value is - 1.0. Set to negative if all voxels are to be involved. 
Only used for algorithmType = MarchingCubesHoppe (0).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Set the off - surface points displacement value. Only used for algorithmType = MarchingCubesRBF (1)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <source>the parameters cloud and meshOut must not be nullptr.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+97"/>
        <source>Valid polygonal mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>output point cloud with the same point type than contained in the mesh but including normal vectors.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Optional list with indices that should be considered.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+48"/>
        <source>saves pointCloud to hard drive (format pcd(binary or ascii), ply(binary or ascii), vtk(ascii), xyz(ascii))</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>loads pointCloud from hard drive and returns it (format pcd(binary or ascii), ply(binary or ascii))</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+0"/>
        <source>Point Cloud (*.pcd *.ply *.xyz)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+111"/>
        <source>Uses copy an organized and dense pointcloud to an dataObject.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-743"/>
        <location line="+29"/>
        <location line="+194"/>
        <source>Valid, organized point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-3776"/>
        <source>file &apos;%s&apos; does not contain valid point cloud data.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+579"/>
        <source>file &apos;%s&apos; does not contain valid polygon mesh data.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+46"/>
        <source>Resulting, transformed point cloud (inplace possible)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+105"/>
        <source>4x4 homogeneous transformation matrix (uint8, int8, uint16, int16, uint32, int32, float32, float64)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-2"/>
        <source>The affine transform is applied to the points in this polygonal mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Resulting, transformed polygon mesh (inplace possible)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+19"/>
        <source>mesh must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>input mesh is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>cloud type of &apos;meshIn&apos; is invalid.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1270"/>
        <source>number of randomly picked points.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1524"/>
        <location line="+1"/>
        <location line="+1"/>
        <source>Output dataObject with z-Values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output dataObject with intensity-Values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output dataObject with curvature-Values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+25"/>
        <location line="+194"/>
        <source>output polygonal mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-191"/>
        <source>&apos;TRIANGLE_RIGHT_CUT&apos;: _always_ cuts a quad from top left to bottom right (default), &apos;TRIANGLE_LEFT_CUT&apos;: _always_ cuts a quad from top right to bottom left, &apos;TRIANGLE_ADAPTIVE_CUT&apos;: cuts where possible and prefers larger differences in z direction, &apos;QUAD_MESH&apos;: create a simple quad mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>Set the edge length (in pixels) used for iterating over rows when constructing the fixed mesh. Default: 1, neighboring pixels are connected</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Set the edge length (in pixels) used for iterating over columns when constructing the fixed mesh. Default: 1, neighboring pixels are connected</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Store shadowed faces or not (default: 1, yes).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+183"/>
        <source>number of deleted elements</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+456"/>
        <source>Projects points onto a given model.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+51"/>
        <source>Used SimplificationRemoveUnusedVertices from the PCL to simplify a pcl mesh.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>PclToolsInterface</name>
    <message>
        <location line="-4677"/>
        <source>Filters and methods for pointClouds and polygonMeshes</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="+2"/>
        <source>This plugin contains methods for filtering, transforming, saving and loading 
point clouds and polygon meshes. Most methods are wrappers for functions provided 
by the open source project PointCloudLibrary (pcl). The function calls are usually 
implemented for the cloud types supported by the itom classes itom.pointCloud 
and itom.polygonMesh (XYZ,XYZI,XYZRGBA,XYZNormals...). 

This library uses also methods from the current pcl version 1.7.1, however also 
compiles with older versions. In this case some methods are not compiled. 

This plugin also covers the methods for loading and saving point clouds and polygon 
meshes to common formats like pcd, ply, stl, obj... Once the plugin is loaded 
itom in general is also able to load and save such structures using the methods provided 
by this plugin.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+18"/>
        <source>licensed under LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1236"/>
        <location line="+296"/>
        <location line="+119"/>
        <location line="+386"/>
        <location line="+191"/>
        <location line="+152"/>
        <location line="+132"/>
        <location line="+135"/>
        <location line="+198"/>
        <location line="+245"/>
        <location line="+129"/>
        <location line="+212"/>
        <location line="+141"/>
        <location line="+51"/>
        <location line="+311"/>
        <location line="+194"/>
        <source>




</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-3593"/>
        <source>saves a 2D or 3D uint8 or uint16 data object to a VTK imageData volume image

This file format allows displaying volume data from the given 3D data object for instance using ParaView.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-516"/>
        <source>save an itom.pointCloud object to a file

The supported file formats are: 

* pcd (point cloud data file format from the point cloud library, binary or ascii mode possible depending on optional parameter &apos;mode&apos;) 
* ply (polygon file format or stanford triangle format, binary or ascii mode possible depending on optional parameter &apos;mode&apos;) 
* vtk (VTK point cloud format) 
* xyz (ascii text format where each line contains a whitespace separated list of the X, Y and Z coordinate of each point. The decimal sign is a dot, the real number precision is 6.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+230"/>
        <source>This filter loads point cloud data files to an itom.pointCloud object

The following file formats are supported:

* pcd (point cloud data file format provided from the point cloud library 
* ply (polygon file format also known under stanford triangle format - be careful ply can also contain polygon mesh data which is loaded using &apos;loadPolygonMesh&apos;). Both ascii and binary formats are supported. 
* vtk (point cloud format from the vtk library
* xyz (a whitespace separated ascii text file where each line contains the x, y and z coordinate of a point, e.g.: 2.546 -4.345 0.001) 

Usually the file format is automatically detected by the suffix of the filename. However it is also possible to indicate the 
type by the optional string parameter &apos;type&apos;.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+490"/>
        <source>save an itom.polygonMesh object to a file

The following file formats are currently supported: 

* obj (wavefront obj file format) 
* ply (polygon file format or stanford triangle format - binary file format only) 
* vtk (VTK file format) 
* stl (Stereolithography file format) 

Usually the format is guessed from the suffix of the given file name. Else use the optional parameter &apos;type&apos; to indicate the desired file format.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+114"/>
        <source>This filter loads polygon mesh data files to an itom.polygonMesh object

The following file formats are supported:

* ply (polygon file format also known under stanford triangle format - be careful ply can also contain point cloud data only which is loaded using &apos;loadPointCloud&apos;). Both ascii and binary formats are supported. 
* vtk (point cloud format from the vtk library
* obj (wavefront OBJ file format) 
* stl (Stereolithography file format)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+123"/>
        <source>Applies an affine coordinate transform to the input pointCloud 

The transformed point cloud is saved in &apos;pointCloudOut&apos; (inplace possible). The transformation matrix has to be a 4x4 homogeneous transformation matrix given by a 4x4 real dataObject (uint8, int8, uint16, int16, uint32, int32 or float32 allowed). Every point P_in in the input cloud is transformed by P_out = transform * P_in. Independent on the type of the transformation matrix, the matrix multiplication is done with float32 precision.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+106"/>
        <source>Applies an affine coordinate transform to all points of the input mesh. 

The transformed points are saved in &apos;meshOut&apos; (inplace possible). The transformation matrix has to be a 4x4 homogeneous transformation matrix given by a 4x4 real dataObject (uint8, int8, uint16, int16, uint32, int32 or float32 allowed). Every point P_in in the input mesh is transformed by P_out = transform * P_in. Independent on the type of the transformation matrix, the matrix multiplication is done with float32 precision.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+735"/>
        <source>pclCropBox is a filter that allows the user to filter all the data inside of a given box.

Indicate the minimum and maximum values in x,y and z direction for the box and optionally tranlate and rotate the box to 
adjust its position and orientation. The rotation vector are the euler angles rx, ry and rz.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2375"/>
        <source>Uses pcl::Poisson-filter to reduce a mesh / estimate the surface of an object.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+130"/>
        <source>The marching cubes surface reconstruction algorithm. 

There are two algorithms implemented: 

1. MarchingCubesHoppe: 
    using a signed distance function based on the distance 
    from tangent planes, proposed by Hoppe et. al.in: 
    
    Hoppe H., DeRose T., Duchamp T., MC - Donald J., Stuetzle W., 
    &quot;Surface reconstruction from unorganized points&quot;, SIGGRAPH &apos;92 

2. MarchingCubesRBF: 
    The marching cubes surface reconstruction algorithm, using a signed distance function based on radial 
    basis functions.Partially based on: 
    
    Carr J.C., Beatson R.K., Cherrie J.B., Mitchell T.J., Fright W.R., McCallum B.C. and Evans T.R., 
    &quot;Reconstruction and representation of 3D objects with radial basis functions&quot; 
    SIGGRAPH &apos;01</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+163"/>
        <source>calculates a point cloud with normal information which contains the normal at each triangle of the given 
polygonal mesh centered at the center of gravity of the triangle. Use indices to filter only certain triangles.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclProjectInliers.cpp" line="-79"/>
        <source>


Possible types are: 
--------------------
SACMODEL_PLANE = 0, 
SACMODEL_LINE = 1, 
SACMODEL_CIRCLE2D = 2, 
SACMODEL_CIRCLE3D = 3, 
SACMODEL_SPHERE = 4, 
SACMODEL_CYLINDER = 5, 
SACMODEL_CONE = 6, 
SACMODEL_TORUS = 7, 
SACMODEL_PARALLEL_LINE = 8, 
SACMODEL_PERPENDICULAR_PLANE = 9, 
SACMODEL_PARALLEL_LINES = 10, 
SACMODEL_NORMAL_PLANE = 11, 
SACMODEL_NORMAL_SPHERE = 12, 
SACMODEL_REGISTRATION = 13, 
SACMODEL_REGISTRATION_2D = 14, 
SACMODEL_PARALLEL_PLANE = 15, 
SACMODEL_NORMAL_PARALLEL_PLANE = 16, 
SACMODEL_STICK = 17 

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclNurbs.cpp" line="-35"/>
        <source>This filter fits a trimmed B-spline to a point cloud. 
This filter is mainly derived from the example at http://pointclouds.org/documentation/tutorials/bspline_fitting.php. 

After the fit, you can either obtain the result as a polygonMesh that is discretized from the resulting B-spline or 
the fitted B-spline can be saved in the OpenNURBS format (3dm) to the harddrive.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../pclModelFit.cpp" line="-830"/>
        <source>fits a geometric model to the given input point cloud using a RANSAC based approach. 

The method used for this fit is from the sample consensus module of point cloud library. 
(See http://docs.pointclouds.org/1.7.0/group__sample__consensus.html). 

The following models are available: 

Plane (0). Hessian Normal Form (n_vec * pt + d = 0). Coefficients: 

* n_x 
* n_y 
* n_z 
* d 

Line (1). Output is the line vector (v) and one point (p) on the line. Coefficients: 

* p_x 
* p_y 
* p_z 
* v_x 
* v_y 
* v_z 

Circle 2D (2). Output is the center point (p) of the circle and its radius (r) - (Fit in X, Y direction only). Coefficients: 

* p_x 
* p_y 
* r 

Circle 3D (3). Output is the normal vector (v), the center point (p) of the circle and the circle radius (r). Coefficients: 

* p_x 
* p_y 
* p_z 
* r 
* v_x 
* v_y 
* v_z 

Sphere (4). Output is the center point (p) and the radius (r). Coefficients: 

* p_x 
* p_y 
* p_z 
* r 

Cylinder (5)*. Output is the orientation vector (v), one point (p) on the line and the cylinder radius (r). Coefficients: 

* p_x 
* p_y 
* p_z 
* v_x 
* v_y 
* v_z 
* r 

Cone (6)*. Output is the orientation vector (v), the tip point (p) and the opening angle in rad. Coefficients: 

* p_x 
* p_y 
* p_z 
* v_x 
* v_y 
* v_z 
* angle 

Models with * need an input cloud where normal vectors are defined.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+109"/>
        <source>fits a geometric model to the given input data object using a RANSAC based approach. 

The input data object is transformed to a point cloud where the values are the Z coordinates, the X and Y coordinates are 
calculated using a meshgrid based on the axis scales and offsets. 

The method used for this fit is from the sample consensus module of point cloud library. 
(See http://docs.pointclouds.org/1.7.0/group__sample__consensus.html). 

The following models are available: 

Plane (0). Hessian Normal Form (n_vec * pt + d = 0). Coefficients: 

* n_x 
* n_y 
* n_z 
* d 

Line (1). Output is the line vector (v) and one point (p) on the line. Coefficients: 

* p_x 
* p_y 
* p_z 
* v_x 
* v_y 
* v_z 

Circle 2D (2). Output is the center point (p) of the circle and its radius (r) - (Fit in X, Y direction only). Coefficients: 

* p_x 
* p_y 
* r 

Circle 3D (3). Output is the normal vector (v), the center point (p) of the circle and the circle radius (r). Coefficients: 

* p_x 
* p_y 
* p_z 
* r 
* v_x 
* v_y 
* v_z 

Sphere (4). Output is the center point (p) and the radius (r). Coefficients: 

* p_x 
* p_y 
* p_z 
* r 

Cylinder (5). Output is the orientation vector (v), one point (p) on the line and the cylinder radius (r). Coefficients: 

* p_x 
* p_y 
* p_z 
* v_x 
* v_y 
* v_z 
* r 

Cone (6). Output is the orientation vector (v), the tip point (p) and the opening angle in rad. Coefficients: 

* p_x 
* p_y 
* p_z 
* v_x 
* v_y 
* v_z 
* angle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+179"/>
        <source>fits a cylindrical model to the given input point cloud using a RANSAC based approach (must have normals defined).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+39"/>
        <source>fits a spherical model to the given input point cloud using a RANSAC based approach</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>fits a planar circle model to the given input point cloud using a RANSAC based approach</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>fits a 3D-circle model to the given input point cloud using a RANSAC based approach</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+44"/>
        <source>fits a plane model to the given input point cloud using a RANSAC based approach</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+42"/>
        <source>fits a line model to the given input point cloud using a RANSAC based approach</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+42"/>
        <source>fits a conical model to the given input point cloud using a RANSAC based approach (must have normals defined)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+44"/>
        <source>Calculates the distances of points of a point cloud to a given model. 

**Possible types are:** 

SACMODEL_SPHERE = 4, 
SACMODEL_CYLINDER = 5, 

**Not supported yet:** 

SACMODEL_PLANE = 0, 
SACMODEL_LINE = 1, 
SACMODEL_CIRCLE2D = 2, 
SACMODEL_CIRCLE3D = 3, 
SACMODEL_CONE = 6, 
SACMODEL_TORUS = 7, 
SACMODEL_PARALLEL_LINE = 8, 
SACMODEL_PERPENDICULAR_PLANE = 9, 
SACMODEL_PARALLEL_LINES = 10, 
SACMODEL_NORMAL_PLANE = 11, 
SACMODEL_NORMAL_SPHERE = 12, 
SACMODEL_REGISTRATION = 13, 
SACMODEL_REGISTRATION_2D = 14, 
SACMODEL_PARALLEL_PLANE = 15, 
SACMODEL_NORMAL_PARALLEL_PLANE = 16, 
SACMODEL_STICK = 17 

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+781"/>
        <source>calculates the distance from points in a given data object to a model.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
