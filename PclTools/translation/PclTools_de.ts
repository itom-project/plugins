<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.0" language="de">
<context>
    <name>PclTools</name>
    <message>
        <location filename="../pclTools.cpp" line="+5680"/>
        <source>saves pointCloud to hard drive (format pcd(binary or ascii), ply(binary or ascii), vtk(ascii), xyz(ascii)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+0"/>
        <location line="+3"/>
        <source>Point Cloud (*.pcd *.ply *.vtk *.xyz)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+0"/>
        <source>loads pointCloud from hard drive and returns it (format pcd(binary or ascii), ply(binary or ascii), vtk(ascii)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
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
        <source>transforms a point cloud with a given homogeneous transformation matrix (4x4 data object)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+36"/>
        <source>estimates normal vectors to the given input point cloud and returns the normal-enhanced representation of the input point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>removes NaN values from input point cloud (input and output can be the same).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>filters a point cloud by giving boundary values to a specific dimension (outside or inside of this field).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
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
        <location line="-5581"/>
        <source>point cloud to save</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+502"/>
        <source>complete filename (type is either read by suffix of filename or by parameter &apos;type&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-499"/>
        <source>mode (b=binary (default) or t=ascii, for type &apos;pcd&apos; and &apos;ply&apos; only)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>type (&apos;pcd&apos; [default],&apos;ply&apos;,&apos;vtk&apos;,&apos;xyz&apos;)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+31"/>
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
        <location line="+187"/>
        <source>ply-support is not compiled in this version (since this is not supported in PCL1.5.1 or lower</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-129"/>
        <source>pointCloud could not be saved: %s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>error while saving point cloud (internal error of method in point cloud library</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+23"/>
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
        <source>type (&apos;pcd&apos;,&apos;ply&apos;,&apos;vtk&apos;,&apos;auto&apos; [default, check suffix of filename])</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+44"/>
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
        <location line="+350"/>
        <source>file &apos;%s&apos; could not be opened.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-322"/>
        <source>vtk file format cannot be loaded (not supported)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <location line="+343"/>
        <source>unsupported format &apos;%s&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-219"/>
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
        <location line="+30"/>
        <source>polygon mesh to save</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>type (&apos;obj&apos; [default],&apos;ply&apos;,&apos;vtk&apos;,&apos;stl&apos;)</source>
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
        <location line="+26"/>
        <source>error while saving polygon mesh (internal error of method in point cloud library</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+28"/>
        <source>loaded polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>type (&apos;obj&apos;,&apos;vtk&apos;,&apos;stl&apos;,&apos;ply&apos;, &apos;auto&apos; [default, check suffix of filename])</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+30"/>
        <source>filename &apos;%s&apos; does not exist</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Filename is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+39"/>
        <source>file &apos;%s&apos; does not contain polygon mesh data.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>error while loading polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+82"/>
        <location line="+1"/>
        <source>The affine transform is applied to this point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>4x4 homogeneous transformation matrix</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+19"/>
        <location line="+78"/>
        <location line="+2265"/>
        <location line="+182"/>
        <location line="+130"/>
        <location line="+168"/>
        <location line="+189"/>
        <location line="+148"/>
        <location line="+138"/>
        <location line="+136"/>
        <location line="+194"/>
        <location line="+250"/>
        <location line="+127"/>
        <source>point cloud must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-3888"/>
        <location line="+140"/>
        <location line="+328"/>
        <location line="+665"/>
        <location line="+61"/>
        <location line="+711"/>
        <location line="+85"/>
        <source>Fit of model type %1 not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1981"/>
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
        <location line="+127"/>
        <location line="+158"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+42"/>
        <location line="+47"/>
        <location line="+45"/>
        <location line="+45"/>
        <location line="+42"/>
        <location line="+782"/>
        <source>Input point cloud with normal values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1245"/>
        <location line="+465"/>
        <location line="+782"/>
        <source>Model type according to enum pcl::SacModel</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1245"/>
        <location line="+156"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+42"/>
        <source>radius limits [min, max]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-282"/>
        <location line="+286"/>
        <location line="+46"/>
        <source>(normal-)axis to fit to [x, y, z]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-331"/>
        <location line="+286"/>
        <location line="+46"/>
        <location line="+45"/>
        <location line="+46"/>
        <source>maximum divergence between (normal-)axis and model oriantation in radiant</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-422"/>
        <location line="+156"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+46"/>
        <location line="+46"/>
        <location line="+45"/>
        <location line="+46"/>
        <source>Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-423"/>
        <location line="+156"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+46"/>
        <location line="+46"/>
        <location line="+45"/>
        <location line="+46"/>
        <source>maximum number of RANSAC iterations [default: 10000]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-423"/>
        <location line="+156"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+46"/>
        <location line="+46"/>
        <location line="+45"/>
        <location line="+46"/>
        <source>distanceThreshold of pcl [default: 0.05]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-423"/>
        <location line="+156"/>
        <location line="+43"/>
        <source>if 1: nonlinear optimization over al 7 parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-198"/>
        <location line="+156"/>
        <location line="+43"/>
        <location line="+42"/>
        <location line="+46"/>
        <location line="+46"/>
        <location line="+45"/>
        <location line="+46"/>
        <source>the probability of choosing at least one sample free from outliers. [default: 0.99]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-421"/>
        <source>Vector with the model coeffizients according to model definition.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+29"/>
        <source>Input dataObject (const)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Number of random samples or all if &gt;65355</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
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
        <location line="+91"/>
        <source>resulting point on axis of symmetrie of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>resulting axis of symmetrie of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>resulting fitted radius of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-157"/>
        <location line="+158"/>
        <location line="+42"/>
        <location line="+42"/>
        <location line="+47"/>
        <location line="+45"/>
        <location line="+45"/>
        <location line="+47"/>
        <source>number of points considered after filtering outliers (due to RANSAC principle)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1050"/>
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
        <location line="+357"/>
        <source>resulting center point of spehre</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>resulting fitted radius of sphere</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+37"/>
        <location line="+46"/>
        <location line="+46"/>
        <location line="+45"/>
        <location line="+46"/>
        <source>if 1: nonlinear optimization over all parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-179"/>
        <source>resulting center point (xy) of circle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>resulting fitted radius of circle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>resulting center point of the circle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+45"/>
        <source>resulting normal vector</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-44"/>
        <source>resulting fitted radius of the circle</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>resulting last value of Hesse Form</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <location line="+46"/>
        <source>axis to fit to [x, y, z]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-36"/>
        <location line="+46"/>
        <source>resulting point on the line</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-45"/>
        <location line="+46"/>
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
        <location line="+26"/>
        <location line="+782"/>
        <source>Output point cloud with distances</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-777"/>
        <location line="+781"/>
        <source>point on cylinder symmetrie axis</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-780"/>
        <location line="+781"/>
        <source>symmetrie axis of cylinder</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-780"/>
        <location line="+781"/>
        <source>cylinder radius</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-729"/>
        <location line="+772"/>
        <source>pclDistanceToModel not implemented for PCL 1.6.1 or lower</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-766"/>
        <location line="+772"/>
        <source>input point cloud must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-766"/>
        <location line="+772"/>
        <source>output point cloud must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-741"/>
        <location line="+792"/>
        <source>Spherical model must have [x,y,z] and r. [x,y,z] was not defined correctly.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-774"/>
        <location line="+793"/>
        <source>Cylinder model must have 7 parameters, [x,y,z], [dx, dy, dz] and r. [x,y,z] was not defined correctly.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-782"/>
        <location line="+793"/>
        <source>Cylinder model must have [x,y,z], [dx, dy, dz] and r. [dx,dy,dz] was not defined correctly.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-755"/>
        <location line="+794"/>
        <location line="+2532"/>
        <source>invalid point cloud type not defined or point cloud invalid</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-3251"/>
        <location line="+794"/>
        <source>invalid point cloud type or type not allowed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-163"/>
        <source>Plane model must have [nx,ny,nz] and d. [nx,ny,nz] was not defined correctly.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+285"/>
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
        <location line="+34"/>
        <source>view point must be empty or a double list with three elements.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>input point cloud must be valid</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+108"/>
        <source>the alpha values of the input point cloud cannot be copied to the output point cloud [not supported]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>type of input point cloud not supported (no normal based input types allowed)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+27"/>
        <location line="+124"/>
        <location line="+166"/>
        <location line="+191"/>
        <location line="+152"/>
        <location line="+133"/>
        <location line="+135"/>
        <location line="+198"/>
        <location line="+247"/>
        <location line="+131"/>
        <source>Valid input point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1476"/>
        <location line="+124"/>
        <location line="+166"/>
        <location line="+191"/>
        <location line="+152"/>
        <source>Output point cloud with removed NaN values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-508"/>
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
        <source>1: values inside of range will be deleted, else 0 [default]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+28"/>
        <location line="+178"/>
        <source>field with name &apos;%s&apos; does not exist in given point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-44"/>
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
        <location line="+148"/>
        <source>number of randomly picked points</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+133"/>
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
        <location line="+234"/>
        <source>Unable to find field name in point type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-190"/>
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
        <location line="+2"/>
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
        <location line="+217"/>
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
        <location line="+23"/>
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
        <location line="+65"/>
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
        <location line="+124"/>
        <location line="+134"/>
        <source>Valid polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-133"/>
        <location line="+134"/>
        <source>output polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-133"/>
        <source>vector with indices of polygons that will be copied into output mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <location line="+131"/>
        <source>polygon mesh must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-98"/>
        <source>vertice index [%i] is out of range</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+130"/>
        <source>point cloud pointer or content must not be NULL</source>
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
        <location line="+18"/>
        <source>Valid point cloud of type XYZ or XYZI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Outpot dataObject with z-Values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Outpot dataObject with intensity-Values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+24"/>
        <location line="+194"/>
        <source>Valid, organized point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-193"/>
        <location line="+194"/>
        <location line="+66"/>
        <source>output polygonal mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-257"/>
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
        <location line="+61"/>
        <source>Valid point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Depth of the octTree to reconstruct.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+108"/>
        <source>fits a model of type pcl::SAC_MODEL to the given input point cloud using a RANSAC based fit (some types must have normals defined).
Internally wrapped to pclFitModelGeneric.
See http://docs.pointclouds.org/1.7.0/group__sample__consensus.html for detailes</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fits a model of type pcl::SAC_MODEL to the given input dataObject using a RANSAC based fit.
Internally wrapped to pclFitModelGeneric.
See http://docs.pointclouds.org/1.7.0/group__sample__consensus.html for detailes</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fits a cylindrical model to the given input point cloud using a RANSAC based fit (must have normals defined). Internally wrapped to pclFitModelGeneric but with adapted output.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fits a spherical model to the given input point cloud using a RANSAC based fit. Internally wrapped to pclFitModelGeneric but with adapted output.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fits a planar circle model to the given input point cloud using a RANSAC based fit Internally wrapped to pclFitModelGeneric but with adapted output.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fits a 3D-circle model to the given input point cloud using a RANSAC based fit. Internally wrapped to pclFitModelGeneric but with adapted output.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fits a line model to the given input point cloud using a RANSAC based fit. Internally wrapped to pclFitModelGeneric but with adapted output.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fits a plane model to the given input point cloud using a RANSAC based fit. Internally wrapped to pclFitModelGeneric but with adapted output.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fits a conical model to the given input point cloud using a RANSAC based fit (must have normals defined). Internally wrapped to pclFitModelGeneric but with adapted output.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Calculates the distances of points of a point cloud to a given model.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Projects points onto a given model.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>Used SimplificationRemoveUnusedVertices from the PCL to simplify a pcl mesh.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Uses pcl::Poisson-filter to reduce a mesh / estimate the surface of an object.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>PclToolsInterface</name>
    <message>
        <location line="-5635"/>
        <source>Filters and methods for pointClouds and polygonMeshes</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="+6"/>
        <source>licensed under LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>N.A.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
