<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.0" language="de">
<context>
    <name>PclTools</name>
    <message>
        <location filename="../pclTools.cpp" line="+152"/>
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
        <location line="+3"/>
        <source>fits a cylindrical model to the given input point cloud using a RANSAC based fit (must have normals defined).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
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
        <location line="+42"/>
        <source>point cloud to save</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+490"/>
        <source>complete filename (type is either read by suffix of filename or by parameter &apos;type&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-487"/>
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
        <location line="+181"/>
        <source>ply-support is not compiled in this version (since this is not supported in PCL1.5.1 or lower</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-123"/>
        <source>pointCloud could not be saved: %s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>error while saving point cloud (internal error of method in point cloud library</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
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
        <location line="+331"/>
        <source>file &apos;%s&apos; could not be opened.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-303"/>
        <source>vtk file format cannot be loaded (not supported)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <location line="+324"/>
        <source>unsupported format &apos;%s&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-200"/>
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
        <location line="+24"/>
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
        <location line="+25"/>
        <source>error while saving polygon mesh (internal error of method in point cloud library</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
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
        <location line="+35"/>
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
        <location line="+108"/>
        <location line="+160"/>
        <location line="+127"/>
        <location line="+123"/>
        <location line="+161"/>
        <location line="+182"/>
        <location line="+141"/>
        <location line="+131"/>
        <location line="+129"/>
        <location line="+187"/>
        <location line="+243"/>
        <location line="+120"/>
        <source>point cloud must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1735"/>
        <source>Input point cloud with normal values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>radius limits [min, max]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>maximum number of RANSAC iterations [default: 10000]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>distanceThreshold of pcl [default: 0.05]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>if 1: nonlinear optimization over al 7 parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>the probability of choosing at least one sample free from outliers. [default: 0.99]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
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
        <location line="+1"/>
        <source>number of points considered after filtering outliers (due to RANSAC principle)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>radiusLimits must contain of 2 entries</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>pclFitCylinder not implemented for PCL 1.6.1 or lower</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>invalid point cloud type not allowed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+80"/>
        <source>point cloud must have normal vectors defined.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>no cylindrical model could be fit to given point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+27"/>
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
        <location line="+24"/>
        <source>view point must be empty or a double list with three elements.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>input point cloud must be valid</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+68"/>
        <source>the alpha values of the input point cloud cannot be copied to the output point cloud [not supported]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>type of input point cloud not supported (no normal based input types allowed)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+20"/>
        <location line="+117"/>
        <location line="+159"/>
        <location line="+184"/>
        <location line="+145"/>
        <location line="+126"/>
        <location line="+128"/>
        <location line="+191"/>
        <location line="+240"/>
        <location line="+124"/>
        <source>Valid input point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1413"/>
        <location line="+117"/>
        <location line="+159"/>
        <location line="+184"/>
        <location line="+145"/>
        <source>Output point cloud with removed NaN values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-487"/>
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
        <location line="+171"/>
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
        <location line="+157"/>
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
        <location line="+141"/>
        <source>number of randomly picked points</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+126"/>
        <source>minimum (x,y,z) values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>maximum (x,y,z) values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+126"/>
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
        <location line="+227"/>
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
        <location line="+210"/>
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
        <location line="+18"/>
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
        <location line="+117"/>
        <location line="+127"/>
        <source>Valid polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-126"/>
        <location line="+127"/>
        <source>output polygon mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-126"/>
        <source>vector with indices of polygons that will be copied into output mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <location line="+124"/>
        <source>polygon mesh must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-91"/>
        <source>vertice index [%i] is out of range</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+116"/>
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
        <location line="+18"/>
        <source>Valid, organized point cloud</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>output polygonal mesh</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
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
</context>
<context>
    <name>PclToolsInterface</name>
    <message>
        <location line="-3009"/>
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
