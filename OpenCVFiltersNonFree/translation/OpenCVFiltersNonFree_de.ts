<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>OpenCVFiltersNonFree</name>
    <message>
        <source>Input parameter - Desired image to extract its descriptor and keypoints by means of SIFT algorithm.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Output parameter - (n x 128) float32 data object with n descriptors</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Optional Output parameter -(n x 7) float32 data object with n keypoints. Every row contains the values (pt_x,pt_y,size,angle,response,octave,id)</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <source>Wrapped algorithms from OpenCV</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>This plugin provides wrappers for various OpenCV algorithms from its section non-free.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Implements the sift algorithm and extracts the corresponding descriptors.
The sift algorithm is a scale invariant feature transform in which image content is transformed into local feature coordinates.
In each octave, the initial image is repeatedly convolved with Gaussians to produce a set of scale space images. Adjacent Gaussians are subtracted to produce the DOG.
After each octave, the Gaussian image is down-sampled by a factor of 2.
Detect maxima and minima of difference-of-Gaussian in scale space. Each point is compared to its 8 neighbours in the current image and 9 neighbours in the scales above and below.
reference: David G. Lowe, &quot;Distinctive image features from scale-invariant key points,&quot; International Journal of Computer Vision, 60, 2 (2004), pp. 91-110.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
