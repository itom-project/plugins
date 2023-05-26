<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>FFTWFilters</name>
    <message>
        <location filename="../FFTWfilters.cpp" line="+2012"/>
        <location line="+420"/>
        <source>Error: source image ptr empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-415"/>
        <location line="+420"/>
        <source>Error: dest image ptr empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1325"/>
        <location line="+41"/>
        <source>Input object (n-dimensional, (u)int8, (u)int16, int32, float32, float64, complex64, complex128)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1034"/>
        <source>Input object (n-dimensional, (u)int8, (u)int16, int32, float32, float64, complex64, complex128) which is shifted in-place.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>shift axis: x and y axis (-1, default), only y axis (0), only x axis (1)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>shift axis in the case of a &gt; 2D dataObject. (-1, default) the axis parameter is considered, (0) the 0 axis of a 3D dataObject is shifted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+989"/>
        <location line="+41"/>
        <source>Output object (inplace allowed, but only feasible if source is complex64 or complex128). Destination has the same size than the input object, the type is either complex128 (for float64 or complex128 inputs) or complex64 (else).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-37"/>
        <location line="+41"/>
        <source>Method flag, 0: Estimate (default), 1: Measure.  Measure instructs FFTW to run and measure the execution time of several FFTs in order to find the best way to compute the transform of size n. This process takes some time (usually a few seconds), depending on your machine and on the size of the transform. Estimate, on the contrary, does not run any computation and just builds a reasonable plan that is probably sub-optimal. In short, if your program performs many transforms of the same size and initialization time is not important, use Measure; otherwise use Estimate. </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-35"/>
        <source>Axis over which to compute the FFT. If not given, the last axis is used.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <location line="+38"/>
        <source>Normalization method. no: neither fft nor ifft are scaled, default: direct transform (fft) is not scaled, inverse transform is scaled by 1/n, ortho: both direct and inverse transforms are scaled by 1/sqrt(n)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+497"/>
        <source>1D FFT (via FFTW). Scaled by 1/sqrt(n).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>unscaled 1D FFT (via FFTW)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>inverse 1D FFT (via FFTW). Scaled by 1/n.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>inverse 1D FFT (via FFTW). Scaled by 1/sqrt(n).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>unscaled inverse 1D FFT (via FFTW)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+248"/>
        <source>2D FFT (via FFTW). Scaled by 1/sqrt(n).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>unscaled 2D FFT (via FFTW)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>inverse 2D FFT (via FFTW). Scaled by 1/n.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>inverse 2D FFT (via FFTW). Scaled by 1/sqrt(n).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>unscaled inverse 2D FFT (via FFTW)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+90"/>
        <location line="+419"/>
        <source>Error: one object must be real, the other complex</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-392"/>
        <location line="+417"/>
        <source>Error: size of output object for r2c does not fit</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-219"/>
        <location line="+121"/>
        <source>Error: more than 2 dimensions are not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-112"/>
        <source>FFTW filter forward real to complex (unscaled!)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <location line="+328"/>
        <source>Error: need float64 output for complex input and c2r mode</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-219"/>
        <source>FFTW filter forward complex to real (unscaled!)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+199"/>
        <location line="+73"/>
        <source>Error: this filter is designed for 2D data. As the name says.....</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-64"/>
        <source>FFTW filter 2D real to complex (unscaled!)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+73"/>
        <source>FFTW filter 2D complex to real (unscaled!)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+44"/>
        <source>Source object not defined</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>DataObject is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Input DataObject must be 1xN</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Output object not defined</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Waviness-result must not be equal to roughness output</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Waviness-result must not be equal to surface input</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+68"/>
        <source>Define R_z or 1 pair of lambdas</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+31"/>
        <source>R_z over 200 µm</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+198"/>
        <source>Roughness after gaussian filter with lambdaS %1 mm and lambdaC %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Waviness after gaussian filter with lambdaC %1 mm and lambdaF %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <source>uninitialized vector for mandatory parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>uninitialized vector for optional parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <location line="+2"/>
        <location line="+11"/>
        <location line="+2"/>
        <location line="+2"/>
        <source>see Algorithm-Doc</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-10"/>
        <source>Short wavelength to filter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Wavelength to seperate between roughness and waviness</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Wavelength to seperate between waviness and form</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+36"/>
        <source>Warning: compatibility error between fftw_complex and ito::complex128</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="-3077"/>
        <source>Wrapper for the FFTW</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>This plugin provides wrappers for Fourier Transforms using the FFTW-library. These are for instance:
- 1D FFT (over an arbitrary axis)
- 1D inverse FFT (over an arbitrary axis)
- 2D FFT (over the last two axes)
- 2D inverse FFT (over the last two axes)

The FFTW package was developed at MIT by Matteo Frigo and Steven G. Johnson.It was published unter GNU General Public License and can be downloaded unter http://www.fftw.org/ .

To build this plugin you will need the libs from the fftw.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>GPL (uses FFTW licensed under GPL, too)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+568"/>
        <source>Perform fftshift as known from Python, Matlab and so on, i.e. make the
zero order of diffraction appear in the center.

The shift is implemented along the x and y or one of both axes within each plane (inplace) by using the axis parameter.
The axisIndex parameter is used the shift a &gt;2D dataObject in the 0 axis.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+150"/>
        <source>Perform ifftshift as known from Python, Matlab and so on, i.e. move the
zero order of diffraction back to the corner to run the inverse fft correctly.

The shift is implemented along the x and y or one of both axes within each plane (inplace) by using the axis parameter.
The axisIndex parameter is used the shift a &gt;2D dataObject in the 0 axis.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+377"/>
        <source>Compute the one-dimensional discrete Fourier Transform.

This method computes the one-dimensional n-point discrete Fourier Transform (DFT) with the efficient
Fast Fourier Transform (FFT) algorithm using the fast, GPL licensed library FFTW (fftw.org). The transform
is executed over a desired axis.

This method applies the forward transform, use &apos;ifft&apos; for the inverse transform. The method works both
inplace as well as out-of-place. The output is a complex64 object of the same size than the input object
if the input is of one of the following types: (u)int8, (u)int16, int32, float32 or complex64. If the
input object has one of the types float64 or complex128, the output is complex128. If a type conversion is necessary,
a new dataObject is always put into the destination object.

Meta and axes information are copied to the output object. Only properties of the chosen axis are changed:

* offset: 0.0
* scaling: 1.0 / (previous-scaling * n), the factor 2pi is not considered here
* unit: inverse of previous-unit, e.g. &apos;1/previous-unit&apos;

Per default, no value scaling is applied to the result. However, the optional parameter &apos;norm&apos; influences this.
If &apos;norm&apos; is set to &apos;ortho&apos;, the values are scaled by 1/sqrt(n).

The FFTW library comes with two execution strategies: Measure and Estimate. One of both can be chosen by the optional
parameter &apos;plan_flag&apos;. While Estimate selects a default algorithm, Measure will process some test runs with several
implementations on your machine in order to find out the fastest algorithm for the given type of object. This will
take some time but might speed-up calculations of huge objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+39"/>
        <source>Compute the inverse one-dimensional discrete Fourier Transform.

This method computes the inverse one-dimensional n-point discrete Fourier Transform (DFT) with the efficient
Fast Fourier Transform (FFT) algorithm using the fast, GPL licensed library FFTW (fftw.org). The transform
is executed over a desired axis.

This method applies the inverse transform, use &apos;fft&apos; for the forward transform. The method works both
inplace as well as out-of-place. The output is a complex64 object of the same size than the input object
if the input is of one of the following types: (u)int8, (u)int16, int32, float32 or complex64. If the
input object has one of the types float64 or complex128, the output is complex128. If a type conversion is necessary,
a new dataObject is always put into the destination object.

Meta and axes information are copied to the output object. Only properties of the chosen axis are changed:

* offset: 0.0
* scaling: 1.0 / (previous-scaling * n), the factor 2pi is not considered here
* unit: inverse of previous-unit, e.g. &apos;1/previous-unit&apos;

Per default, the values are scaled by (1/n). However, the optional parameter &apos;norm&apos; influences this.
If &apos;norm&apos; is set to &apos;no&apos;, no scaling is applied, if &apos;norm&apos; is set to &apos;ortho&apos;, the values are scaled by 1/sqrt(n).

The FFTW library comes with two execution strategies: Measure and Estimate. One of both can be chosen by the optional
parameter &apos;plan_flag&apos;. While Estimate selects a default algorithm, Measure will process some test runs with several
implementations on your machine in order to find out the fastest algorithm for the given type of object. This will
take some time but might speed-up calculations of huge objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+40"/>
        <source>Compute the two-dimensional discrete Fourier Transform.

This method computes the two-dimensional n-point discrete Fourier Transform (DFT) with the efficient
Fast Fourier Transform (FFT) algorithm using the fast, GPL licensed library FFTW (fftw.org). The transform
is executed over the last two axes, denoted as planes.

This method applies the forward transform, use &apos;ifft2D&apos; for the inverse transform. The method works both
inplace as well as out-of-place. The output is a complex64 object of the same size than the input object
if the input is of one of the following types: (u)int8, (u)int16, int32, float32 or complex64. If the
input object has one of the types float64 or complex128, the output is complex128. If a type conversion is necessary,
a new dataObject is always put into the destination object.

Meta and axes information are copied to the output object. Only properties of the last two axes are changed:

* offset: 0.0
* scaling: 1.0 / (previous-scaling * n), the factor 2pi is not considered here
* unit: inverse of previous-unit, e.g. &apos;1/previous-unit&apos;

Per default, no value scaling is applied to the result. However, the optional parameter &apos;norm&apos; influences this.
If &apos;norm&apos; is set to &apos;ortho&apos;, the values are scaled by 1/sqrt(n).

The FFTW library comes with two execution strategies: Measure and Estimate. One of both can be chosen by the optional
parameter &apos;plan_flag&apos;. While Estimate selects a default algorithm, Measure will process some test runs with several
implementations on your machine in order to find out the fastest algorithm for the given type of object. This will
take some time but might speed-up calculations of huge objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+39"/>
        <source>Compute the inverse two-dimensional discrete Fourier Transform.

This method computes the inverse two-dimensional n-point discrete Fourier Transform (DFT) with the efficient
Fast Fourier Transform (FFT) algorithm using the fast, GPL licensed library FFTW (fftw.org). The transform
is executed over the last two axes, denoted as planes.

This method applies the inverse transform, use &apos;fft2D&apos; for the forward transform. The method works both
inplace as well as out-of-place. The output is a complex64 object of the same size than the input object
if the input is of one of the following types: (u)int8, (u)int16, int32, float32 or complex64. If the
input object has one of the types float64 or complex128, the output is complex128. If a type conversion is necessary,
a new dataObject is always put into the destination object.

Meta and axes information are copied to the output object. Only properties of the last two axes are changed:

* offset: 0.0
* scaling: 1.0 / (previous-scaling * n), the factor 2pi is not considered here
* unit: inverse of previous-unit, e.g. &apos;1/previous-unit&apos;

Per default, the values are scaled by (1/n). However, the optional parameter &apos;norm&apos; influences this.
If &apos;norm&apos; is set to &apos;no&apos;, no scaling is applied, if &apos;norm&apos; is set to &apos;ortho&apos;, the values are scaled by 1/sqrt(n).

The FFTW library comes with two execution strategies: Measure and Estimate. One of both can be chosen by the optional
parameter &apos;plan_flag&apos;. While Estimate selects a default algorithm, Measure will process some test runs with several
implementations on your machine in order to find out the fastest algorithm for the given type of object. This will
take some time but might speed-up calculations of huge objects.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
