<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>QObject</name>
    <message>
        <location filename="../roughness.cpp" line="+61"/>
        <source>Algorithms for roughness evaluation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>This plugin contains algorithms for the 1D roughness evaluation.

The contained algorithms are:

* calcRoughnessProfile: divide the given profile row-by-row into roughness and waviness
* evalRoughnessProfile: evaluate either the roughness or waviness based on various 1D roughness parameters (e.g. Rz, Ra...)
* roughnessProfile: combination of the two algorithms above as well as a possible initial subtraction of a regression line
* calcAbbottCurve: determination of the abbott curve based on the roughness or waviness

Some algorithms the plugin &apos;fittingFilters&apos; for a valid execution.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+67"/>
        <source>the chosen sampling length for five sections differs from the intended length (cut-off wavelength)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>measurement length is too short to be split into 5 samples with a length of the cut-off wavelength each</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>wrong sampling_length_mode</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>measurement length is too short</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+446"/>
        <location line="+19"/>
        <source>profile contains at least one sample length with only zero values -&gt; Zq == 0 -&gt; Zsk not determinable</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+103"/>
        <location line="+22"/>
        <source>profile contains at least one sample length with only zero values -&gt; Zq == 0 -&gt; Zku not determinable</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+68"/>
        <source>Rdq or Wdq require a sample length of at least 7 samples.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+49"/>
        <location line="+90"/>
        <source>at least one sample does not contain at least 7 adjacent valid data points</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-48"/>
        <source>Rda or Wda require a sample length of at least 7 samples.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+217"/>
        <source>returns the gaussian filter kernel as used by calcRoughnessProfile, based on EN ISO 11562</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>kernel data object that contains the kernel after the call.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>cut-off wavelength in _m</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>spacing in _m between two adjacent pixels</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>quality factor for the length of the filter kernel. The kernel is set to zero for indices &gt; cutoff-wavelength * cutoff_factor. See ISO 16610-21:2013 A.5: 0.5 for general purpose, high precision: 0.6, reference software: 1.0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>kernel has a size of 0. Invalid.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+28"/>
        <source>calculate the roughness and waviness profile based in a given real input data object.

The roughness and waviness profile is determined row-by-row from the given input object that is filtered using
one or two gaussian convolution operations. The gaussian filters are chosen such that a transmission of 50% is
obtained at the given cut-off wavelength levels.

The waviness is a lowpass of the input data cut at the cut-off wavelength Lc.
The roughness is the obtained by the difference between input and waviness. Additionally a lowpass is applied
at the cut-off wavelength Ls (if Ls &gt; 0.0).

You need to define if your profile is an open, non-periodic profiel or a closed, periodic profile. In the latter case
the evaluation is only implemented by a convolution based algorithm, in the first case one can choose between the fast
dft-implementation or the convolution-based implementation. If &apos;mode&apos; is set to &apos;auto&apos;, lowpass filter operations with
big wavelengths are evaluated using the dft-approach (Lc) while the high-frequency cut-off Ls is done using the convolution operation.

In order to guarantee an efficient algorithm, the gaussian filter kernel will be cut after a certain length. This length can be
controlled using the parameter &apos;cutoff_factor&apos;.

This filter is implemented based on DIN EN ISO 16610-21:2013.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+245"/>
        <source>Evaluates given roughness or waviness profiles for a specific roughness profile parameter.

The given roughness data is evaluated line by line. If only one line is evaluated, &apos;result&apos; contains:

(mean-value-over-all-samples, min-value-over-all-samples, max-value-over-all-samples)

The result values are always in _m. In case of Rt (or Wt), the evaluation is not separated to various samples, therefore mean, min and max contain the same values.

If multiple lines are evaluated, the result contains the

(mean-value-over-all-lines, min-value, max-value, std-dev),
where min-value, max-value and std-dev are calculated over the mean-values of all lines.

The evaluation is done based on DIN EN ISO 4287:2010, the separation of different sample lengths is based on DIN EN ISO 4288:1997.

Possible roughness parameters are Rp, Rv, Rz, Rt, Ra, Rq, Rsk, Rku, Rdq, Rda, Rdc. If you pass the waviness profile instead of the roughness profile
the parameters are then Wp, Wv, Wz, Wt, Wa, Wq, Wsk, Wku, Wdq, Wda, Wdc.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+316"/>
        <source>Calculate a profile roughness parameter for each line in the given input object.

The evaluation is done based on DIN EN ISO 4287:2010, the separation of different sample lengths is based on DIN EN ISO 4288:1997.

This filter is a thre-step filter, based on the single filters &apos;subtract1DRegression&apos; from the plugin &apos;FittingFilters&apos;
and &apos;calcRoughnessProfile&apos; as well as &apos;evalRoughnessProfile&apos;. The first is an optional filter to remove the form of the raw signal.
The second filter splits the raw signal into the waviness and roughness signal (given by the cut-off wavelengths Lc and Ls).
The last filter evaluates the given roughness parameter and returns the result. For more information see the description of the single filters.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+139"/>
        <source>This filter calculates the Abbott firestone curve (as well as an optional histogram) of
a roughness or waviness profile, e.g. obtained from the filter &apos;calcRoughnessProfile&apos;.

The evaluation is done based on DIN EN ISO 4287:2010.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-670"/>
        <location line="+547"/>
        <source>real input object (1D or 2D). The roughness is determined row-by-row. The axis units must be set to &apos;mm&apos;, &apos;_m&apos; or &apos;nm&apos;.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-546"/>
        <source>roughness output object of the same size than the input object. Type is float64.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>waviness output object of the same size than the input object. Type is float64.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+571"/>
        <source>cut-off wavelength in _m for the separation between the waviness and roughness</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-570"/>
        <location line="+571"/>
        <source>short cut-off wavelength in _m for the separation between roughness and further high-frequency components (set 0.0 to omit this filtering)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-569"/>
        <source>mode of determination: convolution, dft or auto (auto-selection of both methods)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <location line="+565"/>
        <source>0: non periodic profile (default), 1: periodic profile (not implemented yet)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-564"/>
        <location line="+565"/>
        <source>quality factor for the convolution based calculation. The convolution kernel is set to zero for indices &gt; cutoff-wavelength * cutoff_factor. See ISO 16610-21:2013 A.5: 0.5 for general purpose, high precision: 0.6, reference software: 1.0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-563"/>
        <location line="+264"/>
        <location line="+405"/>
        <source>nr of pixels at both sides of the profile that contain wrong values to the final length of the convolution filter kernel. Skip these pixels in any evaluation (see DIN EN ISO 16610-21, 4.3; DIN EN ISO 16610-28 will show more information when it is released in a final version)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-648"/>
        <source>empty input object given</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <location line="+311"/>
        <source>axis unit could not be read</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-296"/>
        <location line="+311"/>
        <source>axis unit of input must be mm, _m or nm</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-231"/>
        <source>periodic, closed profiles cannot be evaluated using dft (not implemented yet)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>profile is too long. Frequency analysis cannot be executed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+41"/>
        <source>unsupported mode</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+46"/>
        <location line="+442"/>
        <source>real input object (1D or 2D, no unsigned data types) - must be either the output argument &apos;roughness&apos; or &apos;waviness&apos; from filter &apos;calcRoughnessProfile&apos;. The roughness is determined row-by-row. The axis units must be set to &apos;mm&apos;, &apos;_m&apos; or &apos;nm&apos;.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-441"/>
        <source>roughness or waviness parameter to determine</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+37"/>
        <source>cut-off wavelength in _m used for the separation between the waviness and roughness</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <location line="+299"/>
        <source>mode how to split the measurement length (ml) into different sampling lengths (sl): 0: split ml into five samples (warn if the sl does not correspond to Lc), 1: split ml into n samples whose length is Lc, 2: same as 1 but only use the first 5 sampling lengths.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-298"/>
        <source>returns a Nx3 float64 data object will the mean,min and max roughness value for each evaluated line (only interesting if input contains more than one line)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <location line="+298"/>
        <source>[min,max] range in percent for the parameter Rdc or Wdc (height difference between two levels of the Abbott-Firstone-Curve</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-294"/>
        <location line="+298"/>
        <source>resulting values. See docstring of filter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-297"/>
        <location line="+298"/>
        <source>Number of evaluated samples per line (5 is the desired value with respect to DIN EN ISO 4288), Rt is evaluated over the whole measurement range, therefore nr_of_samples is 1.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-239"/>
        <source>unknown param name</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>value unit of input should be mm, _m or nm. Else the right scaling of the resulting values (in _m) cannot be guaranteed.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+155"/>
        <source>roughness parameter to determine</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+28"/>
        <source>0: no form is subtracted from input as first step, else: a polynomial form of given order (1: line) is fitted and removed as first step (requires filter plugin &apos;FittingFilters&apos;)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+106"/>
        <source>abbott firestone curve with a horizontal step width of 1% (float64, size: Mx100 for M rows in the input object.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>optional histogram with 100 buckets (int32, 1x100). Only the first line is evaluated.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+179"/>
        <location line="+87"/>
        <location line="+70"/>
        <source>given gauss kernel too short</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-66"/>
        <location line="+70"/>
        <source>the given data must have at least %i values</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>Roughness</name>
    <message>
        <location line="-828"/>
        <source>Roughness filtered with L_c = %1 and L_s = %2, mode = %3</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Waviness filtered with L_c = %1 and L_s = %2, mode = %3</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
