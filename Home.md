# Plugins for *itom*

This repository contains freely available plugins for [itom](https://github.com/itom-project).

Currently there are plugins available like:

* CMU 1394 cameras
* Dummy camera
* cameras supported by OpenCV
* SVS Vistek GigE cameras
* PI Piezo Controllers
* Serial port communication
* GWInstek Power Supply communication plugin
* Dummy motor
* Import/Export methods for data objects to many image formats

Further plugins will be released open source in the future.

## Compile plugins

For compiling these plugins, follow these steps:

1. Clone this repository on your hard drive
2. Make sure that you have a valid SDK of itom available on your computer. This SDK is either included in any itom setup (Release only) or you need to clone and compile itom both in debug and release configuration (recommended). See the itom documentation for more information about compiling itom.
3. Run CMake and set the variable **ITOM_SDK_DIR** or the environment variable **ITOM_SDK_ROOT** to the itom's SDK directory. After reconfiguring the variable ITOM_DIR should be automatically set. When setting the path of OpenCV, you normally should indicate the build-folder of a prebuild OpenCV version in Windows.
4. Start the compilation

For detailed information about the compile process, one is also referred to the documentation of itom.

## Notice

If you compiled any plugin however it cannot be started in itom with the error message that any module could not be loaded, it is common that this plugin requires further drivers or libraries that can currently not be found on your computer. Then, install the necessary drivers first and make sure that the necessary libraries are in a searchable path or the lib-folder of itom.
