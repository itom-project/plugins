Livox SDK

* Download from: https://github.com/Livox-SDK/Livox-SDK.git
* CMake and Compile
* ...

Help/Docu:
https://livox-sdk.github.io/Livox-SDK-Doc/



SDK Folder structure (as of Feb'21)
-GITRepo-SDK
	-build
		-CMake
		-samples
		-sdk_core
			-Debug
			-Release
				--> .lib
		-x64
	-doc
	-samples
	-sdk_core
		-include
			-comm
			-third_party
			config.h
			livox_def.h
			livox_sdk.h
		-source
			-base
			-comm
			-command_handler
			-data_handler
			-third_party
			device_discovery.cpp /-.h
			device_manager.cpp /-.h
			livox_sdk.cpp
		CMAKELists.txt
	CMakeLists.txt
	License.txt
	README.txt