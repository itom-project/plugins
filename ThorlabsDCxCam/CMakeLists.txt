SET (target_name ThorlabsDCxCam)

project(${target_name})

message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

cmake_minimum_required(VERSION 2.8)

#################################################################
# Input elements for CMake GUI (Checkboxes, Pathes, Strings...)
#################################################################
OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")
SET (THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY "" CACHE PATH "path to develop directory of Thorlabs / Scientific Imaging / DCx Camera Support")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY)
    IF(WIN32)
        find_path( THORLABS_DCxCAMERA_INCLUDE_DIR uc480.h PATHS ${THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY} PATH_SUFFIXES Include)
        
        IF(BUILD_TARGET64)
            find_file( THORLABS_DCxCAMERA_LIBRARY uc480_64.lib PATHS ${THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY} PATH_SUFFIXES Lib)
        ELSE(BUILD_TARGET64)
            find_file( THORLABS_DCxCAMERA_LIBRARY uc480.lib PATHS ${THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY} PATH_SUFFIXES Lib)
        ENDIF(BUILD_TARGET64)
    ENDIF(WIN32)
ENDIF(THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY)



IF (THORLABS_DCxCAMERA_INCLUDE_DIR)

    #################################################################
    # Automatic package detection
    #   add here find_package commands for searching for 3rd party
    #   libraries
    #
    #   for detecting Qt, use FIND_PACKAGE_QT instead of the
    #   native command, since FIND_PACKAGE_QT detects the Qt4 or 5
    #   version.
    #################################################################

    # the itom SDK needs to be detected, use the COMPONENTS keyword
    # to define which library components are needed. Possible values
    # are:
    # - dataobject for the ito::DataObject (usually required)
    # - itomCommonLib (RetVal, Param,...) (required)
    # - itomCommonQtLib (AddInInterface,...) (required)
    # - itomWidgets (further widgets) (may be used in dock widget...)
    # - pointcloud (pointCloud, point, polygonMesh...) (optional)
    # if no components are indicated, all components above are used
    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
    include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake") #include this mandatory macro file (important)
    find_package(OpenCV COMPONENTS core REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...)
    find_package(VisualLeakDetector QUIET) #silently detects the VisualLeakDetector for Windows (memory leak detector, optional)

    #set( Boost_DEBUG OFF )
    #set( Boost_USE_MULTITHREADED ON )
    #add_definitions( -DBOOST_ALL_NO_LIB )
    #set( Boost_USE_STATIC_LIBS OFF )
    #find_package(Boost REQUIRED COMPONENTS chrono system thread )

    #usage of FIND_PACKAGE_QT(automoc component1, component2, ...)
    # automoc is ON or OFF and only relevant for Qt5, usually set it to ON
    # possible components are: OpenGL,Core,Designer,Xml,Svg,Sql,Network,UiTools,Widgets,PrintSupport,LinguistTools...
    FIND_PACKAGE_QT(ON Core LinguistTools Widgets)

    #################################################################
    # General settings and preprocessor settings
    #################################################################
    IF(BUILD_SHARED_LIBS)
        SET(LIBRARY_TYPE SHARED)
    ELSE(BUILD_SHARED_LIBS)
        SET(LIBRARY_TYPE STATIC)
    ENDIF(BUILD_SHARED_LIBS)

    IF (BUILD_UNICODE)
        ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
    ENDIF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DCMAKE)
    ADD_DEFINITIONS(-DITOMWIDGETS_SHARED)

    IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
    ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

    # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
    IF (DEFINED CMAKE_BUILD_TYPE)
        SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    ELSE(CMAKE_BUILD_TYPE)
        SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    ENDIF (DEFINED CMAKE_BUILD_TYPE)


    #################################################################
    # List of include directories
    #
    # Hint: necessary Qt include directories are automatically added
    #  via the find_package macro above
    #################################################################
    INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${OpenCV_DIR}/include       #include directory of OpenCV
        ${ITOM_SDK_INCLUDE_DIR}     #include directory of the itom SDK (recommended) 
        ${ITOM_SDK_INCLUDE_DIR}/itomWidgets #include this subfolder if you use any widgets from itomWidgets
        ${VISUALLEAKDETECTOR_INCLUDE_DIR} #include directory to the visual leak detector (recommended, does nothing if not available)
        #add further include directories here
        ${Tools_SOURCE_DIR}
        ${THORLABS_DCxCAMERA_INCLUDE_DIR}
    )

    #################################################################
    # List of linker directories
    #
    # Hint: libraries detected using find_package usually provide
    #  all necessary libraries in a specific variable (e.g.
    #  ${OpenCV_LIBS} or ${ITOM_SDK_LIBRARIES}). These variables
    #  already contain absolute pathes, therefore no link directory
    #  needs to be set for them. Simply add these variables to
    #  the link target command below.
    #################################################################
    LINK_DIRECTORIES(
        #add all linker directories
    )

    #################################################################
    # List of header files, source files, ui files and rcc files
    #
    # Add all header files to the plugin_HEADERS list.
    # Add all source (cpp,...) files to the plugin_SOURCES list.
    # Add all ui-files (Qt-Designer layouts) to the plugin_UI list.
    #
    # Use absolute pathes, e.g. using one of the following variables:
    #
    # ${ITOM_SDK_INCLUDE_DIR} is the include directory of itom SDK
    # ${CMAKE_CURRENT_SOURCE_DIR} is the source directory of this plugin
    # ${CMAKE_CURRENT_BINARY_DIR} is the build directory of this plugin
    #
    #################################################################
    set(plugin_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDCxCam.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDCxCam.h
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsDCxCam.h
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsDCxCamInterface.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(plugin_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDCxCam.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDCxCam.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsDCxCam.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/thorlabsDCxCamInterface.cpp
        #add further source files here
    )

    #Append rc file to the source files for adding information about the plugin
    # to the properties of the DLL under Visual Studio.
    if(MSVC)
        list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif(MSVC)

    set(plugin_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogThorlabsDCxCam.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetThorlabsDCxCam.ui
    )

    set(plugin_RCC
        #add absolute pathes to any *.qrc resource files here
    )

    #################################################################
    # Qt related pre-processing of the files above
    # (These methods create the moc, rcc and uic process.)
    #################################################################
    if (QT5_FOUND)
        #if automoc if OFF, you also need to call QT5_WRAP_CPP here
        QT5_WRAP_UI(plugin_UI_MOC ${plugin_UI})
        QT5_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
    else (QT5_FOUND)
        QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
        QT4_WRAP_UI_ITOM(plugin_UI_MOC ${plugin_UI})
        QT4_ADD_RESOURCES(plugin_RCC_MOC ${plugin_RCC})
    endif (QT5_FOUND)


    #################################################################
    # Group files in their original folder structure (MSVC only)
    # If you have some header and source files in a specific
    # subfolder, you can even have this subfolder in your
    # IDE (mainly Visual Studio supports this). Then call 
    # ADD_SOURCE_GROUP(directoryName) for each subdirectory.
    #
    # HINT: This command does nothing for IDE different than MSVC.
    #################################################################
    #ADD_SOURCE_GROUP(subdirectory)


    #################################################################
    # Compile and link the plugin library
    # 
    #################################################################

    #search for all existing translation files in the translation subfolder
    file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts") 

    #add all (generated) header and source files to the library (these files are compiled then)
    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_UI_MOC} ${plugin_RCC_MOC} ${EXISTING_TRANSLATION_FILES})

    #link the compiled library
    #append all libraries this plugin should be linked to at the end of the TARGET_LINK_LIBRARIES command
    # Important variables are:
    # ${ITOM_SDK_LIBRARIES} -> all necessary libraries from find_package(ITOM_SDK)
    # ${QT_LIBRARIES} -> all necessary libraries from FIND_PACKAGE_QT (Qt4 or Qt5)
    # ${OpenCV_LIBS} -> all necessary libraries opencv libraries from find_package(OpenCV)
    #
    # if you want to link against one library whose directory is already added to LINK_DIRECTORIES above
    # simply add its filename without suffix (*.lib, *.so...). This is automatically done by CMake

    TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${THORLABS_DCxCAMERA_LIBRARY} delayimp)
    if (QT5_FOUND)
        qt5_use_modules(${target_name} ${QT_COMPONENTS}) #special command for Qt5
    endif (QT5_FOUND)

    # SET_TARGET_PROPERTIES(${target_name} PROPERTIES LINK_FLAGS "/DELAYLOAD:SC2_Cam.dll")

    #################################################################
    # Plugin Translation
    # 
    # In the source directory of the plugin can be a subfolder 'docs'.
    # This folder can contain one or more *.rst files with the docu-
    # mentation of the plugin. CMake organizes the rest if you 
    # indicate the name of the main documentation file (without
    # suffix rst) in the following command:
    # 
    # PLUGIN_DOCUMENTATION(${target_name} nameOfTheFile)
    #################################################################
    set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
    PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

    #################################################################
    # Plugin Documentation
    # 
    # In the source directory of the plugin can be a subfolder 'docs'.
    # This folder can contain one or more *.rst files with the docu-
    # mentation of the plugin. CMake organizes the rest if you 
    # indicate the name of the main documentation file (without
    # suffix rst) in the following command:
    # 
    # PLUGIN_DOCUMENTATION(${target_name} nameOfTheFile)
    #################################################################
    PLUGIN_DOCUMENTATION(${target_name} ThorlabsDCxCam)

    #################################################################
    # Post-Build Copy Operations
    # 
    # itom is able to force a post-build process that copies
    # different files, like the currently created library, to
    # other destination pathes. This is done in this section.
    # At first pairs of sources and destinations are added
    # to the lists COPY_SOURCES and COPY_DESTINATIONS.
    # Afterwards, the post-build process is generated using
    # POST_BUILD_COPY_FILES.
    #
    # The following macros can be used to fill up the source
    # and destination list:
    #
    # ADD_PLUGINLIBRARY_TO_COPY_LIST 
    # - this is necessary for each plugin such that the library
    #   is automatically copied to the plugins folder of
    #   the itom build directory.
    #
    # ADD_QM_FILES_TO_COPY_LIST
    # - installs the generated translation files (qm) at the
    #   right place in the itom build directory as well.
    #
    #################################################################
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    ADD_PLUGINLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    ADD_QM_FILES_TO_COPY_LIST(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    #set( BOOST_FILES_TO_COPY  ${Boost_CHRONO_LIBRARY_DEBUG} ${Boost_CHRONO_LIBRARY_RELEASE} 
    #                          ${Boost_SYSTEM_LIBRARY_DEBUG} ${Boost_SYSTEM_LIBRARY_RELEASE} 
    #                          ${Boost_THREAD_LIBRARY_DEBUG} ${Boost_THREAD_LIBRARY_RELEASE} )

                              
    #foreach (file ${BOOST_FILES_TO_COPY})
    #  get_filename_component(LIBNAME "${file}" NAME_WE)
    #  get_filename_component(LIBPATH "${file}" DIRECTORY )
    #  list( APPEND FILES_TO_COPY_TO_LIB "${LIBPATH}/${LIBNAME}.dll" )
    #endforeach (file ${BOOST_FILES_TO_COPY})

    #if you want to copy one or more files to the lib-folder of
    # the itom build directory, use the following macro:
    #
    # POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target}, ${listOfFiles})
    # copy SC2_Cam.dll in itoms lib folder
    #POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} FILES_TO_COPY_TO_LIB)

ELSE(THORLABS_DCxCAMERA_INCLUDE_DIR)
    message(WARNING "uc480.h for plugin ${target_name} could not be found. ${target_name} will not be build. Please properly indicate THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY.")
ENDIF(THORLABS_DCxCAMERA_INCLUDE_DIR)