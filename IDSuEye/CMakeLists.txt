set(target_name IDSuEye)


message(STATUS "Project ${target_name} (${CMAKE_CURRENT_BINARY_DIR})")

cmake_minimum_required(VERSION 3.1...3.15)

#################################################################
# Input elements for CMake GUI (Checkboxes, Pathes, Strings...)
#################################################################
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

#this is to automatically detect the SDK subfolder of the itom build directory.
find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if(NOT ITOM_SDK_DIR)
    message(SEND_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

set(IDS_DEVELOP_DIRECTORY "" CACHE PATH "path to develop directory of IDS SDK")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)

include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_plugin_library(${target_name}) #Start the project, init compiler settings and set default configurations for plugins

if(IDS_DEVELOP_DIRECTORY)
    if(WIN32)
        message(STATUS "IDS SDK is located in ${IDS_DEVELOP_DIRECTORY}")
        set(IDS_INCLUDE_DIRECTORY "${IDS_DEVELOP_DIRECTORY}/include" )
        find_file( IDS_HEADER_FILE ueye.h PATHS ${IDS_INCLUDE_DIRECTORY} PATH_SUFFIXES IDS include)
    else(WIN32)
        find_path(IDS_INCLUDE_DIRECTORY
        NAMES ueye.h
        PATHS /usr/local/include/
              /usr/include)
        find_file(IDS_HEADER_FILE ueye.h PATHS ${IDS_INCLUDE_DIRECTORY})
    endif(WIN32)
else(IDS_DEVELOP_DIRECTORY)
    if(WIN32)
        message(STATUS "IDS SDK is located in ${CMAKE_CURRENT_LIST_DIR}/IDSDevLibs")
        set(IDS_INCLUDE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/IDSDevLibs/IDS" )
        find_file( IDS_HEADER_FILE ueye.h PATHS ${IDS_INCLUDE_DIRECTORY} PATH_SUFFIXES "IDS")
    else(WIN32)
        find_path(IDS_INCLUDE_DIRECTORY
        NAMES ueye.h
        PATHS /usr/local/include/
              /usr/include)
        find_file(IDS_HEADER_FILE ueye.h PATHS ${IDS_INCLUDE_DIRECTORY})
    endif(WIN32)
endif(IDS_DEVELOP_DIRECTORY)



if(IDS_HEADER_FILE)

    #################################################################
    # Automatic package detection
    #   add here FIND_PACKAGE commands for searching for 3rd party
    #   libraries
    #
    #   for detecting Qt, use itom_find_package_qt instead of the
    #   native command, since itom_find_package_qt detects the Qt4 or 5
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
    find_package(OpenCV COMPONENTS core REQUIRED) #if you require openCV indicate all components that are required (e.g. core, imgproc...)

    #set( Boost_DEBUG OFF )
    #set( Boost_USE_MULTITHREADED ON )
    #add_definitions( -DBOOST_ALL_NO_LIB )
    #set( Boost_USE_STATIC_LIBS OFF )
    #find_package(Boost REQUIRED COMPONENTS chrono system thread )

    #usage of itom_find_package_qt(automoc component1, component2, ...)
    # automoc is ON or OFF and only relevant for Qt5, usually set it to ON
    # possible components are: OpenGL,Core,Designer,Xml,Svg,Sql,Network,UiTools,Widgets,PrintSupport,LinguistTools...
    itom_find_package_qt(ON Core LinguistTools Widgets)

    if(WIN32)
        set( DELAY_IMP_LIB "delayimp")
        if(IDS_DEVELOP_DIRECTORY)
            if(BUILD_TARGET64)
                find_library(IDS_LIBRARY "ueye_api_64" PATHS ${IDS_DEVELOP_DIRECTORY} PATH_SUFFIXES Lib)
            else(BUILD_TARGET64)
                find_library(IDS_LIBRARY "ueye_api" PATHS ${IDS_DEVELOP_DIRECTORY} PATH_SUFFIXES Lib)
            endif(BUILD_TARGET64)
        else(IDS_DEVELOP_DIRECTORY)
            message(STATUS "IDS SDK is located in ${CMAKE_CURRENT_LIST_DIR}/IDSDevLibs")
            set(IDS_LIB_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/IDSDevLibs" )
            if(BUILD_TARGET64)
                set( IDS_LIBRARY ${IDS_LIB_DIRECTORY}/Lib/uEye_api_64.lib )
            else(BUILD_TARGET64)
                set( IDS_LIBRARY ${IDS_LIB_DIRECTORY}/Lib/uEye_api.lib )
            endif(BUILD_TARGET64)
        endif(IDS_DEVELOP_DIRECTORY)
    else(WIN32)
        set( DELAY_IMP_LIB )
        if(IDS_DEVELOP_DIRECTORY)
        else(IDS_DEVELOP_DIRECTORY)
            if(BUILD_TARGET64)
                FIND_LIBRARY(IDS_LIBRARY NAMES ueye_api64 libueye_api64
                    PATHS /usr/local/lib64/
                          /usr/local/lib/
                          /usr/lib/
                          /usr/lib/x86_64-linux-gnu/
                          /usr/lib64/
                )
            else(BUILD_TARGET64)
                FIND_LIBRARY(IDS_LIBRARY NAMES ueye_api libueye_api
                    PATHS /usr/local/lib/
                          /usr/local/lib64
                          /usr/lib/i386-linux-gnu/
                          /usr/lib/
                          /usr/lib64/
                )
            endif(BUILD_TARGET64)
        endif(IDS_DEVELOP_DIRECTORY)
    endif(WIN32)

    #################################################################
    # General settings and preprocessor settings
    #################################################################
    



    #################################################################
    # List of include directories
    #
    # Hint: necessary Qt include directories are automatically added
    #  via the FIND_PACKAGE macro above
    #################################################################
    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR} #build directory of this plugin (recommended)
        ${CMAKE_CURRENT_SOURCE_DIR} #source directory of this plugin (recommended)
        ${ITOM_SDK_INCLUDE_DIRS}    #include directory of the itom SDK (recommended) 
        #add further include directories here
        ${Tools_SOURCE_DIR}
        ${IDS_INCLUDE_DIRECTORY}
    )

    #################################################################
    # List of linker directories
    #
    # Hint: libraries detected using FIND_PACKAGE usually provide
    #  all necessary libraries in a specific variable (e.g.
    #  ${OpenCV_LIBS} or ${ITOM_SDK_LIBRARIES}). These variables
    #  already contain absolute pathes, therefore no link directory
    #  needs to be set for them. Simply add these variables to
    #  the link target command below.
    #################################################################
    link_directories(
        #add all linker directories
    )

    #################################################################
    # List of header files, source files, ui files and rcc files
    #
    # Add all header files to the PLUGIN_HEADERS list.
    # Add all source (cpp,...) files to the PLUGIN_SOURCES list.
    # Add all ui-files (Qt-Designer layouts) to the PLUGIN_UI list.
    #
    # Use absolute pathes, e.g. using one of the following variables:
    #
    # ${ITOM_SDK_INCLUDE_DIR} is the include directory of itom SDK
    # ${CMAKE_CURRENT_SOURCE_DIR} is the source directory of this plugin
    # ${CMAKE_CURRENT_BINARY_DIR} is the build directory of this plugin
    #
    #################################################################
    set(PLUGIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/DialogIDS.h
        ${CMAKE_CURRENT_SOURCE_DIR}/DockWidgetIDS.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IDSuEye.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IDSInterface.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
        ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        #add further header files (absolute pathes e.g. using CMAKE_CURRENT_SOURCE_DIR)
    )

    set(PLUGIN_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/DialogIDS.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DockWidgetIDS.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IDSuEye.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IDSInterface.cpp
        #add further source files here
    )

    #Append rc file to the source files for adding information about the plugin
    # to the properties of the DLL under Visual Studio.
    if(MSVC)
        list(APPEND PLUGIN_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
    endif()

    set(PLUGIN_UI
        ${CMAKE_CURRENT_SOURCE_DIR}/dialogIDS.ui
        ${CMAKE_CURRENT_SOURCE_DIR}/dockWidgetIDS.ui
    )

    set(PLUGIN_RCC
        #add absolute pathes to any *.qrc resource files here
    )



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


    #add all (generated) header and source files to the library (these files are compiled then)
    add_library(${target_name} SHARED ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI} ${PLUGIN_RCC})

    #link the compiled library
    #append all libraries this plugin should be linked to at the end of the target_link_libraries command
    # Important variables are:
    # ${ITOM_SDK_LIBRARIES} -> all necessary libraries from find_package(ITOM_SDK)
    # -> all necessary libraries from itom_find_package_qt (Qt4 or Qt5)
    # ${OpenCV_LIBS} -> all necessary libraries opencv libraries from find_package(OpenCV)
    #
    # if you want to link against one library whose directory is already added to link_directories above
    # simply add its filename without suffix (*.lib, *.so...). This is automatically done by CMake
        
    # Qt: enable all automoc, autouic and autorcc.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    target_link_libraries(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} ${IDS_LIBRARY} ${DELAY_IMP_LIB})



    # set_target_properties(${target_name} PROPERTIES LINK_FLAGS "/DELAYLOAD:SC2_Cam.dll")

    #################################################################
    # Plugin Translation
    # 
    # In the source directory of the plugin can be a subfolder 'docs'.
    # This folder can contain one or more *.rst files with the docu-
    # mentation of the plugin. CMake organizes the rest if you 
    # indicate the name of the main documentation file(without
    # suffix rst) in the following command:
    # 
    # itom_configure_plugin_documentation(${target_name} nameOfTheFile)
    #################################################################
    set(FILES_TO_TRANSLATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS} ${PLUGIN_UI})
    itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

    #################################################################
    # Plugin Documentation
    # 
    # In the source directory of the plugin can be a subfolder 'docs'.
    # This folder can contain one or more *.rst files with the docu-
    # mentation of the plugin. CMake organizes the rest if you 
    # indicate the name of the main documentation file(without
    # suffix rst) in the following command:
    # 
    # itom_configure_plugin_documentation(${target_name} nameOfTheFile)
    #################################################################
    itom_configure_plugin_documentation(${target_name} IDSuEyeCamera)

    #################################################################
    # Post-Build Copy Operations
    # 
    # itom is able to force a post-build process that copies
    # different files, like the currently created library, to
    # other destination pathes. This is done in this section.
    # At first pairs of sources and destinations are added
    # to the lists COPY_SOURCES and COPY_DESTINATIONS.
    # Afterwards, the post-build process is generated using
    # itom_post_build_copy_files.
    #
    # The following macros can be used to fill up the source
    # and destination list:
    #
    # itom_add_pluginlibrary_to_copy_list 
    # - this is necessary for each plugin such that the library
    #   is automatically copied to the plugins folder of
    #   the itom build directory.
    #
    # itom_add_plugin_qm_files_to_copy_list
    # - installs the generated translation files (qm) at the
    #   right place in the itom build directory as well.
    #
    #################################################################
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")

    itom_add_pluginlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_plugin_qm_files_to_copy_list(${target_name} QM_FILES COPY_SOURCES COPY_DESTINATIONS)

    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

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
    # itom_post_build_copy_files_to_lib_folder(${target}, ${listOfFiles})
    # copy SC2_Cam.dll in itoms lib folder
    #itom_post_build_copy_files_to_lib_folder(${target_name} FILES_TO_COPY_TO_LIB)

else(IDS_HEADER_FILE)
    message(WARNING "ueye.h for plugin ${target_name} could not be found. ${target_name} will not be build.")
endif(IDS_HEADER_FILE)
