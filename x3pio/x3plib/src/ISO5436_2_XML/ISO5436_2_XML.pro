#-------------------------------------------------
#
# Project created by QtCreator 2012-03-23T13:23:34
#
#-------------------------------------------------
# load constants
include( ../../../../../itom_properties.pri )

QT       -= core gui

TARGET = iso5436_2
TEMPLATE = lib

CONFIG(debug,debug|release) {
    DESTDIR = ./../../../../../debug/lib
    OBJECTS_DIR += debug
}
CONFIG(release,debug|release) {
    DESTDIR = ./../../../../../release/lib
    OBJECTS_DIR += release
}


win32 {
    versiontarget.target = version.h
    versiontarget.commands = $${ITOM_SVNPATH}/subwcrev $$PWD $$PWD/cxx/version.in $$PWD/../../include/opengps/cxx/version.h
    versiontarget.depends = FORCE

    PRE_TARGETDEPS += version.h
    QMAKE_EXTRA_TARGETS += versiontarget

    schematarget.target = iso5436_2_xsd.cxx
    schematarget.commands = $${ITOM_XSDPATH}/bin/xsd cxx-tree --prologue-file $$PWD/xsd_Licence_Header.c --generate-doxygen --generate-ostream --generate-serialization --char-type wchar_t --generate-comparison --generate-from-base-ctor --namespace-map http://www.opengps.eu/2008/ISO5436_2=OpenGPS::Schemas::ISO5436_2 --export-symbol _OPENGPS_EXPORT --cxx-suffix \"_xsd.cxx\" --hxx-suffix \"_xsd.hxx\" --output-dir $$PWD/../../include/opengps/cxx/ $$PWD/iso5436_2.xsd
    schematarget.depends = FORCE

    PRE_TARGETDEPS += iso5436_2_xsd.cxx
    QMAKE_EXTRA_TARGETS += schematarget
}
else {
    versiontarget.target = version.h
    versiontarget.commands = $$PWD/../tools/subwcrev.sh $$PWD $$PWD/cxx/version.in $$PWD/../../include/opengps/cxx/version.h
    versiontarget.depends = FORCE

    PRE_TARGETDEPS += version.h
    QMAKE_EXTRA_TARGETS += versiontarget

    schematarget.target = iso5436_2_xsd.cxx
    schematarget.commands = xsdcxx cxx-tree --prologue-file $$PWD/xsd_Licence_Header.c --generate-doxygen --generate-ostream --generate-serialization --char-type wchar_t --generate-comparison --generate-from-base-ctor --namespace-map http://www.opengps.eu/2008/ISO5436_2=OpenGPS::Schemas::ISO5436_2 --export-symbol _OPENGPS_EXPORT --cxx-suffix \"_xsd.cxx\" --hxx-suffix \"_xsd.hxx\" --output-dir $$PWD/../../include/opengps/cxx/ $$PWD/iso5436_2.xsd
    schematarget.depends = FORCE

    PRE_TARGETDEPS += iso5436_2_xsd.cxx
    QMAKE_EXTRA_TARGETS += schematarget

    isoxsd.target = iso5436_2.xsd
    isoxsd.commands = cp $$PWD/iso5436_2.xsd $$DESTDIR
    isoxsd.depends = FORCE

    PRE_TARGETDEPS += iso5436_2.xsd
    QMAKE_EXTRA_TARGETS += isoxsd

    OLDDIR = $$PWD
    libminizip.target = libminizip.so
    libminizip.commands = $$PWD/../zlib/contrib/minizip/buildLibMinizip.sh $$PWD/../zlib/contrib/minizip $${OUT_PWD}/$${DESTDIR}
    libminizip.depends = FORCE

    PRE_TARGETDEPS += libminizip.so
    QMAKE_EXTRA_TARGETS += libminizip
}

DEFINES += ISO5436_2_LIBRARY \
        UNICODE \
        _UNICODE

win32 {
    DEFINES += _WINDOWS QT_LARGEFILE_SUPPORT QT_DLL
}

INCLUDEPATH +=  . \
                ./.. \
                ../../include \
                ../../include/opengps \
                ../../include/opengps/cxx \
                ../zlib \
                ../zlib/contrib/minizip

win32 {
    INCLUDEPATH += $${ITOM_XSDPATH}/include/
}
else {
    INCLUDEPATH += /usr/include
}

#QMAKE_LFLAGS += -Wl,-R./ -municode
QMAKE_LFLAGS += -Wl,-R./:./lib/:../lib/
linux {
    QMAKE_CFLAGS += -fopenmp -fexceptions
    QMAKE_CXXFLAGS += -fopenmp -fexceptions
}

HEADERS += ./iso5436_2.h \
        ./c/data_point_c.hxx \
        ./c/iso5436_2_handle_c.hxx \
        ./c/messages_c.hxx \
        ./c/point_iterator_c.hxx \
        ./c/point_vector_c.hxx \
        ./cxx/binary_lsb_point_vector_reader_context.hxx \
        ./cxx/binary_lsb_point_vector_writer_context.hxx \
        ./cxx/binary_msb_point_vector_reader_context.hxx \
        ./cxx/binary_msb_point_vector_writer_context.hxx \
        ./cxx/binary_point_vector_reader_context.hxx \
        ./cxx/binary_point_vector_writer_context.hxx \
        ./cxx/data_point_impl.hxx \
        ./cxx/data_point_parser.hxx \
        ./cxx/double_data_point_parser.hxx \
        ./cxx/double_point_buffer.hxx \
        ./cxx/environment.hxx \
        ./cxx/float_data_point_parser.hxx \
        ./cxx/float_point_buffer.hxx \
        ./cxx/inline_validity.hxx \
        ./cxx/int16_data_point_parser.hxx \
        ./cxx/int16_point_buffer.hxx \
        ./cxx/int32_data_point_parser.hxx \
        ./cxx/int32_point_buffer.hxx \
        ./cxx/iso5436_2_container.hxx \
        ./cxx/missing_data_point_parser.hxx \
        ./cxx/point_buffer.hxx \
        ./cxx/point_validity_provider.hxx \
        ./cxx/point_vector_iostream.hxx \
        ./cxx/point_vector_parser.hxx \
        ./cxx/point_vector_parser_builder.hxx \
        ./cxx/point_vector_proxy.hxx \
        ./cxx/point_vector_proxy_context.hxx \
        ./cxx/point_vector_proxy_context_list.hxx \
        ./cxx/point_vector_proxy_context_matrix.hxx \
        ./cxx/point_vector_reader_context.hxx \
        ./cxx/point_vector_writer_context.hxx \
        ./cxx/stdafx.hxx \
        ./cxx/valid_buffer.hxx \
        ./cxx/vector_buffer.hxx \
        ./cxx/vector_buffer_builder.hxx \
        ./cxx/win32_environment.hxx \
        ./cxx/linux_environment.hxx \
        ./cxx/xml_point_vector_reader_context.hxx \
        ./cxx/xml_point_vector_writer_context.hxx \
        ./cxx/zip_stream_buffer.hxx \
        ../../include/opengps/data_point.h \
        ../../include/opengps/data_point_type.h \
        ../../include/opengps/info.h \
        ../../include/opengps/iso5436_2.h \
        ../../include/opengps/messages.h \
        ../../include/opengps/opengps.h \
        ../../include/opengps/point_iterator.h \
        ../../include/opengps/point_vector.h \
        ../../include/opengps/cxx/data_point.hxx \
        ../../include/opengps/cxx/exceptions.hxx \
        ../../include/opengps/cxx/info.hxx \
        ../../include/opengps/cxx/iso5436_2.hxx \
        ../../include/opengps/cxx/iso5436_2_handle.hxx \
        ../../include/opengps/cxx/iso5436_2_xsd.hxx \
        ../../include/opengps/cxx/opengps.hxx \
        ../../include/opengps/cxx/point_vector.hxx \
        ../../include/opengps/cxx/point_vector_base.hxx \
        ../../include/opengps/cxx/string.hxx \
        ../../include/opengps/cxx/version.h \
        ./xyssl/md5.h

SOURCES +=  ./xsd_Licence_Header.c \
        ./c/data_point_c.cxx \
        ./c/iso5436_2_c.cxx \
        ./c/messages_c.cxx \
        ./c/point_iterator_c.cxx \
        ./c/point_vector_c.cxx \
        ./cxx/binary_lsb_point_vector_reader_context.cxx \
        ./cxx/binary_lsb_point_vector_writer_context.cxx \
        ./cxx/binary_msb_point_vector_reader_context.cxx \
        ./cxx/binary_msb_point_vector_writer_context.cxx \
        ./cxx/binary_point_vector_reader_context.cxx \
        ./cxx/binary_point_vector_writer_context.cxx \
        ./cxx/data_point.cxx \
        ./cxx/data_point_impl.cxx \
        ./cxx/data_point_proxy.cxx \
        ./cxx/double_data_point_parser.cxx \
        ./cxx/double_point_buffer.cxx \
        ./cxx/environment.cxx \
        ./cxx/exceptions.cxx \
        ./cxx/float_data_point_parser.cxx \
        ./cxx/float_point_buffer.cxx \
        ./cxx/info.cxx \
        ./cxx/inline_validity.cxx \
        ./cxx/int16_data_point_parser.cxx \
        ./cxx/int16_point_buffer.cxx \
        ./cxx/int32_data_point_parser.cxx \
        ./cxx/int32_point_buffer.cxx \
        ./cxx/iso5436_2.cxx \
        ./cxx/iso5436_2_container.cxx \
        ./cxx/missing_data_point_parser.cxx \
        ./cxx/point_buffer.cxx \
        ./cxx/point_iterator.cxx \
        ./cxx/point_validity_provider.cxx \
        ./cxx/point_vector.cxx \
        ./cxx/point_vector_iostream.cxx \
        ./cxx/point_vector_parser.cxx \
        ./cxx/point_vector_parser_builder.cxx \
        ./cxx/point_vector_proxy.cxx \
        ./cxx/point_vector_proxy_context.cxx \
        ./cxx/point_vector_proxy_context_list.cxx \
        ./cxx/point_vector_proxy_context_matrix.cxx \
        ./cxx/string.cxx \
        ./cxx/valid_buffer.cxx \
        ./cxx/vector_buffer.cxx \
        ./cxx/vector_buffer_builder.cxx \
        ./cxx/win32_environment.cxx \
        ./cxx/linux_environment.cxx \
        ./cxx/xml_point_vector_reader_context.cxx \
        ./cxx/xml_point_vector_writer_context.cxx \
        ./cxx/zip_stream_buffer.cxx \
        ../../include/opengps/cxx/iso5436_2_xsd.cxx \
        ./xyssl/md5.c

OTHER_FILES += \
    iso5436-2-demo.pro

message($${OUT_PWD}/$${DESTDIR}   $${PWD})
resources.path = $${OUT_PWD}/$${DESTDIR}
resources.files += $${PWD}/../zlib/contrib/minizip/.libs/*.so*


INSTALLS += resources
