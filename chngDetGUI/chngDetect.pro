#-------------------------------------------------
#
# Project created by QtCreator 2014-04-30T13:56:19
#
#-------------------------------------------------

QT       += core gui

CONFIG += console
TARGET = chngDetect
TEMPLATE = app

INCLUDEPATH += /home/bheliom/develop/masterTh/util \
                /home/bheliom/develop/masterTh/common \
                /home/bheliom/develop/masterTh/chngDet \
                /home/bheliom/develop/masterTh/ \
                /home/bheliom/develop/masterTh/maxflowLib \
                /usr/include/vtk-5.8

INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_contrib
LIBS += -lopencv_legacy
LIBS += -lopencv_flann
LIBS += -lopencv_nonfree

LIBS += -lboost_system\

INCLUDEPATH += /usr/include/pcl
LIBS += -lpcl_registration -lpcl_sample_consensus -lpcl_features -lpcl_filters -lpcl_surface -lpcl_segmentation \
        -lpcl_search -lpcl_kdtree -lpcl_octree -lflann_cpp -lpcl_common -lpcl_io \
        -lpcl_visualization \

LIBS += -L/usr/lib \
          -lQVTK \
         -lvtkHybrid \
         -lvtkVolumeRendering \
         -lvtkIO \
        -lvtkRendering \
         -lvtkGenericFiltering \
         -lvtkGraphics \
         -lvtkImaging \
         -lvtkFiltering \
         -lvtkCommon \
         -lvtkftgl \
         -lvtkDICOMParser \
         -lvtksys


SOURCES += main.cpp\
        chngdetect.cpp \
    /home/bheliom/develop/masterTh/util/utilIO.cpp \
    /home/bheliom/develop/masterTh/util/pbaUtil.cpp \
    /home/bheliom/develop/masterTh/util/meshProcess.cpp \
    /home/bheliom/develop/masterTh/common/common.cpp \
    /home/bheliom/develop/masterTh/chngDet/chngDet.cpp \
    /home/bheliom/develop/masterTh/pipelines.cpp \
    /home/bheliom/develop/masterTh/maxflowLib/graph.cpp \
    /home/bheliom/develop/masterTh/maxflowLib/maxflow.cpp \
    /usr/include/wrap/ply/plylib.cpp \

HEADERS  += chngdetect.h \
    /home/bheliom/develop/masterTh/chngDet/chngDet.hpp \
    /home/bheliom/develop/masterTh/util/utilIO.hpp \
    /home/bheliom/develop/masterTh/util/utilClouds.hpp \
    /home/bheliom/develop/masterTh/util/reconstruction.hpp \
    /home/bheliom/develop/masterTh/util/pbaUtil.h \
    /home/bheliom/develop/masterTh/util/pbaDataInterface.h \
    /home/bheliom/develop/masterTh/util/meshProcess.hpp \
    /home/bheliom/develop/masterTh/common/globVariables.hpp \
    /home/bheliom/develop/masterTh/common/common.hpp \
    /home/bheliom/develop/masterTh/pipelines.hpp \
    /home/bheliom/develop/masterTh/maxflowLib/graph.h

FORMS    += chngdetect.ui
