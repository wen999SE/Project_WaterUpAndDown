#-------------------------------------------------
#
# Project created by QtCreator 2020-12-11T00:43:28
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = WaterUpAndDown
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp\
        V4L2VideoCapture.cpp
HEADERS += \
        mainwindow.h \
        /usr/local/include \
        /usr/local/include/opencv4 \
        V4L2VideoCapture.h

FORMS += \
        mainwindow.ui
LIBS += \
-L/usr/local/lib \
-lpthread \
-lopencv_videostab \
-lopencv_stitching \
-lopencv_superres \
-lopencv_stereo \
-lopencv_hdf \
-lopencv_tracking \
-lopencv_face \
-lopencv_rgbd \
-lopencv_ccalib \
-lopencv_optflow \
-lopencv_dnn_objdetect \
-lopencv_sfm \
-lopencv_xfeatures2d \
-lopencv_shape \
-lopencv_hfs \
-lopencv_plot \
-lopencv_reg \
-lopencv_structured_light \
-lopencv_aruco \
-lopencv_img_hash \
-lopencv_bioinspired \
-lopencv_bgsegm \
-lopencv_video \
-lopencv_surface_matching \
-lopencv_datasets \
-lopencv_text \
-lopencv_dnn \
-lopencv_ml \
-lopencv_freetype \
-lopencv_fuzzy \
-lopencv_dpm \
-lopencv_highgui \
-lopencv_videoio \
-lopencv_saliency \
-lopencv_line_descriptor \
-lopencv_xobjdetect \
-lopencv_objdetect \
-lopencv_phase_unwrapping \
-lopencv_ximgproc \
-lopencv_calib3d \
-lopencv_imgcodecs \
-lopencv_features2d \
-lopencv_flann \
-lopencv_xphoto \
-lopencv_photo \
-lopencv_imgproc \
-lopencv_core        
