
      TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


QMAKE_LFLAGS += -no-pie
LIBS += -lgxiapi \
        -lpthread \
        -lX11

//opencv-3.3.1-dev

#manfiold2
#INCLUDEPATH += /usr/include/opencv2 \
#               /usr/include/opencv

#nano
#INCLUDEPATH += /usr/include/opencv4

#pc
#INCLUDEPATH += /usr/local/include/opencv4

#NUC
INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv4 \
               /usr/local/include/opencv4/opencv2


# 编译生成的so文件（类似于windows下的dll文件）
LIBS += -L/usr/lib/aarch64-linux-gnu -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_highgui -lopencv_objdetect -lopencv_ml -lopencv_video -lopencv_videoio  -lopencv_calib3d -lopencv_dnn -lopencv_features2d -lopencv_flann -lopencv_photo -lopencv_stitching

#nano use
#-lopencv_gapi

SOURCES += \
       JLURoboVision/AngleSolver/AngleSolver.cpp \
       JLURoboVision/Armor/ArmorBox.cpp \
       JLURoboVision/Armor/ArmorDetector.cpp \
       JLURoboVision/Armor/ArmorNumClassifier.cpp \
       JLURoboVision/Armor/findLights.cpp \
       JLURoboVision/Armor/LightBar.cpp \
       JLURoboVision/Armor/matchArmors.cpp \
       JLURoboVision/Armor/ShootState.cpp \
       JLURoboVision/GxCamera/GxCamera.cpp \
       JLURoboVision/Main/ArmorDetecting.cpp \
       JLURoboVision/Main/ImageUpdating.cpp \
       JLURoboVision/Main/main.cpp \
       JLURoboVision/Main/filter.cpp \  
       JLURoboVision/Serial/Serial.cpp \
       JLURoboVision/Serial/predict.cpp \
       JLURoboVision/Serial/SerialReceive.cpp \
    JLURoboVision/Serial/SerialSend.cpp
#       JLURoboVision/Can/Can.cpp

HEADERS += \
    JLURoboVision/AngleSolver/AngleSolver.h \
    JLURoboVision/Armor/Armor.h \
    JLURoboVision/GxCamera/GXSDK/DxImageProc.h \
    JLURoboVision/GxCamera/GXSDK/GxIAPI.h \
    JLURoboVision/GxCamera/GxCamera.h \
    JLURoboVision/General/General.h \
    JLURoboVision/Wind/Energy.h \
    JLURoboVision/Serial/Serial.h \
    JLURoboVision/Serial/predict.h \
    JLURoboVision/AngleSolver/filter.h
#    JLURoboVision/Can/Can.h

DISTFILES += \
   JLURoboVision/General/svm1-6.xml \
    JLURoboVision/General/camera_params.xml

