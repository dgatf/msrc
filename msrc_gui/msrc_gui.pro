QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

system(git describe --tags > VERSION)
PROJECT_VERSION = "\\\"$$cat(VERSION)"\\\"
system(rm VERSION)
message($${PROJECT_VERSION})
DEFINES += PROJECT_VERSION=$${PROJECT_VERSION}

QMAKE_APPLE_DEVICE_ARCHS = x86_64 arm64

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += ../include

SOURCES += \
    circuitdialog.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    circuitdialog.h \
    mainwindow.h

FORMS += \
    circuitdialog.ui \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    resources.qrc
