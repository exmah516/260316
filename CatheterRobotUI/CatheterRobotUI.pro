QT       += core gui widgets
greaterThan(QT_MAJOR_VERSION, 5): QT += core5compat

TARGET = CatheterRobotUI
TEMPLATE = app
CONFIG += c++17

INCLUDEPATH += $$PWD/src/legacy
INCLUDEPATH += $$PWD/third_party/flcatheter/include
INCLUDEPATH += $$PWD/third_party/tcads/include

LIBS += "$$PWD/third_party/flcatheter/lib/FLCatheter.lib"
LIBS += "$$PWD/third_party/tcads/lib/TcAdsDll.lib"

SOURCES += \
    src/main.cpp \
    src/MainWindow.cpp \
    src/ControlEngine.cpp \
    src/ControlThread.cpp \
    src/ForceRecorder.cpp \
    src/AxisDisplayWidget.cpp \
    src/ForcePlotWidget.cpp \
    src/legacy/Handle.cpp \
    src/legacy/ADSComm.cpp

HEADERS += \
    src/SharedState.h \
    src/MainWindow.h \
    src/ControlEngine.h \
    src/ControlThread.h \
    src/ForceRecorder.h \
    src/AxisDisplayWidget.h \
    src/ForcePlotWidget.h \
    src/legacy/Handle.h \
    src/legacy/ADSComm1.h
