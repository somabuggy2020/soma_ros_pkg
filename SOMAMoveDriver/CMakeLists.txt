cmake_minimum_required(VERSION 2.8.12)

project(SOMAMoveDriver)

find_package(Qt5 COMPONENTS Widgets Network REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

ADD_DEFINITIONS(${QT_DEFINITIONS})
ADD_DEFINITIONS(${QT_MESSAGELOGCONTEXT})

INCLUDE_DIRECTORIES("/usr/local/include/EposCmdLib")
LINK_DIRECTORIES("/usr/local/lib/EposCmdLib")

set(src GROB_RECURSE("src"))

add_executable(${PROJECT_NAME}
    "src/main.cpp"
    "src/mainwindow.cpp"
    "src/mainwindow.h"
    "src/mainwindow.ui"
    "src/Data/data.cpp"
    "src/Data/data.h"
    "src/Common/configfilepath.h"
    "src/Common/statevector.h"
    "src/Common/definitions.h"
    "src/Hardware/hardware.cpp"
    "src/Hardware/hardware.h"
    "src/Hardware/hardwareinfo.h"
    "src/Hardware/hardwaredataviewer.cpp"
    "src/Hardware/hardwaredataviewer.h"
    "src/Hardware/hardwaredataviewer.ui"
    "src/Hardware/Motor/motor.cpp"
    "src/Hardware/Motor/motor.h"
    "src/Hardware/Motor/motorinfo.h"
    "src/Hardware/Clutch/clutch.cpp"
    "src/Hardware/Clutch/clutch.h"
    "src/Hardware/Clutch/clutchinfo.h"
    "src/Hardware/Rotary/rotary.cpp"
    "src/Hardware/Rotary/rotary.h"
    "src/Hardware/Rotary/rotaryinfo.h"
    "src/Behavior/behavior.cpp"
    "src/Behavior/behavior.h"
    "src/Behavior/statebase.cpp"
    "src/Behavior/statebase.h"
    "src/Behavior/States/stop.cpp"
    "src/Behavior/States/stop.h"
    "src/Behavior/States/starting.cpp"
    "src/Behavior/States/starting.h"
    "src/Behavior/States/traveling.cpp"
    "src/Behavior/States/traveling.h"
    "src/Behavior/States/stopping.cpp"
    "src/Behavior/States/stopping.h"
    "src/Xbox/xbox.cpp"
    "src/Xbox/xbox.h"
    "src/Xbox/Gamepad/gamepad.c"
    "src/Xbox/Gamepad/gamepad.h")

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Network
    EposCmd
    udev)
