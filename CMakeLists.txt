cmake_minimum_required(VERSION 3.16)

project(PCCSET)

# Let AUTOMOC and AUTOUIC process GENERATED files.
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

#Compile as C++11
add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Boost
find_package(Boost REQUIRED COMPONENTS thread system)

# Find the QtWidgets library
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)
include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})

# Find PCL 
find_package(PCL 1.10 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

#Find VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# if QVTKWidget.h not in /usr/include/vtk-x.x, explicitly set the header directory
# if libvtkGUISupportQt-x.x.so not in /usr/lib/x86_64-linux-gnu, explicitly set the lib directory
#include_directories(/usr/include/.........)
#include_directories(/usr/lib/.........)


# include header files directory
include_directories(include/PCCSET)

# Modify FILE_DIRECTORY for XInt library
include_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/include)
include_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/source)
include_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/build/source)
link_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/include)
link_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/source)
link_directories({FILE_DIRECTORY}/xlnt-duplicate-string-phonetic/build/source)


# init_qt: Let's do the CMake job for us
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) # For meta object compiler

# Run MOC on headers of Q_OBJECT classes
set(MOC_HDRS include/PCCSET/PCCSET.h include/PCCSET/ExcelDialog.h include/PCCSET/SnapshotDialog.h include/PCCSET/ToolOperations.h)
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

# Run RCC on QRC file
set(RCC_FILES src/icons.qrc)
qt5_add_resources(RCC_SOURCES ${RCC_FILES})

# Run UIC on ui definitions
set(UI_FILES ui/PCCSET.ui ui/ExcelDialog.ui ui/SnapshotDialog.ui)
qt5_wrap_ui(UI_SRCS ${UI_FILES})

set(project_SOURCES src/main.cpp src/PCCSET.cpp src/ExcelDialog.cpp src/SnapshotDialog.cpp src/FileOperations.cpp src/EditOperations.cpp src/ViewOperations.cpp src/ToolOperations.cpp src/EvaluateOperations.cpp ${MOC_SRCS} ${RCC_SOURCES} ${UI_SRCS})

add_executable(${PROJECT_NAME} ${project_SOURCES} src/icons.qrc)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES} -lvtkGUISupportQt-7.1)
target_link_libraries(${PROJECT_NAME} xlnt)


