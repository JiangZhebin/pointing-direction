cmake_minimum_required (VERSION 2.6)
project (Gesture_recognition)

set(ASTRA_UNIX TRUE)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")




add_compile_options(-std=c++11)
include_directories(include)
add_executable(${PROJECT_NAME} src/gesture_recognition.cpp)




find_package(SFML COMPONENTS graphics window system)
find_package(Astra REQUIRED )
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})


set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "${SAMPLE_DIR_FOLDER}sfml")

include_directories(${SFML_INCLUDE_DIR})
add_library(JLIB src/pointed_object.cpp src/Utilities.cpp )

target_link_libraries(${PROJECT_NAME} ${ASTRA_LIBRARIES} ${SFML_LIBRARIES} JLIB libpaho-mqttpp3.so libpaho-mqtt3as.so
  ${JSONCPP_LIBRARIES})

#set_debug_working_dir(${PROJECT_NAME} "$(TargetDir)")

#add_astra_sdk_dependency(${PROJECT_NAME})

#install_sample(${PROJECT_NAME})

