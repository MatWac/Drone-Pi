project(Drone)
include_directories(include)
find_package(Threads REQUIRED)

set(EXECUTABLE_OUTPUT_PATH bin/)
link_directories(lib/)
set(LIBRARY_OUTPUT_PATH lib/)

find_library(pigpio_LIBRARY NAMES pigpio)

add_library(
        motor
        SHARED
        src/Motor.cpp
        include/libmotor/Motor.hpp
)

add_library(
        mpu
        SHARED
        src/MPU6050.cpp
        include/libmpu/MPU6050.h
)

#Déclaration de l'exécutable
add_executable(my_exe src/main.cpp)
#Configuration de l'édition de liens
target_link_libraries(my_exe motor ${pigpio_LIBRARY})
target_link_libraries(my_exe mpu)
target_link_libraries(my_exe ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(my_exe i2c)