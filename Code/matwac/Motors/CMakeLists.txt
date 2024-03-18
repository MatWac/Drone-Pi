project(Motor)
include_directories(include)

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

#Déclaration de l'exécutable
add_executable(my_exe src/main.cpp)
#Configuration de l'édition de liens
target_link_libraries(my_exe motor ${pigpio_LIBRARY})
