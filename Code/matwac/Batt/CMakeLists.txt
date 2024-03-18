project(Batt)
find_library(pigpio_LIBRARY NAMES pigpio)

add_executable(my_exe src/main.cpp)
#Configuration de l'édition de liens
target_link_libraries(my_exe ${pigpio_LIBRARY})
