set(eigenpy_FOUND TRUE)
set(eigenpy_VERSION 2.7.10)
set(eigenpy_INCLUDE_DIRS "")
set(eigenpy_LIBRARIES "")

if(NOT TARGET eigenpy::eigenpy)
    add_library(eigenpy::eigenpy INTERFACE IMPORTED)
endif()
