set(pinocchio_FOUND TRUE)
set(pinocchio_VERSION 3.8.0)
set(PINOCCHIO_VERSION 3.8.0)
set(pinocchio_INCLUDE_DIRS "/usr/local/include")
set(PINOCCHIO_INCLUDE_DIRS "${pinocchio_INCLUDE_DIRS}")

if(NOT TARGET pinocchio::pinocchio_headers)
    add_library(pinocchio::pinocchio_headers INTERFACE IMPORTED)
    set_target_properties(pinocchio::pinocchio_headers PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include")
endif()

if(NOT TARGET pinocchio::pinocchio_default)
    add_library(pinocchio::pinocchio_default SHARED IMPORTED)
    set_target_properties(pinocchio::pinocchio_default PROPERTIES
        IMPORTED_LOCATION "/usr/local/lib/libpinocchio_default.so"
        INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio_headers")
endif()

if(NOT TARGET pinocchio::pinocchio_parsers)
    add_library(pinocchio::pinocchio_parsers SHARED IMPORTED)
    set_target_properties(pinocchio::pinocchio_parsers PROPERTIES
        IMPORTED_LOCATION "/usr/local/lib/libpinocchio_parsers.so"
        INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio_default")
endif()

if(NOT TARGET pinocchio::pinocchio_collision)
    add_library(pinocchio::pinocchio_collision INTERFACE IMPORTED)
endif()

if(NOT TARGET pinocchio::pinocchio)
    add_library(pinocchio::pinocchio INTERFACE IMPORTED)
    set_target_properties(pinocchio::pinocchio PROPERTIES
        INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio_default;pinocchio::pinocchio_parsers")
endif()

set(pinocchio_LIBRARIES pinocchio::pinocchio)
set(PINOCCHIO_LIBRARIES ${pinocchio_LIBRARIES})

