get_filename_component(_pinocchio_shim_dir "${CMAKE_CURRENT_LIST_DIR}" DIRECTORY)
get_filename_component(_pinocchio_shim_dir "${_pinocchio_shim_dir}" DIRECTORY)
get_filename_component(_pinocchio_repo_root "${_pinocchio_shim_dir}" DIRECTORY)

set(pinocchio_FOUND TRUE)
set(pinocchio_VERSION 3.8.0)
set(PINOCCHIO_VERSION 3.8.0)
set(pinocchio_INCLUDE_DIRS "${_pinocchio_repo_root}/third_party/pinocchio")
set(PINOCCHIO_INCLUDE_DIRS "${pinocchio_INCLUDE_DIRS}")

if(CMAKE_CXX_COMPILER MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(_pinocchio_archive "${_pinocchio_repo_root}/third_party/pinocchio/libpinocchio_lin_arm64.a")
else()
    set(_pinocchio_archive "${_pinocchio_repo_root}/third_party/pinocchio/libpinocchio_lin_x64.a")
endif()

if(NOT TARGET pinocchio::pinocchio_headers)
    add_library(pinocchio::pinocchio_headers INTERFACE IMPORTED)
    set_target_properties(pinocchio::pinocchio_headers PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_pinocchio_repo_root}/third_party/pinocchio")
endif()

if(NOT TARGET pinocchio::pinocchio_default)
    add_library(pinocchio::pinocchio_default STATIC IMPORTED)
    set_target_properties(pinocchio::pinocchio_default PROPERTIES
        IMPORTED_LOCATION "${_pinocchio_archive}"
        INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio_headers")
endif()

if(NOT TARGET pinocchio::pinocchio_parsers)
    add_library(pinocchio::pinocchio_parsers INTERFACE IMPORTED)
    set_target_properties(pinocchio::pinocchio_parsers PROPERTIES
        INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio_default")
endif()

if(NOT TARGET pinocchio::pinocchio_collision)
    add_library(pinocchio::pinocchio_collision INTERFACE IMPORTED)
endif()

if(NOT TARGET pinocchio::pinocchio)
    add_library(pinocchio::pinocchio INTERFACE IMPORTED)
    set_target_properties(pinocchio::pinocchio PROPERTIES
        INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio_default")
endif()

set(pinocchio_LIBRARIES pinocchio::pinocchio_default)
set(PINOCCHIO_LIBRARIES ${pinocchio_LIBRARIES})
