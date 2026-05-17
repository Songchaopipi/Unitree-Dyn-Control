get_filename_component(_mimalloc_shim_dir "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)

set(mimalloc_FOUND TRUE)
set(mimalloc_VERSION 2.1.0)

if(NOT TARGET mimalloc)
    add_library(mimalloc INTERFACE IMPORTED)
    set_target_properties(mimalloc PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_mimalloc_shim_dir}/include")
endif()

