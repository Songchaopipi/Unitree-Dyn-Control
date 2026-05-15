file(REMOVE_RECURSE
  "doc/doxygen-html"
  "doc/doxygen.log"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/distclean.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
