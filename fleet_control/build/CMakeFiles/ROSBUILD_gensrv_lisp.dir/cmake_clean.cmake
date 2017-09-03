FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/fleet_control/msg"
  "../src/fleet_control/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/InsertToHole.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_InsertToHole.lisp"
  "../srv_gen/lisp/SearchHole.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SearchHole.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
