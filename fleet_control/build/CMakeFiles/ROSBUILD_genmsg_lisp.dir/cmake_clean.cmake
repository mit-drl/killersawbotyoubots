FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/fleet_control/msg"
  "../src/fleet_control/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/FleetDebug.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_FleetDebug.lisp"
  "../msg_gen/lisp/VelTwist.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_VelTwist.lisp"
  "../msg_gen/lisp/Fleet.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Fleet.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
