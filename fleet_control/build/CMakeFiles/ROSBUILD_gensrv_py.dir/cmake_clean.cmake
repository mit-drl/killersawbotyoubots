FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/fleet_control/msg"
  "../src/fleet_control/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/fleet_control/srv/__init__.py"
  "../src/fleet_control/srv/_InsertToHole.py"
  "../src/fleet_control/srv/_SearchHole.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
