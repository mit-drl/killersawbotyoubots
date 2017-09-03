FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/fleet_control/msg"
  "src/fleet_control/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/fleet_control/msg/__init__.py"
  "src/fleet_control/msg/_FleetDebug.py"
  "src/fleet_control/msg/_VelTwist.py"
  "src/fleet_control/msg/_Fleet.py"
  "src/fleet_control/msg/_FleetCommand.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
