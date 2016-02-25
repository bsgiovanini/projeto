FILE(REMOVE_RECURSE
  "../src/tum_ardrone/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/tum_ardrone/StateestimationParamsConfig.h"
  "../docs/StateestimationParamsConfig.dox"
  "../docs/StateestimationParamsConfig-usage.dox"
  "../src/tum_ardrone/cfg/StateestimationParamsConfig.py"
  "../docs/StateestimationParamsConfig.wikidoc"
  "../cfg/cpp/tum_ardrone/GUIParamsConfig.h"
  "../docs/GUIParamsConfig.dox"
  "../docs/GUIParamsConfig-usage.dox"
  "../src/tum_ardrone/cfg/GUIParamsConfig.py"
  "../docs/GUIParamsConfig.wikidoc"
  "../cfg/cpp/tum_ardrone/AutopilotParamsConfig.h"
  "../docs/AutopilotParamsConfig.dox"
  "../docs/AutopilotParamsConfig-usage.dox"
  "../src/tum_ardrone/cfg/AutopilotParamsConfig.py"
  "../docs/AutopilotParamsConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
