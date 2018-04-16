set(controller_SRC
  mc_handover_controller.cpp
  handover_minJerk.cpp
)

set(controller_HDR
  mc_handover_controller.h
  handover_minJerk.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)
