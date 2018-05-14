set(controller_SRC
  handover_controller.cpp
  handover_minJerk.cpp
  handover_step.cpp

 # handover_admittanceTask.cpp
)

set(controller_HDR
  handover_controller.h
  handover_minJerk.h
  handover_step.cpp
 # handover_admittanceTask.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)
