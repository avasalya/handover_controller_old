set(FSM_STATES_INSTALL_PREFIX "${MC_RTC_LIBDIR}/mc_controller/mc_handover_controller/states")

macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED  ${state_SRC} ${state_HDR})
  set_target_properties(${state_name} PROPERTIES PREFIX "")
  target_link_libraries(${state_name} mc_handover_controller)
  install(TARGETS ${state_name} DESTINATION ${FSM_STATES_INSTALL_PREFIX})
  install(FILES ${state_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/mc_control/fsm/states)
endmacro()


macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${state_name}.h)
endmacro()


# INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}/usr/include/Eigen3" )

add_fsm_state_simple(GoHalfSittingStep)
add_fsm_state_simple(GoInitPoseStep)
add_fsm_state_simple(OpenGrippersStep)
add_fsm_state_simple(CloseGrippersStep)

