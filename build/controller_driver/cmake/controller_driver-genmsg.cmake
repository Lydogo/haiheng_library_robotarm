# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "controller_driver: 7 messages, 0 services")

set(MSG_I_FLAGS "-Icontroller_driver:/root/haiheng2_ws/devel/share/controller_driver/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(controller_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg" NAME_WE)
add_custom_target(_controller_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_driver" "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg" "controller_driver/LegMoveFeedback:controller_driver/LegMoveActionFeedback:controller_driver/LegMoveActionResult:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus:controller_driver/LegMoveGoal:controller_driver/LegMoveActionGoal:controller_driver/LegMoveResult"
)

get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg" NAME_WE)
add_custom_target(_controller_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_driver" "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:controller_driver/LegMoveGoal"
)

get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg" NAME_WE)
add_custom_target(_controller_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_driver" "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg" "actionlib_msgs/GoalID:std_msgs/Header:controller_driver/LegMoveResult:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg" NAME_WE)
add_custom_target(_controller_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_driver" "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg" "actionlib_msgs/GoalID:controller_driver/LegMoveFeedback:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg" NAME_WE)
add_custom_target(_controller_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_driver" "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg" ""
)

get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg" NAME_WE)
add_custom_target(_controller_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_driver" "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg" ""
)

get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg" NAME_WE)
add_custom_target(_controller_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_driver" "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg"
  "${MSG_I_FLAGS}"
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
)
_generate_msg_cpp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
)
_generate_msg_cpp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
)
_generate_msg_cpp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
)
_generate_msg_cpp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
)
_generate_msg_cpp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
)
_generate_msg_cpp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(controller_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(controller_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(controller_driver_generate_messages controller_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_cpp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_cpp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_cpp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_cpp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_cpp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_cpp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_cpp _controller_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_driver_gencpp)
add_dependencies(controller_driver_gencpp controller_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg"
  "${MSG_I_FLAGS}"
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
)
_generate_msg_eus(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
)
_generate_msg_eus(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
)
_generate_msg_eus(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
)
_generate_msg_eus(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
)
_generate_msg_eus(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
)
_generate_msg_eus(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(controller_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(controller_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(controller_driver_generate_messages controller_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_eus _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_eus _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_eus _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_eus _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_eus _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_eus _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_eus _controller_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_driver_geneus)
add_dependencies(controller_driver_geneus controller_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg"
  "${MSG_I_FLAGS}"
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
)
_generate_msg_lisp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
)
_generate_msg_lisp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
)
_generate_msg_lisp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
)
_generate_msg_lisp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
)
_generate_msg_lisp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
)
_generate_msg_lisp(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(controller_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(controller_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(controller_driver_generate_messages controller_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_lisp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_lisp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_lisp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_lisp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_lisp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_lisp _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_lisp _controller_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_driver_genlisp)
add_dependencies(controller_driver_genlisp controller_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg"
  "${MSG_I_FLAGS}"
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
)
_generate_msg_nodejs(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
)
_generate_msg_nodejs(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
)
_generate_msg_nodejs(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
)
_generate_msg_nodejs(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
)
_generate_msg_nodejs(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
)
_generate_msg_nodejs(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(controller_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(controller_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(controller_driver_generate_messages controller_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_nodejs _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_nodejs _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_nodejs _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_nodejs _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_nodejs _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_nodejs _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_nodejs _controller_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_driver_gennodejs)
add_dependencies(controller_driver_gennodejs controller_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg"
  "${MSG_I_FLAGS}"
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
)
_generate_msg_py(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
)
_generate_msg_py(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
)
_generate_msg_py(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
)
_generate_msg_py(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
)
_generate_msg_py(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
)
_generate_msg_py(controller_driver
  "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
)

### Generating Services

### Generating Module File
_generate_module_py(controller_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(controller_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(controller_driver_generate_messages controller_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveAction.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_py _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_py _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_py _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveActionFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_py _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveGoal.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_py _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveResult.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_py _controller_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/haiheng2_ws/devel/share/controller_driver/msg/LegMoveFeedback.msg" NAME_WE)
add_dependencies(controller_driver_generate_messages_py _controller_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_driver_genpy)
add_dependencies(controller_driver_genpy controller_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(controller_driver_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(controller_driver_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(controller_driver_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(controller_driver_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(controller_driver_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
