if("1.3.0" STREQUAL "")
  message(FATAL_ERROR "Tag for git checkout should not be empty.")
endif()

set(run 0)

if("/home/joemelt101/catkin_ws/devel/src/libcreate-stamp/libcreate-gitinfo.txt" IS_NEWER_THAN "/home/joemelt101/catkin_ws/devel/src/libcreate-stamp/libcreate-gitclone-lastrun.txt")
  set(run 1)
endif()

if(NOT run)
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/joemelt101/catkin_ws/devel/src/libcreate-stamp/libcreate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/home/joemelt101/catkin_ws/devel/src/libcreate"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/joemelt101/catkin_ws/devel/src/libcreate'")
endif()

# try the clone 3 times incase there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git" clone "https://github.com/AutonomyLab/libcreate.git" "libcreate"
    WORKING_DIRECTORY "/home/joemelt101/catkin_ws/devel/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/AutonomyLab/libcreate.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git" checkout 1.3.0
  WORKING_DIRECTORY "/home/joemelt101/catkin_ws/devel/src/libcreate"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '1.3.0'")
endif()

execute_process(
  COMMAND "/usr/bin/git" submodule init
  WORKING_DIRECTORY "/home/joemelt101/catkin_ws/devel/src/libcreate"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to init submodules in: '/home/joemelt101/catkin_ws/devel/src/libcreate'")
endif()

execute_process(
  COMMAND "/usr/bin/git" submodule update --recursive
  WORKING_DIRECTORY "/home/joemelt101/catkin_ws/devel/src/libcreate"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/joemelt101/catkin_ws/devel/src/libcreate'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/joemelt101/catkin_ws/devel/src/libcreate-stamp/libcreate-gitinfo.txt"
    "/home/joemelt101/catkin_ws/devel/src/libcreate-stamp/libcreate-gitclone-lastrun.txt"
  WORKING_DIRECTORY "/home/joemelt101/catkin_ws/devel/src/libcreate"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/joemelt101/catkin_ws/devel/src/libcreate-stamp/libcreate-gitclone-lastrun.txt'")
endif()

