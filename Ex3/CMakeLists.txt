# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.



project( hausdorf_distance)

cmake_minimum_required(VERSION 2.8.11)
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 OLD)
endif()

find_package(CGAL QUIET)
find_package(GLUT)
find_package(OpenGL)

if ( CGAL_FOUND )

  include( ${CGAL_USE_FILE} )

  if (CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
  endif()
  include(${CGAL_USE_FILE})

  include_directories($ENV{BOOST_INC_DIR})
  
  
  set(SRCS "hausdorf_distance.cpp")
  link_directories($ENV{BOOST_DIR}/lib)
  add_executable(hausdorf_distance ${SRCS})
  
  add_to_cached_list(CGAL_EXECUTABLE_TARGETS hausdorf_distance)
  
  if (NOT CGAL_AUTO_LINK_ENABLED)
    target_link_libraries(hausdorf_distance ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
  endif()
  
  
else()
  
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
  
endif()


