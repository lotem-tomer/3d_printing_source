# search flex
macro(FIND_FLEX)
  if(NOT FLEX_EXECUTABLE)
    if(MSVC)
#      find_program(FLEX_EXECUTABLE flex)
      find_program(FLEX_EXECUTABLE "C:/Progra~1/GnuWin32/bin/flex.exe")
    else(MSVC)
      find_program(FLEX_EXECUTABLE flex)
    endif(MSVC)
    if(NOT FLEX_EXECUTABLE)
      message(FATAL_ERROR "flex not found - aborting")
    endif(NOT FLEX_EXECUTABLE)
  endif(NOT FLEX_EXECUTABLE)
endmacro(FIND_FLEX)

macro(ADD_FLEX_FILES _sources)
  FIND_FLEX()
  foreach(_current_file ${ARGN})
    get_filename_component(_in ${_current_file} ABSOLUTE)
    get_filename_component(_basename ${_current_file} NAME_WE)
    set(_out ${_basename}.cpp)
    # -d option for flex means that it will produce output to stderr while analyzing
    add_custom_command(
      OUTPUT ${_out}
      COMMAND ${FLEX_EXECUTABLE}
      ARGS
      -d -+ -Pmy -o${_out}
      ${_in}
      DEPENDS ${_in}
    )

    set(${_sources} ${${_sources}} ${_out})
  endforeach(_current_file)
endmacro(ADD_FLEX_FILES)

# search bison
macro(FIND_BISON)
  if(NOT BISON_EXECUTABLE)
    if(MSVC)
#      find_program(BISON_EXECUTABLE bison)
      find_program(BISON_EXECUTABLE "C:/Progra~1/GnuWin32/bin/bison.exe")
    else(MSVC)
      find_program(BISON_EXECUTABLE bison)
    endif(MSVC)
    if(NOT BISON_EXECUTABLE)
      message(FATAL_ERROR "bison not found - aborting")
    endif(NOT BISON_EXECUTABLE)
  endif(NOT BISON_EXECUTABLE)
endmacro(FIND_BISON)

macro(ADD_BISON_FILES _sources)
  FIND_BISON()

  foreach(_current_file ${ARGN})
    get_filename_component(_in ${_current_file} ABSOLUTE)
    get_filename_component(_basename ${_current_file} NAME_WE)
    set(_out ${_basename}.cpp)
    add_custom_command(
      OUTPUT ${_out}
      COMMAND ${BISON_EXECUTABLE}
      ARGS
      # also produce a header file
      -d -o${_out}
      ${_in}
      DEPENDS ${_in}
    )
    set(${_sources} ${${_sources}} ${_out} )
  endforeach(_current_file)
endmacro(ADD_BISON_FILES)


project( compute_width_3d )

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
  
  
  set(SRCS "compute_width_3d.cpp;parse.cpp;Polyhedron_viewer.cpp;scan.cpp")
  link_directories($ENV{BOOST_DIR}/lib)
  add_executable(compute_width_3d ${SRCS})
  
  add_to_cached_list(CGAL_EXECUTABLE_TARGETS compute_width_3d)
  
  if (NOT CGAL_AUTO_LINK_ENABLED)
    target_link_libraries(compute_width_3d ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
  endif()
  
  
else()
  
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
  
endif()

project( compute_approx_width_3d )

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
    
  set(SRCS "compute_approx_width_3d.cpp;parse.cpp;Polyhedron_viewer.cpp;scan.cpp")
  link_directories($ENV{BOOST_DIR}/lib)
  add_executable(compute_approx_width_3d ${SRCS})
  
  add_to_cached_list(CGAL_EXECUTABLE_TARGETS compute_approx_width_3d)
  
  if (NOT CGAL_AUTO_LINK_ENABLED)
    target_link_libraries(compute_approx_width_3d ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
  endif()
  
  
else()
  
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
  
endif()

project( compute_width_3d_2_1_c )

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
  
  include_directories(${VRML_READER_INCLUDE_DIR})
  
  set(SRCS "Prepare_VRML_print.cpp;parse.cpp;Polyhedron_viewer.cpp;scan.cpp")
  link_directories($ENV{BOOST_DIR}/lib)
  add_executable(compute_width_3d_2_1_c ${SRCS})
  
  add_to_cached_list(CGAL_EXECUTABLE_TARGETS compute_width_3d_2_1_c)
  
  if (NOT CGAL_AUTO_LINK_ENABLED)
    target_link_libraries(compute_width_3d_2_1_c ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
  endif()
  
  
else()
  
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
  
endif()
