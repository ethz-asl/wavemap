function(set_wavemap_target_properties target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_definitions(${target} PUBLIC $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:_USE_MATH_DEFINES>)
#  target_compile_options(
#      ${target}
#      PRIVATE # MSVC
#      $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/W4>
#      $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/WX>
#      # Clang/AppleClang
#      $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-fcolor-diagnostics>
#      $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Werror>
#      $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wall>
#      $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wextra>
#      $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wconversion>
#      $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wno-sign-conversion>
#      # GNU
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-fdiagnostics-color=always>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Werror>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wextra>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-pedantic>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wcast-align>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wcast-qual>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wconversion>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wdisabled-optimization>
#      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Woverloaded-virtual>)
  set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR})
  get_filename_component(INCLUDE_DIRS ${INCLUDE_DIRS} PATH)
  target_include_directories(${target}
      PUBLIC $<BUILD_INTERFACE:${INCLUDE_DIRS}> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
endfunction()
