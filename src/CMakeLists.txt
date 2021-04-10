project(ESC-Spring2012)

option(USE_ISPC "Compile with ISPC" ON)
option(USE_BVH "Use acceleration structure" ON)
find_package(Threads REQUIRED)

set(SRC_FILES 
    scene/sceneloader.cpp
    main.cpp
)
set(HDR_FILES )

add_executable(ESCViewer2021 ${SRC_FILES} )

add_custom_command(
    TARGET ESCViewer2021 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory              
        ${CMAKE_CURRENT_SOURCE_DIR}/models $<TARGET_FILE_DIR:ESCViewer2021>/models
    COMMENT "Copying models" VERBATIM
    )
  
# target_include_directories(ESCViewer2021 PRIVATE ${OpenGL_INCLUDE_DIR})
# target_compile_definitions(ESCViewer2021 PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
if(USE_ISPC)

target_compile_definitions(ESCViewer2021 PUBLIC USE_ISPC)

endif(USE_ISPC)
if(USE_BVH)

target_compile_definitions(ESCViewer2021 PUBLIC USE_BVH)

endif(USE_BVH)

target_link_directories(ESCViewer2021 PRIVATE <TARGET_FILE_DIR:lib>)
target_link_libraries(ESCViewer2021 PUBLIC Threads::Threads)
# jsoncpp_lib
# )
