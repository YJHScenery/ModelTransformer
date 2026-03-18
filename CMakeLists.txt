cmake_minimum_required(VERSION 4.1)
project(ModelTransformer)

set(CMAKE_CXX_STANDARD 20)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include"
                    "${CMAKE_CURRENT_SOURCE_DIR}/model_loader"
                    "${CMAKE_CURRENT_SOURCE_DIR}/point_cloud"
                    "${CMAKE_CURRENT_SOURCE_DIR}/arg_parser"
)

set(TARGET_MODE)
set(LIB_DEBUG_SUFFIX)
if (MSVC AND CMAKE_BUILD_TYPE MATCHES "Debug")
    set(TARGET_MODE "Debug")
    set(LIB_DEBUG_SUFFIX "d")
else ()
    set(TARGET_MODE "Release")
    set(LIB_DEBUG_SUFFIX "")
endif ()

set(ASSIMP_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/${TARGET_MODE}/assimp-vc143-mt${LIB_DEBUG_SUFFIX}.lib")


set(ASSIMP_MODEL_LOADER
        model_loader/model_loader.h
        model_loader/model_loader.cpp
        model_loader/plane_coordinate_transform.h
        model_loader/plane_coordinate_transform.cpp
        model_loader/units_define.h
        model_loader/units_define.cpp
        point_cloud/point_cloud_loader.h
        point_cloud/point_cloud_loader.cpp
)

set(ARG_PARSER
        arg_parser/CLI11.hpp
        point_cloud/BinaryMesh.cpp
        point_cloud/BinaryMesh.h
)

add_executable(ModelTransformer main.cpp
        ${ASSIMP_MODEL_LOADER}
        ${ARG_PARSER}
)


target_include_directories(ModelTransformer PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/${TARGET_MODE}"
)


target_link_libraries(ModelTransformer PRIVATE
        ${ASSIMP_LIB_PATH}
)

set(PROJECT_NAME "ModelTransformer")

if (WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_CURRENT_SOURCE_DIR}/bin/${TARGET_MODE}/assimp-vc143-mt${LIB_DEBUG_SUFFIX}.dll"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    )


endif()