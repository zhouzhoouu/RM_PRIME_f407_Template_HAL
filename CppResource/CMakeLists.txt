cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(User_Defines_Syms

        $<$<CONFIG:Debug>:DEBUG>
)

# 递归收集下的所有源文件
file(GLOB_RECURSE USER_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

# 递归收集所有头文件路径
file(GLOB_RECURSE USER_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.h
)

# 提取路径并去重
set(USER_INCLUDE_DIRS "")
foreach(_header ${USER_HEADERS})
    get_filename_component(_dir ${_header} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ${User_Defines_Syms})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${USER_INCLUDE_DIRS})
