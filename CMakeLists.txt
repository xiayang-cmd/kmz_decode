cmake_minimum_required(VERSION 3.10)

project(wpml_parser LANGUAGES CXX)

# 要求 C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找 protobuf（假设已正确安装）
find_package(Protobuf REQUIRED)

# 可执行文件
add_executable(${PROJECT_NAME}
    main.cpp
    wpml_parser.cpp
    third_party/protobuf/TelemetryDataBuf-new.pb.cpp
)

# 头文件路径
target_include_directories(${PROJECT_NAME} PRIVATE
    ${Protobuf_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pugixml
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/protobuf
)

# 链接库 —— 用导入目标更现代
target_link_libraries(${PROJECT_NAME} PRIVATE
    protobuf::libprotobuf     # 或者 ${Protobuf_LIBRARIES}
)

# 让 pugixml 以 header-only 方式编译
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PUGIXML_HEADER_ONLY
)

# ---- libzip ------------------------------------------------------------
# 尝试优先用 Config 模式；找不到时回退 pkg-config
find_package(LibZip QUIET CONFIG)
if (NOT LibZip_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBZIP REQUIRED libzip)

    add_library(libzip::zip UNKNOWN IMPORTED)
    set_target_properties(libzip::zip PROPERTIES
        IMPORTED_LOCATION "${LIBZIP_LINK_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBZIP_INCLUDE_DIRS}"
    )
endif()
# ------------------------------------------------------------------------

add_executable(unzip_test
    test/unzip_all_libzip.cpp
)
target_link_libraries(unzip_test PRIVATE libzip::zip)
