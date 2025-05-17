# CMakeList.txt : CMake project for qore-cpp
cmake_minimum_required(VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project("qore-cpp")

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Print the current source directory for debugging
message(STATUS "Current source directory: ${CMAKE_CURRENT_SOURCE_DIR}")

# Add third-party dependencies
add_subdirectory(thirdparty/nlohmann_json)

# Define source and include directories
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty")

# Add include directories
include_directories(
  "${INCLUDE_DIR}"                  # Project headers
  "${THIRDPARTY_DIR}/ACSC"          # ACSC headers
  "${THIRDPARTY_DIR}/nlohmann_json" # JSON library headers
)

# Find all source files using GLOB_RECURSE
file(GLOB_RECURSE PROJECT_SOURCES 
  "${SRC_DIR}/*.cpp"
)

# Find all header files
file(GLOB_RECURSE PROJECT_HEADERS 
  "${INCLUDE_DIR}/*.h"
)

# Print files found for debugging
message(STATUS "Source files: ${PROJECT_SOURCES}")
message(STATUS "Header files: ${PROJECT_HEADERS}")

# Add executable target
add_executable(qore-cpp ${PROJECT_SOURCES} ${PROJECT_HEADERS})

# Link with the appropriate ACSC library based on architecture
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  # 64-bit build
  target_link_libraries(qore-cpp "${THIRDPARTY_DIR}/ACSC/ACSCL_x64.LIB" nlohmann_json)
  message(STATUS "Linking with 64-bit ACSC library")
else()
  # 32-bit build
  target_link_libraries(qore-cpp "${THIRDPARTY_DIR}/ACSC/ACSCL_x86.LIB" nlohmann_json)
  message(STATUS "Linking with 32-bit ACSC library")
endif()

# Set additional compiler options if needed
if(MSVC)
  target_compile_options(qore-cpp PRIVATE /W4)
else()
  target_compile_options(qore-cpp PRIVATE -Wall -Wextra)
endif()

# Add preprocessor definitions if needed
# target_compile_definitions(qore-cpp PRIVATE SOME_DEFINITION=1)

# TODO: Add tests and install targets if needed