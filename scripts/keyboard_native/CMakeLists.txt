# Native joint worker (r1_keyboard_worker) and its unit tests.
# Build with scripts/build_keyboard.sh (output in build/keyboard), then run the
# tests with: ctest --test-dir build/keyboard
cmake_minimum_required(VERSION 3.16)
project(r1_keyboard LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
get_filename_component(SDK "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
find_package(Threads REQUIRED)

# Vendored Ruckig v0.15.3 (third_party/ruckig/SOURCE.json): local
# state-to-state sources only, no cloud client.
set(RUCKIG "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ruckig")
add_library(r1_ruckig STATIC
  "${RUCKIG}/src/brake.cpp"
  "${RUCKIG}/src/position_first_step1.cpp" "${RUCKIG}/src/position_first_step2.cpp"
  "${RUCKIG}/src/position_second_step1.cpp" "${RUCKIG}/src/position_second_step2.cpp"
  "${RUCKIG}/src/position_third_step1.cpp" "${RUCKIG}/src/position_third_step2.cpp"
  "${RUCKIG}/src/velocity_second_step1.cpp" "${RUCKIG}/src/velocity_second_step2.cpp"
  "${RUCKIG}/src/velocity_third_step1.cpp" "${RUCKIG}/src/velocity_third_step2.cpp")
target_include_directories(r1_ruckig PUBLIC "${RUCKIG}/include")

# Build privately; do not overwrite the SDK's existing binaries or shared library.
add_executable(r1_keyboard_worker worker.cpp
  "${SDK}/src/arm_motor_bus.cpp"
  "${SDK}/thirdparty/motor_controller/src/SerialPort.cpp"
  "${SDK}/thirdparty/motor_controller/src/MotorProtocol.cpp")
target_include_directories(r1_keyboard_worker PRIVATE "${SDK}/include"
  "${SDK}/thirdparty/motor_controller/include")
target_link_libraries(r1_keyboard_worker PRIVATE Threads::Threads r1_ruckig)
target_compile_options(r1_keyboard_worker PRIVATE -Wall -Wextra -Wpedantic)

# Unit tests of the header-only control components. tests/test_gravity.py also
# runs test_gravity to compare the native and Python gravity models. -UNDEBUG
# keeps assert() active in Release builds for the tests that rely on it.
enable_testing()
add_executable(test_smooth_motion test_smooth_motion.cpp)
target_link_libraries(test_smooth_motion PRIVATE r1_ruckig)
add_test(NAME smooth_motion COMMAND test_smooth_motion)
add_executable(test_gravity test_gravity.cpp)
target_compile_options(test_gravity PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME gravity COMMAND test_gravity)
add_executable(test_feedback_diagnostics test_feedback_diagnostics.cpp)
target_include_directories(test_feedback_diagnostics PRIVATE "${SDK}/include")
target_compile_options(test_feedback_diagnostics PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME feedback_diagnostics COMMAND test_feedback_diagnostics)
add_executable(test_gravity_diagnostic test_gravity_diagnostic.cpp)
target_compile_options(test_gravity_diagnostic PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
add_test(NAME gravity_diagnostic COMMAND test_gravity_diagnostic)
add_executable(test_position_integral test_position_integral.cpp)
target_compile_options(test_position_integral PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
add_test(NAME position_integral COMMAND test_position_integral)
