cmake_minimum_required(VERSION 3.16)
project(InfiniTimeHostTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(minute_data_protocol_test MinuteDataProtocolTest.cpp)
target_include_directories(minute_data_protocol_test PRIVATE ../src)

enable_testing()
add_test(NAME MinuteDataProtocol COMMAND minute_data_protocol_test)
