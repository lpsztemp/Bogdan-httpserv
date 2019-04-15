cmake_minimum_required(VERSION 2.8.3)

project(cmd_client)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wl,--no-as-needed -Wall -Wno-unused-function -Wno-unused-local-typedefs -Wno-maybe-uninitialized")

SET(SOURCES arch_ac_domain_xml2bin.cpp bin2text.cpp binary_streams.cpp domain_converter.cpp entrypoint.cpp face.cpp hgt_optimizer.cpp radio_hf_domain_xml2bin.cpp text_streams.cpp xml_exceptions.cpp xml_parser.cpp xml2bin.cpp)

set_property(SOURCE ${SOURCES} PROPERTY CXX_STANDARD 17)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} pthread)

install(TARGETS ${PROJECT_NAME} DESTINATION ${BIN})

message(${BIN})
