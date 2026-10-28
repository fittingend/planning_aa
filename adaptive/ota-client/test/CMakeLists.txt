add_executable(OTAClientLogicTest
    ota_client_logic_test.cpp
    ../src/otaclient/aa/OtaClientConfig.cpp
    ../src/otaclient/aa/OtacStatusStore.cpp
)

target_include_directories(OTAClientLogicTest
    PRIVATE
        ../include
)

set_target_properties(OTAClientLogicTest PROPERTIES CXX_STANDARD 17)

add_test(
    NAME OTAClientLogicTest
    COMMAND OTAClientLogicTest
            ${CMAKE_CURRENT_SOURCE_DIR}/../config/ota_client_config.json
)
