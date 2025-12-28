if(NOT TARGET GTest::GTest)
    add_subdirectory(third-party/google-test)
endif()

set(TESTS
    config_unit_tests.cpp
#    timer_test.cpp

#    modules/encoder_test.cpp
#    modules/motorcontroller_test.cpp

#    modules/registers/hc165_test.cpp
#    modules/registers/hc595_mock.h
#    modules/registers/hc595_test.cpp
#    modules/registers/registercontroller_test.cpp
)

add_executable(rc_agent_unit_tests ${TESTS})

target_link_libraries(rc_agent_unit_tests
    Config

    GTest::GTest
    GTest::Main
)

add_test(test_all amurcontroller_unit_tests)
