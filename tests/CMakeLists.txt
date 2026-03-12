find_package(GTest REQUIRED)

# Helper to add a test executable linked against vision_lib
function(add_vision_test name file)
    add_executable(${name} ${file})
    target_link_libraries(${name} PRIVATE vision_lib GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# Add tests here as files are created
# add_vision_test(test_pipeline    test_pipeline.cpp)
# add_vision_test(test_db          test_db.cpp)
# add_vision_test(test_cli         test_cli.cpp)