if(EXISTS "/home/vv/bin_picking_robot/build/test/cpp-test[1]_tests.cmake")
  include("/home/vv/bin_picking_robot/build/test/cpp-test[1]_tests.cmake")
else()
  add_test(cpp-test_NOT_BUILT cpp-test_NOT_BUILT)
endif()
