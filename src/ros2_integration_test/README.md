# ros2_integration_test

An example of testing C++ ROS2 nodes (ie., ROS2 integration test or level 2 unit testing) using the catch2 framework. 

First, move this directory to the `src/` folder of your colcon
workspace.  This is desirable or you may get a lot of errors from
`ament_lint_cmake`.

For example, your workspace directory structure should look like this:

```
colcon_ws/
├── build/
├── install/
├── log/
└── src/
    ├── ros2_integration_test/
    ├── ros_package1/
    ├── ros_package2/
    └── ros_package3/
```

``` bash
$ mkdir -p ~/colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone git@github.com:TommyChangUMD/ros2_integration_test.git
```


## Terminology

- **Auxiliary test node**: the node under test
- **Integration test node**:  the node which performs the test

## Install the catch_ros2

``` bash
$ source /opt/ros/humble/setup.bash  # if needed
$ apt install ros-${ROS_DISTRO}-catch-ros2
```

Verify that the package is installed
``` bash
$ source /opt/ros/humble/setup.bash  # if needed
$ ros2 pkg list | grep catch_ros2
  catch_ros2
```

see https://github.com/ngmor/catch_ros2/tree/main

## How to Compile:
```bash
$ cd ~/colcon_ws/   # assuming your workspace is at '~/colcon_ws'
$ rm -rf install/ build/
$ source /opt/ros/humble/setup.bash  # if needed
$ colcon build --packages-select integration_test
```

## How to Run:
First, soruce the setup file:
```bash
$ source install/setup.bash
```

### then, run the test and look at the output:
```bash
$ colcon test --packages-select integration_test
$ cat log/latest_test/integration_test/stdout_stderr.log
```

### alternatively, you can combine these into one step:
```bash
$ colcon test  --return-code-on-test-failure --event-handlers console_cohesion+ --packages-select integration_test
$ echo ?
```

### check the return status (after colcon test):
You don't have to re-run the test to see the old result. You can just do:
```
$ colcon test-result --verbose --test-result-base build/integration_test
$ echo $?
```

## Example output

```
$ colcon test  --return-code-on-test-failure --event-handlers console_cohesion+ --packages-select integration_test
... <SKIP> ...

test 1
    Start 1: ExampleIntegration_TestYAML

1: Test command: /usr/bin/python3 "-u" "/opt/ros/humble/share/catch_ros2/cmake/../scripts/run_test.py" "/home/tchang/Projects/ros2_integration_test/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml" "--package-name" "integration_test" "--command" "ros2" "launch" "integration_test" "integration_test.launch.yaml" "result_file:=/home/tchang/Projects/ros2_integration_test/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml"
1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '/home/tchang/Projects/ros2_integration_test/build/integration_test':
1:  - ros2 launch integration_test integration_test.launch.yaml result_file:=/home/tchang/Projects/ros2_integration_test/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml
1: [INFO] [launch]: All log files can be found below /home/tchang/.ros/log/2024-11-10-00-53-27-624341-tchang-IdeaPad-3-17ABA7-1534607
1: [INFO] [launch]: Default logging verbosity is set to INFO
1: [INFO] [service_server-1]: process started with pid [1534608]
1: [INFO] [talker-2]: process started with pid [1534610]
1: [INFO] [integration_test_node-3]: process started with pid [1534612]
1: [service_server-1] [INFO] [1731218007.734964750] []: initialize ROS2
1: [service_server-1] [INFO] [1731218007.739188331] []: Create service_server_node
1: [service_server-1] [INFO] [1731218007.750853990] [service_server_node]: Start server
1: [integration_test_node-3] [INFO] [1731218007.751413977] [IntegrationTestNode1]: Got test_duration =2
1: [integration_test_node-3] [INFO] [1731218007.752059171] [IntegrationTestNode1]: myServiceName client created
1: [integration_test_node-3] [INFO] [1731218007.752104149] [IntegrationTestNode1]: Performing Test...
1: [integration_test_node-3] [INFO] [1731218007.752166657] [IntegrationTestNode1]: duration = 5.7619e-05 service_found=1
1: [integration_test_node-3] [INFO] [1731218007.762298322] [IntegrationTestNode1]: Got test_duration =2
1: [integration_test_node-3] [INFO] [1731218007.763018595] [IntegrationTestNode1]: duration = 2.374e-06 timeout=2
1: [talker-2] [INFO] [1731218008.752196339] [talker]: Publishing: 'Hello World: 1'
1: [integration_test_node-3] [INFO] [1731218008.763276798] [IntegrationTestNode1]: I heard:Hello World: 1
1: [integration_test_node-3] [INFO] [1731218008.863156811] [IntegrationTestNode1]: duration = 1.10012 got_topic=1
1: [integration_test_node-3] Randomness seeded to: 2027099100
1: [integration_test_node-3] ===============================================================================
1: [integration_test_node-3] All tests passed (2 assertions in 2 test cases)
1: [integration_test_node-3] 
1: [INFO] [integration_test_node-3]: process has finished cleanly [pid 1534612]
1: [INFO] [launch]: process[integration_test_node-3] was required: shutting down launched system
1: [INFO] [talker-2]: sending signal 'SIGINT' to process[talker-2]
1: [INFO] [service_server-1]: sending signal 'SIGINT' to process[service_server-1]
1: [talker-2] [INFO] [1731218008.974335815] [rclcpp]: signal_handler(signum=2)
1: [service_server-1] [INFO] [1731218008.974978983] [rclcpp]: signal_handler(signum=2)
1: [service_server-1] [INFO] [1731218008.975229853] [service_server_node]: Shutdown ROS2
1: [INFO] [talker-2]: process has finished cleanly [pid 1534610]
1: [INFO] [service_server-1]: process has finished cleanly [pid 1534608]
1: -- run_test.py: return code 0
1: -- run_test.py: verify result file '/home/tchang/Projects/ros2_integration_test/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml'
1/1 Test #1: ExampleIntegration_TestYAML ......   Passed    1.73 sec

100% tests passed, 0 tests failed out of 1

Total Test time (real) =   1.73 sec
```
