# ROS 2 Composition: Testing Component Shutdown Behavior

This package tests the behavior of ROS 2 composition when shutting down components under different conditions:

- Manual Composition using CLI
  - Using `component unload`
  - Killing the container
- Automatic Composition using a launch file
  - Killing the launch process

The shutdown behaviors are tested for two different types of ROS 2 Components:

    - `tc::TimerComponent`: a node using a timer to execute periodic tasks.
    - `tc::ThreadComponent`: a node using thread to execute periodic tasks.

## Test using CLI

### Manually load two components in a container

1. Open a terminal (Ctrl+Alt+T) and start a ROS 2 Container:

    ros2 run rclcpp_components component_container

2. Open a new terminal and load the Timer Component:

    ros2 component load /ComponentManager ros2_test_composition_components tc::TimerComponent

3. In the same terminal, load the Thread Component:

    ros2 component load /ComponentManager ros2_test_composition_components tc::ThreadComponent

In the Container terminal, you should see two increasing counter logs from the two components:

    [...]
    [INFO] [1740866265.099532548] [timer_component]: TimerComponent: 16
    [INFO] [1740866265.320233581] [thread_component]: ThreadComponent: 14
    [INFO] [1740866266.099476756] [timer_component]: TimerComponent: 17
    [INFO] [1740866266.320681491] [thread_component]: ThreadComponent: 15
    [INFO] [1740866267.099508902] [timer_component]: TimerComponent: 18
    [INFO] [1740866267.321268257] [thread_component]: ThreadComponent: 16
    [INFO] [1740866268.099518067] [timer_component]: TimerComponent: 19
    [INFO] [1740866268.321455570] [thread_component]: ThreadComponent: 17
    [...]

### Test #1: Manual Unload

This test verifies that the destructor functions of the two components are correctly called when the `ros2 component unload` command is used.

> **Note:** First, create the container and load the two components as described above.

Open a terminal and unload the Timer Component:

  ros2 component unload /ComponentManager 1

If the destructor is correctly called, you should see:

  [INFO] [1740863892.171265019] [timer_component]: Destroying TimerComponent...
  [INFO] [1740863892.171324522] [timer_component]: TimerComponent destroyed.

Now, unload the Thread Component:

  ros2 component unload /ComponentManager 2

If the destructor is correctly called, you should see:

  [INFO] [1740865871.023047663] [thread_component]: Destroying ThreadComponent...
  [INFO] [1740865871.023105361] [thread_component]: Joining thread...
  [INFO] [1740865871.096381352] [thread_component]: Thread stopped
  [INFO] [1740865871.096469086] [thread_component]: ThreadComponent thread finished.
  [INFO] [1740865871.096555005] [thread_component]: Thread joined.
  [INFO] [1740865871.096602504] [thread_component]: ThreadComponent destroyed.

You can now kill the Container process with `Ctrl+C`.

### Test #2: Killing the Container

This test verifies that the destructor functions of the two components are correctly called when the container process is killed by the SIGINT signal (`Ctrl+C`).

> **Note:** First, create the container and load the two components as described above.

Select the terminal where the container is running the two components and press `Ctrl+C`.

If the destructors are correctly called, you should see something similar to:

  [...]
  ^C[INFO] [1740867172.577737212] [rclcpp]: signal_handler(signum=2)
  [INFO] [1740867172.577987806] [thread_component]: Ctrl+C received: stopping thread
  [INFO] [1740867172.578000029] [thread_component]: Thread stopped
  [INFO] [1740867172.578004778] [thread_component]: ThreadComponent thread finished.
  [INFO] [1740867172.579603680] [thread_component]: Destroying ThreadComponent...
  [INFO] [1740867172.579611921] [thread_component]: Joining thread...
  [INFO] [1740867172.579617159] [thread_component]: Thread joined.
  [INFO] [1740867172.579621420] [thread_component]: ThreadComponent destroyed.
  [INFO] [1740867172.581202861] [timer_component]: Destroying TimerComponent...
  [INFO] [1740867172.581223465] [timer_component]: TimerComponent destroyed.
