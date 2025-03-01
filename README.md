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

      `ros2 run rclcpp_components component_container_isolated --use_multi_threaded_executor`

2. Open a new terminal and load the Timer Component:

      `ros2 component load /ComponentManager ros2_test_composition_components tc::TimerComponent`

3. In the same terminal, load the Thread Component:

      `ros2 component load /ComponentManager ros2_test_composition_components tc::ThreadComponent`

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

## Test using Composition with a launch file

Use the command

  ros2 launch ros2_test_composition test_composition.launch.py

to automatically create the Container and load the two components.

Finally use `Ctrl+C` to kill the launch process and all the nodes.

If everything works correctly and all the desctructors are called, you should get something similar to this:

```bash
$ ros2 launch ros2_test_composition test_composition.launch.py 
[INFO] [launch]: All log files can be found below /home/walter/.ros/log/2025-03-02-00-23-51-473888-walter-Legion-5-u24-6839
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: * Starting Composable node container: /test_composition/test_container
[INFO] [launch.user]: * Starting a Timer Component
[INFO] [launch.user]: * Starting a Thread Component
[INFO] [component_container_isolated-1]: process started with pid [6867]
[component_container_isolated-1] [INFO] [1740871431.953412335] [test_composition.test_container]: Load Library: /home/walter/devel/ros2/ros2_wl/install/ros2_test_composition_components/lib/libtimer_component.so
[component_container_isolated-1] [INFO] [1740871431.953951718] [test_composition.test_container]: Found class: rclcpp_components::NodeFactoryTemplate<tc::TimerComponent>
[component_container_isolated-1] [INFO] [1740871431.953977699] [test_composition.test_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<tc::TimerComponent>
[component_container_isolated-1] [INFO] [1740871431.968485863] [test_composition.timer_component]: *****************************************
[component_container_isolated-1] [INFO] [1740871431.968507374] [test_composition.timer_component]:  ROS 2 Composition Test: Timer Component 
[component_container_isolated-1] [INFO] [1740871431.968522599] [test_composition.timer_component]: *****************************************
[component_container_isolated-1] [INFO] [1740871431.968535520] [test_composition.timer_component]:  * namespace: /test_composition
[component_container_isolated-1] [INFO] [1740871431.968548301] [test_composition.timer_component]:  * node name: timer_component
[component_container_isolated-1] [INFO] [1740871431.968560313] [test_composition.timer_component]: *****************************************
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/test_composition/timer_component' in container '/test_composition/test_container'
[component_container_isolated-1] [INFO] [1740871431.970147173] [test_composition.test_container]: Load Library: /home/walter/devel/ros2/ros2_wl/install/ros2_test_composition_components/lib/libthread_component.so
[component_container_isolated-1] [INFO] [1740871431.970429680] [test_composition.test_container]: Found class: rclcpp_components::NodeFactoryTemplate<tc::ThreadComponent>
[component_container_isolated-1] [INFO] [1740871431.970473610] [test_composition.test_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<tc::ThreadComponent>
[component_container_isolated-1] [INFO] [1740871431.973512746] [test_composition.thread_component]: *****************************************
[component_container_isolated-1] [INFO] [1740871431.973531883] [test_composition.thread_component]:  ROS 2 Composition Test: Thread Component 
[component_container_isolated-1] [INFO] [1740871431.973544245] [test_composition.thread_component]: *****************************************
[component_container_isolated-1] [INFO] [1740871431.973557165] [test_composition.thread_component]:  * namespace: /test_composition
[component_container_isolated-1] [INFO] [1740871431.973571413] [test_composition.thread_component]:  * node name: thread_component
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/test_composition/thread_component' in container '/test_composition/test_container'
[component_container_isolated-1] [INFO] [1740871431.973583915] [test_composition.thread_component]: *****************************************
[component_container_isolated-1] [INFO] [1740871431.973677921] [test_composition.thread_component]: ThreadComponent thread started.
[component_container_isolated-1] [INFO] [1740871431.973787362] [test_composition.thread_component]: ThreadComponent: 0
[component_container_isolated-1] [INFO] [1740871432.968689415] [test_composition.timer_component]: TimerComponent: 0
[component_container_isolated-1] [INFO] [1740871432.973894464] [test_composition.thread_component]: ThreadComponent: 1
[component_container_isolated-1] [INFO] [1740871433.968701863] [test_composition.timer_component]: TimerComponent: 1
[component_container_isolated-1] [INFO] [1740871433.974004272] [test_composition.thread_component]: ThreadComponent: 2
[component_container_isolated-1] [INFO] [1740871434.969010348] [test_composition.timer_component]: TimerComponent: 2
[component_container_isolated-1] [INFO] [1740871434.974146535] [test_composition.thread_component]: ThreadComponent: 3
[component_container_isolated-1] [INFO] [1740871435.969058791] [test_composition.timer_component]: TimerComponent: 3
[component_container_isolated-1] [INFO] [1740871435.974265239] [test_composition.thread_component]: ThreadComponent: 4
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[component_container_isolated-1] [INFO] [1740871436.560358157] [rclcpp]: signal_handler(signum=2)
[component_container_isolated-1] [INFO] [1740871436.560609096] [test_composition.thread_component]: Ctrl+C received: stopping thread
[component_container_isolated-1] [INFO] [1740871436.560619573] [test_composition.thread_component]: Thread stopped
[component_container_isolated-1] [INFO] [1740871436.560622855] [test_composition.thread_component]: ThreadComponent thread finished.
[component_container_isolated-1] [INFO] [1740871436.560691998] [test_composition.thread_component]: Destroying ThreadComponent...
[component_container_isolated-1] [INFO] [1740871436.560696258] [test_composition.thread_component]: Joining thread...
[component_container_isolated-1] [INFO] [1740871436.560701077] [test_composition.thread_component]: Thread joined.
[component_container_isolated-1] [INFO] [1740871436.560704011] [test_composition.thread_component]: ThreadComponent destroyed.
[INFO] [component_container_isolated-1]: process has finished cleanly [pid 6867]
[component_container_isolated-1] [INFO] [1740871436.563705504] [test_composition.timer_component]: Destroying TimerComponent...
[component_container_isolated-1] [INFO] [1740871436.563724641] [test_composition.timer_component]: TimerComponent destroyed.
```

## Problems with ROS 2 Humble

This is what happens when killing the launch file in ROS 2 Humble:

```bash
$ ros2 launch ros2_test_composition test_composition.launch.py
[INFO] [launch]: All log files can be found below /home/walter/.ros/log/2025-03-02-00-03-06-565875-walter-Legion-5-15ACH6H-56619
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: * Starting Composable node container: /test_composition/test_container
[INFO] [launch.user]: * Starting a Timer Component
[INFO] [launch.user]: * Starting a Thread Component
[INFO] [component_container_isolated-1]: process started with pid [56630]
[component_container_isolated-1] [INFO] [1740870186.909257148] [test_composition.test_container]: Load Library: /home/walter/devel/ros2/ros2_walt/install/ros2_test_composition_components/lib/libtimer_component.so
[component_container_isolated-1] [INFO] [1740870186.910120519] [test_composition.test_container]: Found class: rclcpp_components::NodeFactoryTemplate<tc::TimerComponent>
[component_container_isolated-1] [INFO] [1740870186.910152017] [test_composition.test_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<tc::TimerComponent>
[component_container_isolated-1] [INFO] [1740870186.912552792] [test_composition.timer_component]: *****************************************
[component_container_isolated-1] [INFO] [1740870186.912592322] [test_composition.timer_component]:  ROS 2 Composition Test: Timer Component 
[component_container_isolated-1] [INFO] [1740870186.912605242] [test_composition.timer_component]: *****************************************
[component_container_isolated-1] [INFO] [1740870186.912617884] [test_composition.timer_component]:  * namespace: /test_composition
[component_container_isolated-1] [INFO] [1740870186.912630665] [test_composition.timer_component]:  * node name: timer_component
[component_container_isolated-1] [INFO] [1740870186.912642258] [test_composition.timer_component]: *****************************************
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/test_composition/timer_component' in container '/test_composition/test_container'
[component_container_isolated-1] [INFO] [1740870186.914282817] [test_composition.test_container]: Load Library: /home/walter/devel/ros2/ros2_walt/install/ros2_test_composition_components/lib/libthread_component.so
[component_container_isolated-1] [INFO] [1740870186.914460561] [test_composition.test_container]: Found class: rclcpp_components::NodeFactoryTemplate<tc::ThreadComponent>
[component_container_isolated-1] [INFO] [1740870186.914480256] [test_composition.test_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<tc::ThreadComponent>
[component_container_isolated-1] [INFO] [1740870186.915900327] [test_composition.thread_component]: *****************************************
[component_container_isolated-1] [INFO] [1740870186.915921699] [test_composition.thread_component]:  ROS 2 Composition Test: Thread Component 
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/test_composition/thread_component' in container '/test_composition/test_container'
[component_container_isolated-1] [INFO] [1740870186.915934689] [test_composition.thread_component]: *****************************************
[component_container_isolated-1] [INFO] [1740870186.915945514] [test_composition.thread_component]:  * namespace: /test_composition
[component_container_isolated-1] [INFO] [1740870186.915957457] [test_composition.thread_component]:  * node name: thread_component
[component_container_isolated-1] [INFO] [1740870186.915967584] [test_composition.thread_component]: *****************************************
[component_container_isolated-1] [INFO] [1740870186.916025971] [test_composition.thread_component]: ThreadComponent thread started.
[component_container_isolated-1] [INFO] [1740870186.916082053] [test_composition.thread_component]: ThreadComponent: 0
[component_container_isolated-1] [INFO] [1740870187.912827983] [test_composition.timer_component]: TimerComponent: 0
[component_container_isolated-1] [INFO] [1740870187.916200102] [test_composition.thread_component]: ThreadComponent: 1
[component_container_isolated-1] [INFO] [1740870188.913014093] [test_composition.timer_component]: TimerComponent: 1
[component_container_isolated-1] [INFO] [1740870188.916747428] [test_composition.thread_component]: ThreadComponent: 2
[component_container_isolated-1] [INFO] [1740870189.913053224] [test_composition.timer_component]: TimerComponent: 2
[component_container_isolated-1] [INFO] [1740870189.917040361] [test_composition.thread_component]: ThreadComponent: 3
[component_container_isolated-1] [INFO] [1740870190.912790750] [test_composition.timer_component]: TimerComponent: 3
[component_container_isolated-1] [INFO] [1740870190.917506466] [test_composition.thread_component]: ThreadComponent: 4
[component_container_isolated-1] [INFO] [1740870191.913063152] [test_composition.timer_component]: TimerComponent: 4
[component_container_isolated-1] [INFO] [1740870191.918061583] [test_composition.thread_component]: ThreadComponent: 5
[component_container_isolated-1] [INFO] [1740870192.913011878] [test_composition.timer_component]: TimerComponent: 5
[component_container_isolated-1] [INFO] [1740870192.918465601] [test_composition.thread_component]: ThreadComponent: 6
[component_container_isolated-1] [INFO] [1740870193.913020913] [test_composition.timer_component]: TimerComponent: 6
[component_container_isolated-1] [INFO] [1740870193.919016531] [test_composition.thread_component]: ThreadComponent: 7
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[component_container_isolated-1] [INFO] [1740870194.314739311] [rclcpp]: signal_handler(signum=2)
[component_container_isolated-1] [INFO] [1740870194.314891564] [test_composition.thread_component]: Ctrl+C received: stopping thread
[component_container_isolated-1] [INFO] [1740870194.314900085] [test_composition.thread_component]: Thread stopped
[component_container_isolated-1] [INFO] [1740870194.314903437] [test_composition.thread_component]: ThreadComponent thread finished.
[ERROR] [component_container_isolated-1]: process[component_container_isolated-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
[INFO] [component_container_isolated-1]: sending signal 'SIGTERM' to process[component_container_isolated-1]
[component_container_isolated-1] [INFO] [1740870200.131167169] [rclcpp]: signal_handler(signum=15)
[ERROR] [component_container_isolated-1]: process[component_container_isolated-1] failed to terminate '10.0' seconds after receiving 'SIGTERM', escalating to 'SIGKILL'
[INFO] [component_container_isolated-1]: sending signal 'SIGKILL' to process[component_container_isolated-1]
[ERROR] [component_container_isolated-1]: process has died [pid 56630, exit code -9, cmd '/opt/ros/humble/lib/rclcpp_components/component_container_isolated --use_multi_threaded_executor --ros-args --log-level info --ros-args -r __node:=test_container -r __ns:=/test_composition'].
```

it seems that the `component_container_isolated` gets stuck at some point and the destructors of the two components are not called completely.

The same happens when killing with `Ctrl+C` a `component_container_isolated` manually created by using CLI commands.

> :pushpin: **Note**:: The problem seems to affect only ROS 2 Humble. I was not able to reproduce it with ROS 2 Jazzy
