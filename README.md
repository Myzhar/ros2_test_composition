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

      ```bash
      ros2 run rclcpp_components component_container_isolated --use_multi_threaded_executor
      ```

2. Open a new terminal and load the Timer Component:

      ```bash
      ros2 component load /ComponentManager ros2_test_composition_components tc::TimerComponent
      ```

3. In the same terminal, load the Thread Component:

      ```bash
      ros2 component load /ComponentManager ros2_test_composition_components tc::ThreadComponent
      ```

In the Container terminal, you should see two increasing counter logs from the two components:

```bash
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
```

### Test #1: Manual Unload

This test verifies that the destructor functions of the two components are correctly called when the `ros2 component unload` command is used.

> **Note:** First, create the container and load the two components as described above.

Open a terminal and unload the Timer Component:

```bash
ros2 component unload /ComponentManager 1
```

If the destructor is correctly called, you should see:

```bash
[INFO] [1740863892.171265019] [timer_component]: Destroying TimerComponent...
[INFO] [1740863892.171324522] [timer_component]: TimerComponent destroyed.
```

Now, unload the Thread Component:

```bash
ros2 component unload /ComponentManager 2
```

If the destructor is correctly called, you should see:

```bash
[INFO] [1740865871.023047663] [thread_component]: Destroying ThreadComponent...
[INFO] [1740865871.023105361] [thread_component]: Joining thread...
[INFO] [1740865871.096381352] [thread_component]: Thread stopped
[INFO] [1740865871.096469086] [thread_component]: ThreadComponent thread finished.
[INFO] [1740865871.096555005] [thread_component]: Thread joined.
[INFO] [1740865871.096602504] [thread_component]: ThreadComponent destroyed.
```

You can now kill the Container process with `Ctrl+C`.

### Test #2: Killing the Container

This test verifies that the destructor functions of the two components are correctly called when the container process is killed by the SIGINT signal (`Ctrl+C`).

> **Note:** First, create the container and load the two components as described above.

Select the terminal where the container is running the two components and press `Ctrl+C`.

If the destructors are correctly called, you should see something similar to:

```bash
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
```

## Test using Composition with a launch file

Use the command

```bash
ros2 launch ros2_test_composition test_composition.launch.py
```

to automatically create the Container and load the two components.

Finally, use `Ctrl+C` to kill the launch process and all the nodes.

If everything works correctly and all the destructors are called, you should get something similar to this:

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

> :pushpin: **Note**:: The problem seems to affect only ROS 2 Humble. I was not able to reproduce it with ROS 2 Jazzy.

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

It seems that the `component_container_isolated` gets stuck at some point, and the destructors of the two components are not correctly called.

The same happens when killing with `Ctrl+C` a `component_container_isolated` manually created by using CLI commands:

```bash
$ ros2 run rclcpp_components component_container_isolated --use_multi_threaded_executor
[INFO] [1740873040.977379421] [ComponentManager]: Load Library: /home/walter/devel/ros2/ros2_walt/install/ros2_test_composition_components/lib/libtimer_component.so
[INFO] [1740873040.977788694] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<tc::TimerComponent>
[INFO] [1740873040.977811672] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<tc::TimerComponent>
[INFO] [1740873040.979723386] [timer_component]: *****************************************
[INFO] [1740873040.979760961] [timer_component]:  ROS 2 Composition Test: Timer Component 
[INFO] [1740873040.979780098] [timer_component]: *****************************************
[INFO] [1740873040.979796650] [timer_component]:  * namespace: /
[INFO] [1740873040.979813273] [timer_component]:  * node name: timer_component
[INFO] [1740873040.979828847] [timer_component]: *****************************************
[INFO] [1740873041.980389354] [timer_component]: TimerComponent: 0
[INFO] [1740873042.980114593] [timer_component]: TimerComponent: 1
[INFO] [1740873043.980349823] [timer_component]: TimerComponent: 2
[INFO] [1740873044.980420490] [timer_component]: TimerComponent: 3
[INFO] [1740873045.980102427] [timer_component]: TimerComponent: 4
[INFO] [1740873046.980024815] [timer_component]: TimerComponent: 5
[INFO] [1740873047.600076983] [ComponentManager]: Load Library: /home/walter/devel/ros2/ros2_walt/install/ros2_test_composition_components/lib/libthread_component.so
[INFO] [1740873047.600473828] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<tc::ThreadComponent>
[INFO] [1740873047.600504209] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<tc::ThreadComponent>
[INFO] [1740873047.602085791] [thread_component]: *****************************************
[INFO] [1740873047.602111284] [thread_component]:  ROS 2 Composition Test: Thread Component 
[INFO] [1740873047.602127557] [thread_component]: *****************************************
[INFO] [1740873047.602143760] [thread_component]:  * namespace: /
[INFO] [1740873047.602158567] [thread_component]:  * node name: thread_component
[INFO] [1740873047.602173304] [thread_component]: *****************************************
[INFO] [1740873047.602250549] [thread_component]: ThreadComponent thread started.
[INFO] [1740873047.602318646] [thread_component]: ThreadComponent: 0
[INFO] [1740873047.980113105] [timer_component]: TimerComponent: 6
[INFO] [1740873048.602540757] [thread_component]: ThreadComponent: 1
[INFO] [1740873048.980103051] [timer_component]: TimerComponent: 7
[INFO] [1740873049.602746782] [thread_component]: ThreadComponent: 2
[INFO] [1740873049.980098272] [timer_component]: TimerComponent: 8
[INFO] [1740873050.602923772] [thread_component]: ThreadComponent: 3
[INFO] [1740873050.980119623] [timer_component]: TimerComponent: 9
[INFO] [1740873051.603341921] [thread_component]: ThreadComponent: 4
[INFO] [1740873051.980182932] [timer_component]: TimerComponent: 10
[INFO] [1740873052.603861799] [thread_component]: ThreadComponent: 5
[INFO] [1740873052.980444760] [timer_component]: TimerComponent: 11
[INFO] [1740873053.604143104] [thread_component]: ThreadComponent: 6
[INFO] [1740873053.980222858] [timer_component]: TimerComponent: 12
[INFO] [1740873054.604331994] [thread_component]: ThreadComponent: 7
[INFO] [1740873054.980199986] [timer_component]: TimerComponent: 13
[INFO] [1740873055.604749934] [thread_component]: ThreadComponent: 8
[INFO] [1740873055.980488569] [timer_component]: TimerComponent: 14
[INFO] [1740873056.605309734] [thread_component]: ThreadComponent: 9
[INFO] [1740873056.980249350] [timer_component]: TimerComponent: 15
[INFO] [1740873057.605790323] [thread_component]: ThreadComponent: 10
[INFO] [1740873057.980342005] [timer_component]: TimerComponent: 16
^C[INFO] [1740873058.445305169] [rclcpp]: signal_handler(signum=2)
[INFO] [1740873058.445496539] [thread_component]: Ctrl+C received: stopping thread
[INFO] [1740873058.445509041] [thread_component]: Thread stopped
[INFO] [1740873058.445514419] [thread_component]: ThreadComponent thread finished.
^C[INFO] [1740873061.296726440] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873062.336458643] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873062.529026767] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873062.728727419] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873062.904943572] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873063.725067411] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873063.905083973] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873064.077282744] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873064.261484064] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873064.433057021] [rclcpp]: signal_handler(signum=2)
^C[INFO] [1740873064.905162270] [rclcpp]: signal_handler(signum=2)
```


