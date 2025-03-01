# ros2_test_composition

Test ROS 2 composition behavior when closing components

## Test using CLI

1. Open a terminal (Ctrl+Alt+t) and start a ROS 2 Container:

    ```bash
    ros2 run rclcpp_components component_container
    ```

2. Open a new terminal and load one Timer Component

    ```bash
    ros2 component load /ComponentManager  ros2_test_composition_components tc::TimerComponent
    ```

### Test manual unload

Open a terminal and unload the Timer Component

```bash
ros2 component unload /ComponentManager 1
```

If the destructor is correctly called, you should get

```bash
[INFO] [1740863892.171265019] [timer_component]: Destroying TimerComponent...
[...]
[INFO] [1740863892.171324522] [timer_component]: TimerComponent destroyed.
```

Now, unload the Thread Component

```bash
ros2 component unload /ComponentManager 1
```

If the destructor is correctly called, you should get

```bash
[INFO] [1740865871.023047663] [thread_component]: Destroying ThreadComponent...
[INFO] [1740865871.023105361] [thread_component]: Joining thread...
[INFO] [1740865871.096381352] [thread_component]: Thread stopped
[INFO] [1740865871.096469086] [thread_component]: ThreadComponent thread finished.
[INFO] [1740865871.096555005] [thread_component]: Thread joined.
[INFO] [1740865871.096602504] [thread_component]: ThreadComponent destroyed.

```
