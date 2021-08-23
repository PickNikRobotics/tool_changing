### MoveGroup Tool Changing capability

This's package th

#### Usage

To use this package you need to:

1- List all the end-effectors in the URDF file

2- Add the `group` & `end_effector` tags to the SRDF for each end-effector

3- Add `move_group/ToolChangingCapability` as a move_group node capability

4- Add `initial_end_effector` parameter to move_group node to set the initial end-effector
```py
move_group_capabilities = {
    "capabilities": """Capability1 Capability2 ... move_group/ToolChangingCapability"""
}
run_move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    parameters=[
        ...
        move_group_capabilities,
        {"initial_end_effector": "end_effector_name"}
    ],
)
```

5- Add `default_planner_request_adapters/ToolChangingAdapter` to the planning pipeline's adapters
```py
ompl_planning_pipeline_config = {
    "planning_pipelines": ["planning_pipeline"],
    "planning_pipeline": {
        ...
        "request_adapters": """Adapter1 Adapter2 ... default_planner_request_adapters/ToolChangingAdapter""",
    },
}
```

Now launch your move_group instance

##### Getting current end-effector

`ros2 service call /get_current_end_effector tool_changing_capability_msgs/srv/GetCurrentEndEffector {}`


##### Change end-effector

`ros2 service call /change_end_effector tool_changing_capability_msgs/srv/ChangeEndEffector end_effector_name:\ \'new_end_effector_name\'\ `
