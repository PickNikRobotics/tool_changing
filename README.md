# MoveGroup Tool Changing capability

Description: This package adds a MoveGroupCapability to dynamically switch between a set of configured end-effectors. A separate PlanningRequestAdapter makes sure that only the enabled end-effector is being used for planning. 

<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

[![Build and Test](https://github.com/PickNikRobotics/tool_changing/actions/workflows/build_and_test.yaml/badge.svg)](https://github.com/PickNikRobotics/tool_changing/actions/workflows/build_and_test.yaml)

## Installation

These instructions assume you are running on Ubuntu 20.04, rolling, and the latest MoveIt 2 main branch:
- git clone git@github.com:PickNikRobotics/tool_changing.git

## Usage

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

#### Getting current end-effector

`ros2 service call /get_current_end_effector tool_changing_capability_msgs/srv/GetCurrentEndEffector {}`


#### Change end-effector

`ros2 service call /change_end_effector tool_changing_capability_msgs/srv/ChangeEndEffector end_effector_name:\ \'new_end_effector_name\'\ `

## Setup pre-commit

pre-commit is a tool to automatically run formatting checks on each commit, which saves you from manually running clang-format (or, crucially, from forgetting to run them!).

Install pre-commit like this:

```
pip3 install pre-commit
```

Run this in the top directory of the repo to set up the git hooks:

```
pre-commit install
```
