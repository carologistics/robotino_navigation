# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import yaml


def rewrite_yaml(yaml_path, num_sources):
    with open(yaml_path, "r") as file:
        config = yaml.safe_load(file)

    obstacle_layer = config["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    for i in range(1, num_sources + 1):
        source_name = f"scan_robotinobase{i}"
        obstacle_layer[source_name] = {
            "topic": f"/robotinobase{i}/scan",
            "max_obstacle_height": 2.0,
            "clearing": True,
            "marking": True,
            "data_type": "LaserScan",
            "raytrace_max_range": 1.5,
            "raytrace_min_range": 0.0,
            "obstacle_max_range": 1.5,
            "obstacle_min_range": 0.0,
        }

    with open(yaml_path, "w") as file:
        yaml.safe_dump(config, file)
