# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import itertools
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory


def _shared_data_directories():
    controller_share = Path(get_package_share_directory("robot_controller"))
    description_share = Path(get_package_share_directory("robot_description"))
    return controller_share, description_share


def _available_robot_models():
    controller_share, description_share = _shared_data_directories()
    config_dir = controller_share / "config"
    urdf_dir = description_share / "urdf"

    if not config_dir.is_dir() or not urdf_dir.is_dir():
        return []

    controller_models = {path.name for path in config_dir.iterdir() if path.is_dir()}
    urdf_models = {
        path.name.replace(".urdf.xacro", "") for path in urdf_dir.glob("*.urdf.xacro")
    }
    return sorted(controller_models & urdf_models)


def test_robot_description_parsing():
    robot_model_values = _available_robot_models()
    assert robot_model_values, "Expected controller and URDF data for at least one robot"

    mecanum_values = ["True", "False"]
    use_sim_values = ["True", "False"]

    all_combinations = list(
        itertools.product(
            robot_model_values,
            mecanum_values,
            use_sim_values,
        )
    )

    for combination in all_combinations:
        robot_model, mecanum, use_sim = combination

        controller_share, description_share = _shared_data_directories()
        controller_config_filename = (
            "mecanum_drive_controller.yaml" if mecanum else "diff_drive_controller.yaml"
        )
        controller_config = (
            controller_share / "config" / robot_model / controller_config_filename
        )

        mappings = {
            "controller_config": str(controller_config),
            "mecanum": mecanum,
            "use_sim": use_sim,
        }
        xacro_path = description_share / "urdf" / f"{robot_model}.urdf.xacro"
        try:
            xacro.process_file(str(xacro_path), mappings=mappings)
        except xacro.XacroException as e:
            assert (
                False
            ), f"xacro parsing failed: {str(e)} for mecanum: {mecanum}, use_sim: {use_sim}"
