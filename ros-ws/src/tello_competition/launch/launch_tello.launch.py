#!/usr/bin/env python3

import subprocess

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, Shutdown
from launch_ros.actions import Node

SSID_OBJETIVO = "TELLO-A04B3A"
#SSID_OBJETIVO = "ConectaUACJ"
# SSID_OBJETIVO = "MotherBase"


def verificar_ssid(context, *args, **kwargs):
    try:
        resultado = subprocess.run(
            ["iwgetid", "-r"], capture_output=True, text=True, check=True
        )
        ssid_actual = resultado.stdout.strip()
        if ssid_actual == SSID_OBJETIVO:
            return [LogInfo(msg=f"Conectado a la red WiFi correcta: {ssid_actual}")]
        else:
            return [
                LogInfo(
                    msg=f"Error: Conectado a la red WiFi incorrecta: {ssid_actual}. Se esperaba: {SSID_OBJETIVO}"
                ),
                Shutdown(),
            ]
    except subprocess.CalledProcessError as e:
        return [
            LogInfo(msg=f"Error al obtener el SSID de la red WiFi: {e}"),
            Shutdown(),
        ]

def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    # Check if connected to Tello WiFi network
    ld.add_action(OpaqueFunction(function=verificar_ssid))

    tello_driver_node: Node = Node(
        package="tello",
        executable="tello",
        output="screen",
        namespace="/",
        name="tello",
        parameters=[
            {"connect_timeout": 10.0},
            {"tello_ip": "192.168.10.1"},
            {"tf_base": "map"},
            {"tf_drone": "drone"},
        ],
        remappings=[("/image_raw", "/camera"), ("/imu", "/imu/data_raw")],
        respawn=True,
    )

    ld.add_action(tello_driver_node)

    tello_control_node: Node = Node(
        package="tello_control",
        executable="tello_control",
        output="screen",
        namespace="/",
        name="control",
        respawn=False,
    )

    ld.add_action(tello_control_node)

    tello_tf_node: Node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="/",
        name="tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "drone",
        ],
        respawn=True,
    )

    ld.add_action(tello_tf_node)

    rviz_node: Node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        namespace="/",
        name="rviz",
        arguments=["-d", "/home/xtal/ros2_ws/src/tello_competition/rviz/tello.rviz"],
        respawn=True,
    )

    ld.add_action(rviz_node)

    imu_filter_node: Node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        namespace="/",
        name="imu_filter",
        parameters=[{"use_mag": False}, {"use_gyro": True}, {"use_accel": True}],
        respawn=True,
    )

    ld.add_action(imu_filter_node)

    return ld
