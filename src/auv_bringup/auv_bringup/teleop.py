import subprocess

def main():

    nodes = [

        ["ros2", "run", "auv_sensors", "imu_node"],

        ["ros2", "run", "auv_sensors", "depth_sensor"],

        ["ros2", "run", "auv_sensors", "battery_monitor"],

        ["ros2", "run", "auv_sensors", "leak_sensor"],

        ["ros2", "run", "auv_teleop", "receiver"],

        ["ros2", "run", "auv_teleop", "auv_control"],
    ]

    procs = []

    for node in nodes:
        procs.append(
            subprocess.Popen(node)
        )

    try:

        for p in procs:
            p.wait()

    except KeyboardInterrupt:

        for p in procs:
            p.terminate()
