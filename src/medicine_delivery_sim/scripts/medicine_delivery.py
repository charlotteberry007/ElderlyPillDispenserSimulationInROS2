#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# Fixed delivery locations. These are intentionally kept here because the rooms
# are fixed in this project; a separate config file would add complexity without
# giving us much benefit.
ROOMS = [
    {"name": "Room 1", "x": -6.0, "y": 2.15, "yaw": math.pi / 2},
    {"name": "Room 2", "x": -2.0, "y": 2.15, "yaw": math.pi / 2},
    {"name": "Room 3", "x":  2.0, "y": 2.15, "yaw": math.pi / 2},
    {"name": "Room 4", "x":  6.0, "y": 2.15, "yaw": math.pi / 2},
]

DOCK = {"name": "Dock", "x": -7.0, "y": 0.0, "yaw": 0.0}

# Patient cylinders are about 1.0 m in front of the robot at each room waypoint.
# The narrow angular gate avoids seeing the bed at the side of the room.
DETECTION_HALF_ANGLE_DEG = 12.0
DETECTION_MIN_RANGE_M = 0.65
DETECTION_MAX_RANGE_M = 1.35
MIN_CONTIGUOUS_HITS = 4


class MedicineDeliveryRobot(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='medicine_delivery_robot')
        self.latest_scan = None
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.dispense_pub = self.create_publisher(String, '/medicine_dispensed', 10)

    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _wait_for_scan(self, timeout_sec: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_scan is not None:
                return True
        return False

    def person_present(self) -> bool:
        """Detect the simulated patient cylinder using only the front LiDAR scan."""
        if not self._wait_for_scan():
            self.get_logger().warning('No /scan data received; treating room as empty.')
            return False

        scan = self.latest_scan
        half_angle = math.radians(DETECTION_HALF_ANGLE_DEG)
        longest_run = 0
        current_run = 0
        matching_ranges = []

        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > half_angle:
                continue

            valid = (
                math.isfinite(distance)
                and DETECTION_MIN_RANGE_M <= distance <= DETECTION_MAX_RANGE_M
            )

            if valid:
                current_run += 1
                longest_run = max(longest_run, current_run)
                matching_ranges.append(distance)
            else:
                current_run = 0

        if matching_ranges:
            nearest = min(matching_ranges)
            self.get_logger().info(
                f'LiDAR front ROI: nearest={nearest:.2f} m, longest cluster={longest_run} beams'
            )
        else:
            self.get_logger().info('LiDAR front ROI: no object in patient distance window.')

        return longest_run >= MIN_CONTIGUOUS_HITS

    def navigate_to(self, location: dict) -> bool:
        self.get_logger().info(f'Navigating to {location["name"]} ...')
        self.goToPose(self._pose(location['x'], location['y'], location['yaw']))

        while rclpy.ok() and not self.isTaskComplete():
            # isTaskComplete() services the navigator futures; spin_once also lets
            # our /scan subscription continue updating during navigation.
            rclpy.spin_once(self, timeout_sec=0.05)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Arrived at {location["name"]}.')
            return True

        if result == TaskResult.CANCELED:
            self.get_logger().warning(f'Navigation to {location["name"]} was canceled.')
        else:
            self.get_logger().error(f'Navigation to {location["name"]} failed.')
        return False

    def dispense(self, room_name: str):
        msg = String()
        msg.data = room_name
        self.dispense_pub.publish(msg)
        self.get_logger().info(f'MEDICINE DISPENSED at {room_name}')
        time.sleep(1.5)

    def run_mission(self):
        # The simulated robot is spawned at the docking pose, so seed AMCL there.
        self.setInitialPose(self._pose(DOCK['x'], DOCK['y'], DOCK['yaw']))
        self.get_logger().info('Waiting for Nav2 + AMCL to become active ...')
        self.waitUntilNav2Active()
        self.get_logger().info('Nav2 active. Starting medicine delivery route.')

        delivered = 0
        skipped = 0

        for room in ROOMS:
            if not self.navigate_to(room):
                skipped += 1
                continue

            # Allow the final pose to settle before evaluating the scan.
            settle_until = time.monotonic() + 0.8
            while rclpy.ok() and time.monotonic() < settle_until:
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.person_present():
                self.get_logger().info(f'Patient detected in {room["name"]}.')
                self.dispense(room['name'])
                delivered += 1
            else:
                self.get_logger().info(f'No patient detected in {room["name"]}; moving on.')
                skipped += 1

        self.navigate_to(DOCK)
        self.get_logger().info(
            f'Mission complete: dispensed={delivered}, skipped/failed={skipped}. Returned to dock.'
        )


def main(args=None):
    rclpy.init(args=args)
    navigator = MedicineDeliveryRobot()
    try:
        navigator.run_mission()
    except KeyboardInterrupt:
        navigator.get_logger().info('Mission interrupted by user.')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
