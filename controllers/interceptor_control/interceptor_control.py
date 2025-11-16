# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example of Python controller for Mavic patrolling around the house.
   Open the robot window to see the camera view.
   This demonstrates how to go to specific world coordinates using its GPS, imu and gyroscope.
   The drone reaches a given altitude and patrols from waypoint to waypoint."""

from controller import Robot
import sys
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")

# Get drone ID
if len(sys.argv) > 1:
    drone_id = int(sys.argv[1])
else:
    drone_id = 0

print(f"Drone {drone_id} started")

FORMATION_OFFSET = [
    [0, 0],     # leader
    [-1, 1],     # right wing
    [-2, 0],    # rear
    [-1, -1],     # left wing
]

offset = FORMATION_OFFSET[drone_id % len(FORMATION_OFFSET)]
target_x, target_y = offset

CENTER_WAYPOINTS = [[0, -10]]


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 4.0 #3.0        # P constant of the vertical PID.
    K_ROLL_P = 85.0           # P constant of the roll PID.
    K_PITCH_P = 52.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 2.4
    MAX_PITCH_DISTURBANCE = -5
    # Precision between the target position and the robot position in meters
    target_precision = 0.3

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        # Receiver for enemy position broadcasts (emitter will be used by enemy)
        try:
            # swarm drones only need to receive enemy broadcasts
            self.receiver = self.getDevice("receiver")
            self.receiver.enable(self.time_step)
            self.emitter = None
        except Exception:
            # If device missing, disable comms gracefully.
            self.receiver = None
            self.emitter = None

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0
        # enemies: id -> (x, y, timestamp)
        self.enemies = {}
        self.enemy_timeout = 5.0  # seconds to consider an enemy position fresh

    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos

    def _broadcast_position(self, now):
        # Swarm no longer broadcasts its own position for consensus.
        return

    def _process_incoming(self, now):
        if not self.receiver:
            return
        while self.receiver.getQueueLength() > 0:
            data = self.receiver.getData()
            try:
                s = data.decode("utf-8")
                parts = s.strip().split(",")
                # Expect messages formatted as: <id>,<x>,<y>,<z>
                if len(parts) >= 4:
                    nid = int(parts[0])
                    nx = float(parts[1])
                    ny = float(parts[2])
                    nz = float(parts[3])
                    self.enemies[nid] = (nx, ny, nz, now)
                elif len(parts) >= 3:
                    # backward compatible: no z provided
                    nid = int(parts[0])
                    nx = float(parts[1])
                    ny = float(parts[2])
                    nz = None
                    self.enemies[nid] = (nx, ny, nz, now)
            except Exception:
                pass
            self.receiver.nextPacket()

    def _run_consensus(self, now):
        # Consensus removed: swarm no longer computes a centroid of neighbors.
        return

    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """
        # If enemies detected, move_to_target will be called with enemy waypoint from run()

        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):

            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def run(self):
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        # Specify the patrol coordinates
        waypoints = [[x + target_x, y + target_y] for x, y in CENTER_WAYPOINTS]
        print(f"Drone {drone_id} waypoints: {waypoints}")

        # target altitude of the robot in meters
        self.target_altitude = 20

        while self.step(self.time_step) != -1:

            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            now = self.getTime()
            # process incoming enemy broadcasts
            self._process_incoming(now)
            # choose nearest fresh enemy (if any) and override waypoints
            active = []
            for nid, val in self.enemies.items():
                # val is (ex, ey, ez, t) or (ex,ey,None,t)
                if len(val) == 4:
                    ex, ey, ez, t = val
                else:
                    # defensive: skip malformed entries
                    continue
                if now - t <= self.enemy_timeout:
                    active.append((nid, ex, ey, ez))
            if len(active) > 0:
                # pick nearest enemy in 3D when available (fallback to 2D if ez is None)
                def dist3(e):
                    ex, ey, ez = e[1], e[2], e[3]
                    dx = ex - self.current_pose[0]
                    dy = ey - self.current_pose[1]
                    dz = 0 if ez is None else (ez - self.current_pose[2])
                    return dx * dx + dy * dy + dz * dz
                nearest = min(active, key=dist3)
                # nearest is (nid, ex, ey, ez)
                waypoints = [[nearest[1], nearest[2]]]
                if nearest[3] is not None:
                    # set target altitude to the enemy altitude
                    self.target_altitude = nearest[3]

            # Always compute horizontal disturbances periodically so the drone can chase while changing altitude
            if self.getTime() - t1 > 0.1:
                yaw_disturbance, pitch_disturbance = self.move_to_target(waypoints)
                t1 = self.getTime()

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
robot.run()
