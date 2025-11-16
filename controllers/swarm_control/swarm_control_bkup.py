# swarm_controller.py
# Works with Webots DJI Mavic PROTO
# Drone 1 chases enemy; drones 2-4 follow drone 1 in square formation.

from controller import Robot
import sys
import numpy as np

# Get drone ID (if passed from Webots)
if len(sys.argv) > 1:
    drone_id = int(sys.argv[1])
else:
    drone_id = 0

def clamp(value, low, high):
    return min(max(value, low), high)

class Mavic(Robot):
    # Same constants as official Webots Mavic controller
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0

    # Formation offsets for drones 2-5 relative to leader
    # Arrange followers in a 1m x 1m square centered 1.0m behind the leader (negative x)
    # Corners (relative to leader): (-1.5, -0.5), (-1.5, 0.5), (-0.5, -0.5), (-0.5, 0.5)
    formation_offsets = {
        2: [-1.5, -0.5],  # back-left
        3: [-1.5,  0.5],  # back-right
        4: [-0.5, -0.5],  # front-left (of the square, closer to leader)
        5: [-0.5,  0.5],  # front-right
    }

    def __init__(self):
        super(Mavic, self).__init__()

        self.time_step = int(self.getBasicTimeStep())

        # Devices
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)

        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        # Motors
        self.fl = self.getDevice("front left propeller")
        self.fr = self.getDevice("front right propeller")
        self.rl = self.getDevice("rear left propeller")
        self.rr = self.getDevice("rear right propeller")

        for m in [self.fl, self.fr, self.rl, self.rr]:
            m.setPosition(float('inf'))
            m.setVelocity(1)

        # Receiver for enemy or leader pose
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.time_step)

        self.enemy_pos = None   # for leader
        self.leader_pos = None  # for followers
        self.target_altitude = 6  # default

    def read_messages(self):
        """Read receiver messages: enemy for leader, leader for followers."""
        while self.receiver.getQueueLength() > 0:
            msg = self.receiver.getString()
            self.receiver.nextPacket()
            try:
                parts = msg.split(",")
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                if drone_id == 1:
                    self.enemy_pos = [x, y, z]  # leader tracks enemy
                else:
                    self.leader_pos = [x, y, z]  # followers track leader
            except:
                pass

    def send_position(self, pos):
        """Send current position to swarm (for leader)."""
        # Only leader broadcasts
        if drone_id == 1:
            try:
                msg = f"{drone_id},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}"
                self.getDevice("emitter").send(msg.encode("utf-8"))
            except:
                pass

    def run(self):
        while self.step(self.time_step) != -1:
            self.read_messages()

            # Sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x, y, z = self.gps.getValues()
            roll_rate, pitch_rate, _ = self.gyro.getValues()

            # Default hover
            yaw_disturbance = 0
            pitch_disturbance = 0
            roll_disturbance = 0

            target_x, target_y, target_z = x, y, z

            if drone_id == 1 and self.enemy_pos:
                # Leader chases enemy
                ex, ey, ez = self.enemy_pos
                dx = ex - x
                dy = ey - y
                dz = ez - z

                target_x, target_y, target_z = ex, ey, ez

                desired_yaw = np.arctan2(dy, dx)
                yaw_error = (desired_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
                yaw_disturbance = 1.2 * yaw_error

                dist = np.sqrt(dx*dx + dy*dy + dz*dz)
                pitch_disturbance = clamp(-2.0 * (dist / 10.0), -3.0, -0.5)
                self.target_altitude = ez

            elif drone_id > 1 and self.leader_pos:
                # Followers maintain formation
                lx, ly, lz = self.leader_pos
                offset = self.formation_offsets[drone_id]
                target_x = lx + offset[0]
                target_y = ly + offset[1]
                target_z = lz

                dx = target_x - x
                dy = target_y - y
                dz = target_z - z

                desired_yaw = np.arctan2(dy, dx)
                yaw_error = (desired_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
                yaw_disturbance = 1.0 * yaw_error
                pitch_disturbance = clamp(-1.0 * np.sqrt(dx*dx + dy*dy), -2.0, -0.5)
                self.target_altitude = target_z

            # Rotor thrust control
            error_alt = self.target_altitude - z + self.K_VERTICAL_OFFSET
            vertical_input = self.K_VERTICAL_P * (error_alt ** 3)

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_rate + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_rate + pitch_disturbance
            yaw_input = yaw_disturbance

            fl_cmd = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            fr_cmd = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rl_cmd = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rr_cmd = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.fl.setVelocity(fl_cmd)
            self.fr.setVelocity(-fr_cmd)
            self.rl.setVelocity(-rl_cmd)
            self.rr.setVelocity(rr_cmd)

            # Broadcast leader's position
            if drone_id == 1:
                self.send_position([x, y, z])

# start
robot = Mavic()
robot.run()
