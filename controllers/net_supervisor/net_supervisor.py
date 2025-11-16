from controller import Supervisor
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Get drone nodes
interceptor = robot.getFromDef("DRONE2_INTERCEPTOR")
enemy = robot.getFromDef("ENEMY_DRONE")

enemy_translation_field = enemy.getField("translation")
interceptor_translation_field = interceptor.getField("translation")

# Global variables
net_fired = False
net_node = None
NET_SPEED = 12

def spawn_net(position, forward_vector):
    global net_node

    root = robot.getRoot()
    children_field = root.getField("children")

    proto = f'NetProjectile {{ translation {position[0]} {position[1]} {position[2]} }}'

    index = children_field.getCount()
    children_field.importMFNodeFromString(index, proto)
    net_node = children_field.getMFNode(index)

    # Add velocity field
    physics = net_node.getField("physics")
    if physics:
        vel_field = physics.getSFNode().getField("velocity")
        vel_field.setSFVec3f([
            forward_vector[0] * NET_SPEED,
            forward_vector[1] * NET_SPEED,
            forward_vector[2] * NET_SPEED
        ])

def distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def disable_enemy():
    # Remove controller (enemy drops / loses control)
    enemy.getField("controller").setSFString("")
    print("Enemy disabled!")

while robot.step(timestep) != -1:

    # Get current positions
    interceptor_pos = interceptor_translation_field.getSFVec3f()
    enemy_pos = enemy_translation_field.getSFVec3f()

    # Fire condition: interceptor within 5 meters
    if not net_fired and distance(interceptor_pos, enemy_pos) < 5.0:
        print("Interceptor in range — firing net!")

        # Compute forward vector (enemy direction)
        direction = [
            enemy_pos[0] - interceptor_pos[0],
            enemy_pos[1] - interceptor_pos[1],
            enemy_pos[2] - interceptor_pos[2]
        ]
        norm = math.sqrt(sum(d*d for d in direction))
        forward = [d / norm for d in direction]

        spawn_net(interceptor_pos, forward)
        net_fired = True

    # After firing: check collision
    if net_fired and net_node:
        net_pos = net_node.getField("translation").getSFVec3f()
        if distance(net_pos, enemy_pos) < 0.6:
            print("HIT! Net captured enemy drone.")
            disable_enemy()
            break
