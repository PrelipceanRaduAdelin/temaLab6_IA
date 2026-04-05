import time
import random
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

V_BASE = 2.0
V_MAX = 4.0
K_SENSOR = 1.0
SENSOR_MAX = 1.2
EMERGENCY_DIST = 0.15

WEIGHTS = [
    (+0.5, -0.5), (+1.0, -1.0), (+2.0, -2.0), (+3.0, -3.0),
    (-3.0, +3.0), (-2.0, +2.0), (-1.0, +1.0), (-0.5, +0.5),
]

def get_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    robot = sim.getObject('/PioneerP3DX')
    lm = sim.getObject('/PioneerP3DX/leftMotor')
    rm = sim.getObject('/PioneerP3DX/rightMotor')
    sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(16)]

    last_pos = sim.getObjectPosition(robot, sim.handle_world)
    last_check_time = time.time()
    stuck_threshold = 0.05

    sim.startSimulation()

    try:
        while True:
            now = time.time()
            v_left = V_BASE
            v_right = V_BASE
            recovery_needed = False

            if now - last_check_time > 2.0:
                curr_pos = sim.getObjectPosition(robot, sim.handle_world)
                dist_moved = get_distance(curr_pos, last_pos)
                
                if dist_moved < stuck_threshold:
                    recovery_needed = True
                
                last_pos = curr_pos
                last_check_time = now

            for i in range(2, 6): 
                res, dist, *_ = sim.readProximitySensor(sensors[i])
                if res and dist < EMERGENCY_DIST:
                    recovery_needed = True
                    break

            if recovery_needed:
                sim.setJointTargetVelocity(lm, -V_BASE)
                sim.setJointTargetVelocity(rm, -V_BASE)
                time.sleep(1.0)
                
                turn_dir = random.choice([-1, 1])
                sim.setJointTargetVelocity(lm, V_BASE * turn_dir)
                sim.setJointTargetVelocity(rm, -V_BASE * turn_dir)
                time.sleep(0.5)
                
                last_pos = sim.getObjectPosition(robot, sim.handle_world)
                last_check_time = time.time()
            else:
                for i in range(8):
                    res, dist, *_ = sim.readProximitySensor(sensors[i])
                    if res:
                        proximity = 1.0 - (dist / SENSOR_MAX)
                        proximity = max(0.0, min(1.0, proximity))
                        v_left  += K_SENSOR * WEIGHTS[i][0] * proximity
                        v_right += K_SENSOR * WEIGHTS[i][1] * proximity

                v_left = max(-V_MAX, min(V_MAX, v_left))
                v_right = max(-V_MAX, min(V_MAX, v_right))
                sim.setJointTargetVelocity(lm, v_left)
                sim.setJointTargetVelocity(rm, v_right)
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        sim.setJointTargetVelocity(lm, 0)
        sim.setJointTargetVelocity(rm, 0)
        sim.stopSimulation()

if __name__ == "__main__":
    main()