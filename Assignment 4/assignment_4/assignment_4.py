import numpy as np
import pybullet as p
import pybullet_data
import time
np.set_printoptions(precision=3)

PLANE_H = 0.03
Y_OBS = np.clip(np.random.normal(loc=0., scale=.2, size=2), -0.2, 0.2)
OBS = [[0.45, Y_OBS[0], PLANE_H], [1., Y_OBS[1], PLANE_H]]
ROBOT_MASS = 0.1


class Env:
    def __init__(self, with_obs=False):
        # initialize the simulator and blocks
        self.physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF('plane.urdf', useFixedBase=True)
        p.changeDynamics(plane_id, -1, lateralFriction=0.99)

        # set camera
        p.resetDebugVisualizerCamera(cameraDistance=2,
                                     cameraYaw=45,  # 45
                                     cameraPitch=-80,  # -89
                                     cameraTargetPosition=[0, 0, 0])

        # set gravity
        p.setGravity(0, 0, 0)

        # add obstacles
        self.with_obs = with_obs
        if with_obs:
            for obs in OBS:
                self.obs = p.loadURDF('/Users/hemanthmurali/Library/CloudStorage/OneDrive-Umich/University of Michigan/Fall24/ROB 599 Intoduction to Robotic Manipulation/Assignment/Assignment 4/assignment_4/cylinder_obs.urdf',
                                      basePosition=obs,
                                      useFixedBase=True)

        # add robot
        if with_obs:
            y0 = 0.
            x0 = 1.75
        else:
            y0 = np.random.randn()
            x0 = np.random.randn()

        self.rob = p.loadURDF('/Users/hemanthmurali/Library/CloudStorage/OneDrive-Umich/University of Michigan/Fall24/ROB 599 Intoduction to Robotic Manipulation/Assignment/Assignment 4/assignment_4/cylinder_robot.urdf',
                              basePosition=[x0, y0, PLANE_H],
                              useFixedBase=False)
        time.sleep(10)

    def simulate(self, max_t=600):
        for sim_step in range(max_t):
            q, v = self.get_state()
            u = dmp_control(q, v)
            if self.with_obs:
                u = u + collision_avoidance(q, v)
            self.apply_control(u)
            p.stepSimulation()
            # print(self.get_state())
            time.sleep(0.01)

    def get_state(self):
        q, ori = p.getBasePositionAndOrientation(self.rob)
        v, w = p.getBaseVelocity(self.rob)

        return np.asarray(q[:2]), np.asarray(v[:2])

    def apply_control(self, u):
        link_id = -1
        force_loc = np.array([0., 0., 0.])
        u_3 = np.append(u, 0.)
        p.applyExternalForce(self.rob,
                             link_id,
                             u_3,
                             force_loc,
                             p.LINK_FRAME)


def dmp_control(q, v, qd=np.zeros((2, )), vd=np.zeros((2, ))):
    """
        The DMP controller
        :param q: np.array((2, )) -- configuration of the robot
        :param v: np.array((2, )) -- velocity of the robot
        :param qd: np.array((2, )) -- desired configuration of the robot
        :param vd: np.array((2, )) -- desired velocity of the robot
        :return: u: np.array((2, )) -- DMP control
    """
    u = np.zeros(2)
    #########################################

    kq = 10.
    kv = 1.5
    u = -kq * (q - qd) - kv * (v - vd)

    #########################################
    return u


def collision_avoidance(q, v, qd=np.array([0., 0.]), OBS=OBS):
    """
        The collision avoidance controller
        :param q: np.array((2, )) -- configuration of the robot
        :param v: np.array((2, )) -- velocity of the robot
        :param qd: np.array((2, )) -- desired configuration of the robot
        :param OBS: (np.array(3,), np.array(3,)) -- position of the obstacles by default is the one specified in this file.
        :return: u: np.array((2, )) -- collision avoidance control
    """
    u = np.zeros(2)
    gamma = 100. # 150.
    beta = 4.5 # 3.5
    #########################################
    
    R = np.array([[0, -1], [1, 0]])
    min_phi = np.inf
    
    for obs in OBS:
        o_minus_x = np.array(obs[:2]) - q
        numerator = o_minus_x.T @ v
    
        if numerator < 1e-5:
            phi = 0
        else:
            denominator = np.linalg.norm(o_minus_x) * np.linalg.norm(v)
            phi = np.arccos(numerator / denominator)
        
        if phi < min_phi:
            min_phi = phi
            
    u = gamma * R @ v * min_phi * np.exp(-beta * np.abs(min_phi))

    #########################################
    return u


# def main(with_obs=False):
def main(with_obs=True):
    env = Env(with_obs)  # env = Env(with_obs=True)
    env.simulate()


if __name__ == "__main__":
    main()

