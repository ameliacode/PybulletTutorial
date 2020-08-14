import gym
from gym import error, spaces, utils
from gym.utils import seeding
import os
import math
import numpy as np
import pybullet as p
import pybullet_data

class BalancebotEnv(gym.Env):
    metadata = {"render.modes":["human"]}

    def __init__(self):
        self._observation=[]
        self.action_space=spaces.Discrete(9) #space object
        #potch, gyto, commanded speed
        self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]),
                                            np.array([math.pi, math.pi, 5]))
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._seed()

    def _step(self, action):
        self._assign_throttle(action)
        p.stepSimulation()
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()

        self._envStepCounter += 1

        return np.array(self._observation), reward, done, {}

    def _assign_throttle(self, action):
        dv = 0.1
        deltav = [-10. * dv, -5. * dv, -2. * dv, -0.1 * dv, 0, 0.1 * dv, 2. * dv, 5. * dv, 10. * dv][action]
        vt = self.vt + deltav
        self.vt = vt
        p.setJointMotorControl2(bodyUniqueId=self.botId,
                                jointIndex=0,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=vt)
        p.setJointMotorControl2(bodyUniqueId=self.botId,
                                jointIndex=1,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=-vt)

    def _compute_observation(self):
        pass

    def _compute_reward(self):
        _, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        return (1-abs(cubeEuler[0]))-0.1 - abs(self.vt - self.vd)*0.01

    def _compute_done(self):
        cubePos,_=p.getBasePositionAndOrientation(self.botId)
        return cubePos[2] < 0.15 or self._envStepCounter >= 1500

    def _reset(self):
        self.vt = 0
        self.vd = 0
        self._envStepCounter = 0

        p.resetSimulation()
        p.setGravity(0,0,-10)
        p.setTimeStep(0.01)

        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0.001]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, "balancebot_simple.xml"),
                                cubeStartPos,
                                cubeStartOrientation)
        self._observation = self._compute_observation()
        return np.array(self._observation)

    def render(self,mode="human",close=False):
        pass

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]