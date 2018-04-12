"""
Implementation the inverted pendulum where the base is fixed and a weight shifts at the top of the pendulum.
"""

import math

import numpy as np

import gym
from gym import spaces, logger
from gym.utils import seeding

class CartTableAngularEnv(gym.Env):
    """
    State:

        yeta = angle of hip joint, 0 means perfectly aligned with pole
        yeta_dot = angular velocity of hip joint
        theta = angle of pole from the ground
        theta_dot = angular velocity of the pole
    """
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    def __init__(self):
        self.gravity = 9.8 # meter/s^2
        self.masscart = 300. # gram
        self.masspole = 50. # gram
        self.masspole2 = self.masspole/2.
        self.total_mass = (self.masspole + self.masspole2 + self.masscart)
        self.length = 0.5 # meters, actually half the pole's length
        self.length2 = self.length/2. # distance of the mass from the top of the second pole
        self.polemass_length = (self.masspole * self.length) # kg * meter
        # self.force_mag = 10.0
        self.tau = 0.02  # seconds between state updates

        self.servo_max_angle = 135 * math.pi / 180 # 135 degrees
        self.servo_torque = 32.4 # kg/cm
        self.servo_torque = self.servo_torque * 1000/1. * 1/100. # g/m
        
        self.cog_offset = self.length2/2.

        # Angle at which to fail the episode
        # self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.theta_threshold_radians = 90 * math.pi / 180
        self.yeta_threshold = 90 * math.pi / 180

        # Angle limit set to 2 * theta_threshold_radians so failing observation is still within bounds
        high = np.array([
            self.yeta_threshold * 2,
            np.finfo(np.float32).max,
            self.theta_threshold_radians * 2,
            np.finfo(np.float32).max])

        self.action_space = spaces.Discrete(2) # only two actions, move left or right
        self.observation_space = spaces.Box(-high, high) # pylint: disable=invalid-unary-operand-type

        self.seed()
        self.viewer = None
        self.state = None

        self.yeta_error = 0.
        self.yeta_error_cnt = 0

        self.steps_beyond_done = None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        state = self.state
        # print('state:', state)
        yeta, yeta_dot, theta, theta_dot = state

        if action == 0:
            direction = 1 # clockwise
        else:
            direction = -1 # counter-clockwise

        # force = self.force_mag if action == 1 else -self.force_mag
        # costheta = math.cos(theta)
        # sintheta = math.sin(theta)

        # F = m*a => a = F/m
        #xacc  = temp - self.polemass_length * thetaacc * costheta / self.total_mass
        # print('-'*80)
        # print('yeta deg:', yeta*180/math.pi)
        # print('theta deg:', theta*180/math.pi)

        # Calculate the initial 2D position of the mass atop the second pole due to yeta.
        # x' = x*cos(theta) - y*sin(theta)
        # y' = x*sin(theta) + y*cos(theta)
        
        # Without offset.
        xp20 = -self.length2 * math.sin(yeta)
        yp20 = +self.length2 * math.cos(yeta)
        
        # With offset.
        xp2 = self.cog_offset * math.cos(theta) - self.length2 * math.sin(theta)
        yp2 = self.cog_offset * math.sin(theta) + self.length2 * math.cos(theta)
        # print('pole2.end.0:', xp2, yp2)

        # Translate second pole up to top of first.
        yp2 += self.length
        # print('pole2.end.1:', xp2, yp2)

        # Calculate the final 2D position of the mass on the second pole due to theta, including mass offset.
        # x' = x*cos(theta) - y*sin(theta)
        # y' = x*sin(theta) + y*cos(theta)
        xp1 = xp2 * math.cos(theta) - yp2 * math.sin(theta)
        yp1 = xp2 * math.sin(theta) + yp2 * math.cos(theta)
        # print('pole2.end.2:', xp1, yp1)
        
        # Calculate final position of second pole tip, without mass offset.
        xp10 = xp20 * math.cos(theta) - yp20 * math.sin(theta)
        yp10 = xp20 * math.sin(theta) + yp20 * math.cos(theta)
        
        # Calculate position of the hip.
        hx = - self.length * math.sin(theta)
        hy = + self.length * math.cos(theta)

        # Calculate length of virtual arm.
        arm_length = math.sqrt(xp1**2 + yp1**2)
        # print('arm_length:', arm_length)

        # Calculate overall angle of virtual arm offset from table center.
        # theta_p = math.atan(x/float(self.length))

        # Calculate total angle of virtual arm from pivot point to the cart.
        # arm_theta = theta + theta_p
        arm_theta = math.atan(xp1/arm_length)
        # print('arm_theta deg:', arm_theta*180/math.pi)

        # Calculate torque of cart acting on virtual arm.
        # torque = radius * force * sin(theta)
        arm_torque = arm_length * self.masscart * self.gravity * math.sin(arm_theta) #TODO:include mass of poles?
        # print('arm_torque:', arm_torque)

        # Calculate first pole's angular acceleration due to torque.
        # torque = moment of inertia * angular acceleration => angular acceleration = torque / moment of inertia
        # If we treat the cart as a point mass rotating around the pivot => I = m*r^2
        theta_acc = arm_torque/(self.masscart * arm_length**2) #?
        # print('theta_acc:', theta_acc)

        # Calculate angular acceleration of the cart due to applied force.
        # a = (V1 - V0)/(t1 - t0)
        # a = F/m
        # I = m*r^2
        # torque = r * F * sin(theta) = moment of inertia * angular acceleration
        # xacc = force * self.masscart #TODO:include gravity component as table tilts?
        # torque = moment of inertia * angular acceleration => angular acceleration = torque / moment of inertia
        # pivot => I = m*r^2
        yeta_acc = (self.servo_torque * direction) / (self.masscart * (self.length2**2))

        # Calculate change in second pole's angle due to angular velocity over time.
        # Df = Di + t * V
        new_yeta = yeta + self.tau * yeta_dot
        # Enforce hard stops of the servo.
        new_yeta = max(min(new_yeta, self.servo_max_angle), -self.servo_max_angle)
        # print('new_yeta:', new_yeta)

        # Calculate change in cart's velocity due to acceleration over time.
        # Vf = Vi + t * a
        if new_yeta == yeta:
            # If we've reached an end-stop, then there's no more angular acceleration.
            new_yeta_dot = 0
        else:
            new_yeta_dot = yeta_dot + self.tau * yeta_acc
        # print('new_yeta_dot:', new_yeta_dot)

        # Calculate change in pole's angle due to angular velocity over time.
        new_theta = theta + self.tau * theta_dot
        # print('new_theta deg:', new_theta*180/math.pi)

        # Calculate change in pole's angular velocity due to angular acceleration over time.
        new_theta_dot = theta_dot + self.tau * theta_acc
        # print('new_theta_dot:', new_theta_dot)

        self.state = (new_yeta, new_yeta_dot, new_theta, new_theta_dot)

        done = yeta < -self.yeta_threshold or yeta > self.yeta_threshold or theta < -self.theta_threshold_radians or theta > self.theta_threshold_radians
        done = bool(done)

        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn("You are calling 'step()' even though this environment has already returned done = True. "
                    "You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        # 1=no angle, 0=worse angle
        # off_center_score = 1 - abs(theta)/(math.pi/2.)

        yeta_error_mean = 0

        # self.yeta_error += abs(yeta)
        # self.yeta_error_cnt += 1
        # yeta_error_mean = 1 - self.yeta_error/self.yeta_error_cnt/math.pi # should be bounded to [0:1]
        
        # Reward based on keeping the second pole pointing straight up.
        # self.yeta_error += abs(xp10 - hx)
        # self.yeta_error_cnt += 1
        # yeta_error_mean = 1 - self.yeta_error/self.yeta_error_cnt/self.length2 # should be bounded to [0:1]

        # weight = 1.0 # only look at raw step rewarad
        # weight = 0.33 # step count 1/3 as important as angle
        # weight = 0.5 # step count equally as important as angle
        # final_score = weight*reward + (1-weight)*off_center_score
        # final_score = reward + off_center_score*2
        final_score = reward + yeta_error_mean
        # final_score = reward*5/10. + off_center_score*5/10.

        return np.array(self.state), final_score, done, {}

    def reset(self):
        self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
        # self.state = self.np_random.uniform(low=-0.05*2, high=0.05*2, size=(4,))
        self.steps_beyond_done = None

        self.yeta_error = 0.
        self.yeta_error_cnt = 0

        return np.array(self.state)

    def render(self, mode='human'):
        screen_width = 600
        screen_height = 400

        world_width = self.yeta_threshold * 2
        scale = screen_width/world_width
        carty = 100 # TOP OF CART
        polewidth = 10.0
        polelen = scale * 1.0
        cartwidth = 50.0
        cartheight = 30.0
        
        weight_offset_x = self.cog_offset * scale

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)

            self.trans = rendering.Transform()

            # Draw pole.
            axleoffset = cartheight/4.0
            l, r, t, b = -polewidth/2, polewidth/2, polelen, -polewidth/2
            pole = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            pole.set_color(.8, .6, .4)
            self.poletrans = rendering.Transform(translation=(0, axleoffset))
            pole.add_attr(self.poletrans)
            pole.add_attr(self.trans)
            self.viewer.add_geom(pole)

            # Draw pole axle.
            axle = rendering.make_circle(polewidth/2)
            axle.add_attr(self.poletrans)
            axle.add_attr(self.trans)
            axle.set_color(.5, .5, .8)
            self.viewer.add_geom(axle)

            # Draw second pole.
            axleoffset = cartheight/4.0
            l, r, t, b = -polewidth/2, polewidth/2, polelen/2.-polewidth/2, -polewidth/2
            pole2 = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            pole2.set_color(.8, .6, .4)
            self.carttrans = rendering.Transform()
            pole2.add_attr(self.carttrans)
            pole2.add_attr(self.poletrans)
            pole2.add_attr(self.trans)
            self.viewer.add_geom(pole2)
            
            # Draw second pole axle.
            axle2 = rendering.make_circle(polewidth/2)
            axle2.add_attr(self.carttrans)
            axle2.add_attr(self.poletrans)
            axle2.add_attr(self.trans)
            axle2.set_color(.5, .5, .8)
            self.viewer.add_geom(axle2)
            
            # top weight.
            weight = rendering.make_circle(polewidth*2)
            weight.add_attr(rendering.Transform(translation=(weight_offset_x, polelen/2)))
            weight.add_attr(self.carttrans)
            weight.add_attr(self.poletrans)
            weight.add_attr(self.trans)
            weight.set_color(.5, .5, .8)
            self.viewer.add_geom(weight)

            # Draw cart.
            # # l, r, t, b = -cartwidth/2, cartwidth/2, cartheight/2, -cartheight/2
            # # cart = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            # # self.carttrans = rendering.Transform()
            # # cart.add_attr(self.carttrans)
            # # cart.add_attr(self.poletrans)
            # # cart.add_attr(self.trans)
            # # self.viewer.add_geom(cart)

            # Draw track.
            self.track = rendering.Line((0, carty), (screen_width, carty))
            self.track.set_color(0, 0, 0)
            self.viewer.add_geom(self.track)

        if self.state is None:
            return None

        x = self.state
        self.carttrans.set_translation(0, polelen/2)
        self.carttrans.set_rotation(-x[0])
        self.carttrans.set_translation(0, polelen)
        cartx = screen_width/2.0 # MIDDLE OF THE SCREEN
        self.trans.set_translation(cartx, carty)
        self.poletrans.set_rotation(-x[2])

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.viewer:
            self.viewer.close()
