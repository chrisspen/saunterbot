"""
Implementation the inverted pendulum where the base is fixed and a weight shifts at the top of the pendulum.
"""

import sys
import math

import numpy as np

import gym
from gym import spaces, logger
from gym.utils import seeding

class Point(object):

    def __init__(self, x=0.0, y=0.0, **kwargs):
        self.x = x
        self.y = y
        self.__dict__.update(kwargs)

    def __str__(self):
        return 'Point(%s, %s)' % (self.x, self.y)
        
    def copy(self):
        return type(self)(**self.__dict__)

class CartTableAngularEnv(gym.Env):
    """
    State:

        yeta = angle of hip joint, 0 means perfectly aligned with pole
        yeta_dot = angular velocity of hip joint
        theta = angle of pole from the ground, 0 means perpendicular to ground
        theta_dot = angular velocity of the pole
        
    Reference:
    
        Point A is the top of the first pole, where it meets the second pole.
        Point B is the top of the second pole, where it meets the weight.
        Point C is the center of the weight.

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
        
        # length of main pole, in meters, actually half the pole's length
        self.length = 0.5
        
        # distance of the mass from the top of the second pole
        self.length2 = self.length/2.
        
        self.polemass_length = (self.masspole * self.length) # kg * meter
        
        # self.force_mag = 10.0
        
        # seconds between state updates
        self.tau = 0.02

        # The maximum range of the servo left or right. The total range is twice this.
        self.servo_max_angle = 135 * math.pi / 180 # 135 degrees
        
        # The amount of torque applied by the servo at the hip.
        self.servo_torque = 32.4 # kg/cm
        self.servo_torque = self.servo_torque * 1000/1. * 1/100. # g/m
        
        self.cog_offset = self.length2/2.

        # Leg angle at which to fail the episode.
        # self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.theta_threshold_radians = 89 * math.pi / 180
        
        # Hip angle at which to fail the episode.
        self.yeta_threshold = 130 * math.pi / 180

        # only two actions, move left or right.
        #self.action_space = spaces.Discrete(2)
        self.set_actions(2)

        # Angle limit set to 2 * theta_threshold_radians so failing observation is still within bounds
        high = np.array([
            self.yeta_threshold * 2,
            np.finfo(np.float32).max,
            self.theta_threshold_radians * 2,
            np.finfo(np.float32).max])
        
        self.observation_space = spaces.Box(-high, high) # pylint: disable=invalid-unary-operand-type

        self.seed()
        self.viewer = None
        self.state = None

        self.yeta_error = 0.
        self.yeta_error_cnt = 0

        self.steps_beyond_done = None
        
        self.verbose = False
    
    def set_actions(self, actions=2):
        self.actions = actions
        self.action_space = spaces.Discrete(actions)

    def vprint(self, *args):
        if self.verbose:
            print(' '.join(map(str, args)))

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
        # self.state = self.np_random.uniform(low=-0.05*2, high=0.05*2, size=(4,))
        self.steps_beyond_done = None

        # Variables for tracking the mean-absolute-error of yeta.
        self.yeta_error = 0.
        self.yeta_error_cnt = 0
        
        self.pole2_error = 0.
        self.pole2_error_cnt = 0

        return np.array(self.state)

    def step(self, action):
        self.vprint('-'*80)
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        state = list(self.state)
        self.vprint('state:', state)
        yeta, yeta_dot, theta, theta_dot = state
        self.vprint('state: yeta=%s, yeta_dot=%s, theta=%s, theta_dot=%s' % (yeta, yeta_dot, theta, theta_dot))

        self.vprint('actions:', self.actions)
        self.vprint('action:', action)
        if self.actions == 2:
            if action == 0:
                direction = +1 # clockwise
            else:
                direction = -1 # counter-clockwise
        elif self.actions == 3:
            if action == 0:
                direction = +1 # clockwise
            elif action == 1:
                direction = 0 # stationary
            else:
                direction = -1 # counter-clockwise
        else:
            raise NotImplementedError

        # force = self.force_mag if action == 1 else -self.force_mag
        # costheta = math.cos(theta)
        # sintheta = math.sin(theta)

        # F = m*a => a = F/m
        #xacc  = temp - self.polemass_length * thetaacc * costheta / self.total_mass
        # print('-'*80)
        # print('yeta deg:', yeta*180/math.pi)
        # print('theta deg:', theta*180/math.pi)

        # x' = x*cos(theta) - y*sin(theta)
        # y' = x*sin(theta) + y*cos(theta)
        
        # Calculate the initial 2D position of the mass atop the second pole due to yeta, without offset.
        c1 = Point(
            0 * math.cos(yeta) - self.length2 * math.sin(yeta), # xp20
            0 * math.cos(yeta) + self.length2 * math.cos(yeta) # yp20
        )
        self.vprint('c1:', c1)
        
        # Calculate the initial 2D position of the mass atop the second pole due to yeta, with offset.
        c2 = Point(
            self.cog_offset * math.cos(yeta) - self.length2 * math.sin(yeta), # xp2
            self.cog_offset * math.sin(yeta) + self.length2 * math.cos(yeta) # yp2
        )
        self.vprint('c2:', c2)

        # Translate weight-offset point on the second pole up to top of first.
        c3 = c2.copy()
        c3.y += self.length
        self.vprint('c3:', c3)
        # print('pole2.end.1:', xp2, yp2)

        # Calculate the final 2D position of the mass on the second pole due to theta, including mass offset.
        # x' = x*cos(theta) - y*sin(theta)
        # y' = x*sin(theta) + y*cos(theta)
        c4 = Point(
            c3.x * math.cos(theta) - c3.y * math.sin(theta), # xp1
            c3.x * math.sin(theta) + c3.y * math.cos(theta) # yp1
        )
        self.vprint('c4=C=weight:', c4)
        # print('pole2.end.2:', xp1, yp1)
        
        # Calculate final position of second pole tip, without mass offset, translated on top of the first pole.
        B = c5 = Point(
            c1.x * math.cos(theta) - (c1.y + self.length) * math.sin(theta), # xp10
            c1.x * math.sin(theta) + (c1.y + self.length) * math.cos(theta) # yp10
        )
        self.vprint('c5=B=head:', c5)
        # print('b=%s, %s' % (xp10, yp10))
        
        # Calculate final position of the hip.
        A = c6 = Point(
            0 * math.cos(theta) - self.length * math.sin(theta), # hx
            0 * math.cos(theta) + self.length * math.cos(theta) # hy
        )
        self.vprint('c6=A=hip:', c6)

        # Calculate length of virtual arm going from mass to origin.
        arm_length = math.sqrt(c4.x**2 + c4.y**2)
        self.vprint('arm_length:', arm_length)

        # Calculate overall angle of virtual arm offset from table center.
        # theta_p = math.atan(x/float(self.length))

        # Calculate total angle of virtual arm from pivot point to the cart.
        # tan(angle) = opposite/hypotenuse => angle = atan(opposite/hypotenuse)
        arm_theta = math.atan(c4.x/arm_length)
        self.vprint('arm_theta deg:', arm_theta*180/math.pi)

        # Calculate torque of weight acting on virtual arm.
        # torque = radius * force * sin(theta) = radius * mass * gravity * sin(angle)
        arm_torque = arm_length * self.masscart * self.gravity * math.sin(arm_theta) #TODO:include mass of poles?
        self.vprint('arm_torque:', arm_torque)
        # if abs(arm_theta*180/math.pi) > 89:
            # sys.exit(1)

        # Calculate first pole's angular acceleration due to torque.
        # torque = moment of inertia * angular acceleration => angular acceleration = torque / moment of inertia
        # If we treat the cart as a point mass rotating around the pivot => I = m*r^2
        theta_acc = -arm_torque/(self.masscart * arm_length**2) #?
        self.vprint('theta_acc:', theta_acc)

        # Calculate angular acceleration of the cart due to applied force.
        # a = (V1 - V0)/(t1 - t0)
        # a = F/m
        # I = m*r^2
        # torque = r * F * sin(theta) = moment of inertia * angular acceleration
        # torque = moment of inertia * angular acceleration => angular acceleration = torque / moment of inertia
        # pivot => I = m*r^2
        yeta_acc = (self.servo_torque * direction) / (self.masscart * (self.length2**2))

        # Calculate change in second pole's angle due to angular velocity over time.
        # Df = Di + t * V
        new_yeta = yeta + self.tau * yeta_dot
        # Enforce hard stops of the servo.
        new_yeta = max(min(new_yeta, self.servo_max_angle), -self.servo_max_angle)
        # print('new_yeta:', new_yeta)
        self.vprint('new_yeta deg:', new_yeta*180/math.pi)
        # if abs(new_yeta) > math.pi/2:
            # sys.exit(1)

        # Calculate change in cart's velocity due to acceleration over time.
        # Vf = Vi + t * a
        if new_yeta == yeta:
            # If we've reached an end-stop, then there's no more angular acceleration.
            new_yeta_dot = 0
        else:
            new_yeta_dot = yeta_dot + self.tau * yeta_acc
        self.vprint('new_yeta_dot:', new_yeta_dot)

        # Calculate change in pole's angle due to angular velocity over time.
        new_theta = theta + self.tau * theta_dot
        # Enforce hard stop imposed by the ground.
        new_theta = min(max(new_theta, -math.pi/2), math.pi/2)
        self.vprint('new_theta deg:', new_theta*180/math.pi)

        # Calculate change in pole's angular velocity due to angular acceleration over time.
        if theta == new_theta:
            # Angle has not changed, meaning we've come to a stop, so velocity/acceleration stops.
            new_theta_dot = theta_dot
        else:
            new_theta_dot = theta_dot + self.tau * theta_acc
        self.vprint('new_theta_dot:', new_theta_dot)
        # if abs(new_theta*180/math.pi) >= 90:
            # sys.exit(1)

        self.state = (new_yeta, new_yeta_dot, new_theta, new_theta_dot)

        # done = yeta < -self.yeta_threshold or yeta > self.yeta_threshold or theta < -self.theta_threshold_radians or theta > self.theta_threshold_radians
        done = theta <= -self.theta_threshold_radians or theta >= self.theta_threshold_radians
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

        # Reward a pole that is perpendicular to the ground.
        # yeta_error_mean = 0
        # self.yeta_error += abs(yeta)
        # self.yeta_error_cnt += 1
        # yeta_error_mean = self.yeta_error/float(self.yeta_error_cnt)
        # self.vprint('yeta_error_mean:', yeta_error_mean)
        # yeta_error_mean_bounded = yeta_error_mean/self.servo_max_angle # should be bounded to [0:1]
        # self.vprint('yeta_error_mean_bounded:', yeta_error_mean_bounded)
        # yeta_error_mean_inv = 1 - yeta_error_mean_bounded
        # self.vprint('yeta_error_mean_inv:', yeta_error_mean_inv)
        
        # Reward a pole that is perpendicular to the ground.
        self.pole2_error += abs(A.x - B.x)/self.length2
        self.pole2_error_cnt += 1.
        pole2_error_mean = self.pole2_error/self.pole2_error_cnt
        self.vprint('pole2_error_mean:', pole2_error_mean)
        pole2_error_mean_inv = 1 - pole2_error_mean
        self.vprint('pole2_error_mean_inv:', pole2_error_mean_inv)
        
        # Find the amount the pole is from being parallel to the ground.
        # We want to reward balancers that keep the upper pole parallel.
        # self.pole2_error += abs(A.y - B.y)/self.length2
        # self.pole2_error_cnt += 1.
        # pole2_error_mean = self.pole2_error/self.pole2_error_cnt
        # self.vprint('pole2_error_mean:', pole2_error_mean)
        # pole2_error_mean_inv = 1 - pole2_error_mean
        # self.vprint('pole2_error_mean_inv:', pole2_error_mean_inv)
        
        # Reward based on keeping the second pole pointing straight up.
        # self.yeta_error += abs(xp10 - hx)
        # self.yeta_error_cnt += 1
        # yeta_error_mean = 1 - self.yeta_error/self.yeta_error_cnt/self.length2 # should be bounded to [0:1]

        # weight = 1.0 # only look at raw step rewarad
        # weight = 0.33 # step count 1/3 as important as angle
        # weight = 0.5 # step count equally as important as angle
        # final_score = weight*reward + (1-weight)*off_center_score
        # final_score = reward + off_center_score*2
        
        # final_score = reward
        
        # Rewards stability and an upper-pole that's upright
        # final_score = reward + yeta_error_mean_inv
        
        # Rewards stability and an upper-pole that's parallel to the ground.
        ratio = 0.5
        final_score = (reward * ratio + pole2_error_mean_inv * (1 - ratio))*2
        
        # final_score = reward + yeta_error_mean_inv*10#TODO:remove
        # final_score = reward*5/10. + off_center_score*5/10.

        return np.array(self.state), final_score, done, {}

    def render(self, mode='human'):
        screen_width = 600
        screen_height = 400

        world_width = self.yeta_threshold * 2
        scale = screen_width/world_width
        cartx = screen_width/2.0 # MIDDLE OF THE SCREEN
        carty = 100 # TOP OF CART
        polewidth = 10.0
        polelen = scale * 1.0
        cartwidth = 50.0
        cartheight = 30.0
        
        weight_offset_x = self.cog_offset * scale

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)

            # Used to shift entire drawing right, to center it in the screen.
            self.trans = rendering.Transform()

            # Draw pole, going from ground to hip.
            axleoffset = cartheight/4.0
            l, r, t, b = -polewidth/2, polewidth/2, polelen, 0
            pole = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            pole.set_color(.8, .6, .4)
            self.poletrans = rendering.Transform(translation=(0, axleoffset)) # shift everything up to "ground" level
            pole.add_attr(self.poletrans)
            pole.add_attr(self.trans)
            self.viewer.add_geom(pole)

            # Draw pole axle.
            axle = rendering.make_circle(polewidth/2)
            axle.add_attr(self.poletrans)
            axle.add_attr(self.trans)
            axle.set_color(.5, .5, .8)
            self.viewer.add_geom(axle)

            # Draw second pole, going from hip to weight.
            l, r, t, b = -polewidth/2, polewidth/2, polelen/2., 0
            pole2 = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            pole2.set_color(.8, .6, .4)
            self.carttrans = rendering.Transform() # Used to shift everything relative to the second pole.
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

            # Draw track.
            self.track = rendering.Line((0, carty), (screen_width, carty))
            self.track.set_color(0, 0, 0)
            self.viewer.add_geom(self.track)

        if self.state is None:
            return None

        # x = self.state
        yeta, yeta_dot, theta, theta_dot = self.state
        self.carttrans.set_translation(0, polelen/2) # shift weight on top of second pole
        self.carttrans.set_rotation(yeta) # rotate second pole about hip angle
        self.carttrans.set_translation(0, polelen) # shift second pole on top of first pole
        self.trans.set_translation(cartx, carty) # center everything
        self.poletrans.set_rotation(theta) # rotate everything about foot angle

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.viewer:
            self.viewer.close()
