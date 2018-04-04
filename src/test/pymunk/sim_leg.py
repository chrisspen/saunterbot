#!/usr/bin/env/python
"""
Simulates the balancing problem of a torso with a power hip joint attached to a leg with a ball foot.
"""
from __future__ import print_function
import os
import sys
from math import pi, isnan
# from math import sin, exp

import pygame
from pygame.locals import QUIT, KEYDOWN, KEYUP, K_s, K_r, K_q, K_ESCAPE, K_LEFT, K_RIGHT
# from pygame.locals import USEREVENT
# from pygame.locals import K_UP, K_DOWN
from pygame.color import THECOLORS

import pymunk
from pymunk import Vec2d
import pymunk.pygame_util

from reinforce import rl

LEG_GROUP = 1

POWERDOWN = 'powerdown'

CENTER = 'center'
OFFSET = 'offset'
CORNER = 'corner'
LEG_POSITIONS = (
    CENTER,
    OFFSET,
    CORNER,
)

def valmap(value, istart, istop, ostart, ostop):
    value = float(value)
    istart = float(istart)
    istop = float(istop)
    ostart = float(ostart)
    ostop = float(ostop)
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


class ServoController(object):

    def __init__(self, motor_joint, **kwargs):
        self.min_position = kwargs.pop('min_position', 1000)
        self.max_position = kwargs.pop('max_position', 2000)
        self.min_degrees = kwargs.pop('min_degrees', 0)
        self.max_degrees = kwargs.pop('max_degrees', 270)
        self.max_rate = kwargs.pop('max_rate', 5.0)
        self.target_position = None
        self.attached = False
        self.motor_joint = motor_joint
        self.auto_clamp = True

    def stop(self):
        self.motor_joint.rate = 0

    def attach(self):
        self.attached = True

    def detach(self):
        self.attached = False
        self.stop()

    def degree_to_position(self, deg):
        return valmap(deg, self.min_degrees, self.max_degrees, self.min_position, self.max_position)

    def set_position(self, pos):
        if self.auto_clamp:
            pos = max(min(pos, self.max_position), self.min_position)
        else:
            assert self.min_position <= pos <= self.max_position, 'Invalid position: %s' % pos
        self.target_position = pos

    def set_degrees(self, deg):
        # self.set_position(valmap(deg, self.min_degrees, self.max_degrees, self.min_position, self.max_position))
        self.set_position(self.degree_to_position(deg))

    def update(self, current_degrees):
        if not self.attached:
            self.stop()
            return
        # current_position = valmap(current_degrees, self.min_degrees, self.max_degrees, self.min_position, self.max_position)
        current_position = self.degree_to_position(current_degrees)
        error = current_position - self.target_position
        max_pos_diff = abs(self.min_position - self.max_position)
        rate = valmap(error, -max_pos_diff, +max_pos_diff, -self.max_rate, self.max_rate)
        self.motor_joint.rate = rate


class Simulator(rl.Domain):

    def __init__(self, headless=False, balancer=None, balancer_fn=None, verbose=False, **kwargs):
        super(Simulator, self).__init__()
        self.headless = headless
        self.display_flags = 0
        self.display_size = (600, 600)

        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.space.damping = 0.99 # to prevent it from blowing up.

        # Pymunk physics coordinates start from the lower right-hand corner of the screen.
        self.ground_y = 100
        ground = pymunk.Segment(self.space.static_body, (5, self.ground_y), (595, self.ground_y), 1.0)
        ground.friction = 1.0
        ground.elasticity = 0.0
        self.space.add(ground)

        self.screen = None

        self.draw_options = None

        self.cnt = 0

        self.verbose = verbose

        self.leg_position = kwargs.get('leg_position') or OFFSET
        assert self.leg_position in LEG_POSITIONS, 'Unknown leg position: %s' % self.leg_position

        shifter = kwargs.get('shifter')
        shifter_fn = kwargs.get('shifter_fn')
        self.shifter = None
        if shifter:
            if shifter == 'SARSALFAShifter':
                shifter_fn = shifter_fn or SARSALFAShifter.filename
                if os.path.isfile(shifter_fn):
                    if self.verbose:
                        print('Loading existing model: %s' % balancer_fn)
                    self.shifter = SARSALFAShifter.load()
                else:
                    if self.verbose:
                        print('Initializing blank model.')
                    self.shifter = SARSALFAShifter()
                    # We have a large state, so we'll need to explore for a while.
                    # self.shifter.epsilon_decay_factor = 0.001
                    # self.shifter.epsilon = 0.999
                    # self.shifter._epsilon = self.shifter.epsilon
                    # self.shifter.gamma = 0.8
                    # self.shifter.lambda_discount = 0.8

                # self.shifter._epsilon = 0.9999999
                # self.shifter.epsilon_decay_factor = 0.05
                print('ep:', self.shifter.epsilon)
                print('_ep:', self.shifter._epsilon)
            else:
                raise NotImplementedError('Unknown shifter: %s' % shifter)

        self.balancer = None
        if balancer:
            if balancer == 'SARSALFABalancer':
                balancer_fn = balancer_fn or SARSALFABalancer.filename
                if os.path.isfile(balancer_fn):
                    if self.verbose:
                        print('Loading existing model: %s' % balancer_fn)
                    self.balancer = SARSALFABalancer.load()
                else:
                    if self.verbose:
                        print('Initializing blank model.')
                    self.balancer = SARSALFABalancer()
                    # We have a large state, so we'll need to explore for a while.
                    self.balancer.epsilon_decay_factor = 0.001
                    self.balancer.epsilon = 0.999
                    self.balancer._epsilon = self.balancer.epsilon
                    self.balancer.gamma = 0.8
                    self.balancer.lambda_discount = 0.8

                # self.balancer._epsilon = 0.9999999
                print('ep:', self.balancer.epsilon)
                print('_ep:', self.balancer._epsilon)
            else:
                raise NotImplementedError('Unknown balancer: %s' % balancer)

        if self.verbose:
            print('balancer:', self.balancer)
            print('shifter:', self.shifter)

    def reset(self):
        self.cnt = 0
        self.reset_bodies()
        if self.balancer:
            self.balancer.reset()
        if self.shifter:
            self.shifter.reset()

    def reset_bodies(self):
        for body in self.space.bodies:
            if not hasattr(body, 'start_position'):
                continue
            body.position = Vec2d(body.start_position)
            body.force = 0, 0
            body.torque = 0
            body.velocity = 0, 0
            body.angular_velocity = 0
            body.angle = body.start_angle

    def draw(self):
        ### Clear the screen
        self.screen.fill(THECOLORS["white"])

        ### Draw space
        self.space.debug_draw(self.draw_options)

        ### All done, lets flip the display
        pygame.display.flip()

    def get_actions(self, agent):
        """
        If supported, returns a list of legal actions for the agent
        in the current state.
        """
        # Since the action is to set the hip servo position, and the servo has 1000 possible positions, there are technically 1000 possible actions.
        # However, to simplify the state, we'll abstract this into fewer actions by making the action relative to where the servo currently is.
        # So if we're at position 1500, and we want to go to position 1496, instead of outputing "1496" we'd output -4.
        # return [-64, -32, -16, -8, -4, -2, -1, 0, +1, +2, +4, +8, +16, +32, +64, POWERDOWN]
        return [-1000, -100, -10, -1, 0, +1, +10, +100, +1000]

    def run(self, fn):
        """
        Runs the interactive learning process.
        """
        self.run_once()

    def run_once(self, keep=False, start=False, verbose=False, **kwargs):

        pygame.init()
        if not self.headless:
            self.screen = pygame.display.set_mode(self.display_size, self.display_flags)
            width, height = self.screen.get_size()
            self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)

        def to_pygame(p):
            """Small hack to convert pymunk to pygame coordinates"""
            return int(p.x), int(-p.y+height)
        def from_pygame(p):
            return to_pygame(p)

        clock = pygame.time.Clock()
        running = True
        font = pygame.font.Font(None, 16)

        # Create the torso box.
        box_width = 50
        box_height = 100
        # box_width = 75
        # box_height = 200
        # box_width = 200
        # box_height = 100
        leg_length = 100
        # leg_length = 125
        leg_thickness = 2

        leg_shape_filter = pymunk.ShapeFilter(group=LEG_GROUP)

        # Create torso.
        torso_mass = 500
        torso_points = [(-box_width/2, -box_height/2), (-box_width/2, box_height/2), (box_width/2, box_height/2), (box_width/2, -box_height/2)]
        moment = pymunk.moment_for_poly(torso_mass, torso_points)
        body1 = pymunk.Body(torso_mass, moment)
        body1.position = (self.display_size[0]/2, self.ground_y+box_height/2+leg_length)
        body1.start_position = Vec2d(body1.position)
        body1.start_angle = body1.angle
        body1.center_of_gravity = (0, 0) # centered
        # body1.center_of_gravity = (0, box_height/2) # top-heavy
        # body1.center_of_gravity = (box_width/2, 0) # right-heavy
        shape1 = pymunk.Poly(body1, torso_points)
        shape1.filter = leg_shape_filter
        shape1.friction = 0.8
        shape1.elasticity = 0.0
        self.space.add(body1, shape1)

        # Create leg extending from the right to the origin.
        leg_mass = 1
        leg_points = [
            (leg_thickness/2, -leg_length/2),
            (-leg_thickness/2, -leg_length/2),
            # (-leg_thickness/2, leg_length/2-leg_thickness),
            # (leg_thickness/2, leg_length/2-leg_thickness/2)
            (-leg_thickness/2, leg_length/2),
            (leg_thickness/2, leg_length/2)
        ]
        leg_moment = pymunk.moment_for_poly(leg_mass, leg_points)
        body2 = pymunk.Body(leg_mass, leg_moment)
        if self.leg_position == CORNER:
            # Position leg at far corner of torso.
            body2.position = (self.display_size[0]/2-box_width/2+leg_thickness/2, self.ground_y+leg_length/2)
        elif self.leg_position == OFFSET:
            # Position leg near center of torso, but still offset to create slight instability.
            body2.position = (self.display_size[0]/2-leg_thickness/2, self.ground_y+leg_length/2)
        elif self.leg_position == CENTER:
            body2.position = (self.display_size[0]/2, self.ground_y+leg_length/2)
        else:
            raise NotImplementedError
        body2.start_position = Vec2d(body2.position)
        body2.start_angle = body2.angle
        shape2 = pymunk.Poly(body2, leg_points)
        shape2.filter = leg_shape_filter
        shape2.friction = 0.8
        shape2.elasticity = 0.0
        self.space.add(body2, shape2)

        # Link bars together at end.
        # pj = pymunk.PivotJoint(body1, body2, (self.display_size[0]/2-box_width/2, self.ground_y+leg_length-leg_thickness))
        if self.leg_position == CORNER:
            pj = pymunk.PivotJoint(body1, body2, (self.display_size[0]/2-box_width/2+leg_thickness/2, self.ground_y+leg_length)) # far corner
        elif self.leg_position == OFFSET:
            pj = pymunk.PivotJoint(body1, body2, (self.display_size[0]/2-leg_thickness/2, self.ground_y+leg_length)) # near center
        elif self.leg_position == CENTER:
            pj = pymunk.PivotJoint(body1, body2, (self.display_size[0]/2, self.ground_y+leg_length)) # near center
        else:
            raise NotImplementedError
        # pj = pymunk.PivotJoint(body1, body2, Vec2d(body2.position))
        self.space.add(pj)

        # Attach the foot to the ground in a fixed position.
        # We raise it above by the thickness of the leg to simulate a ball-foot. Otherwise, the default box foot creates discontinuities.
        if self.leg_position == CORNER:
            pj = pymunk.PivotJoint(self.space.static_body, body2, (self.display_size[0]/2-box_width/2+leg_thickness/2, self.ground_y+leg_thickness)) # far
        elif self.leg_position == OFFSET:
            pj = pymunk.PivotJoint(self.space.static_body, body2, (self.display_size[0]/2-leg_thickness/2, self.ground_y+leg_thickness)) # off center
        elif self.leg_position == CENTER:
            pj = pymunk.PivotJoint(self.space.static_body, body2, (self.display_size[0]/2, self.ground_y+leg_thickness)) # center
        else:
            raise NotImplementedError
        self.space.add(pj)

        # Actuate the bars via a motor.
        motor_joint = None
        motor_joint = pymunk.SimpleMotor(body1, body2, 0)
        # motor_joint.max_force = 1e10 # mimicks default infinity, too strong, breaks rotary limit joints
        motor_joint.max_force = 1e9
        # motor_joint.max_force = 1e7 # too weak, almost no movement
        self.space.add(motor_joint)

        # Simulate friction on the foot joint.
        # https://chipmunk-physics.net/forum/viewtopic.php?f=1&t=2916
        foot_gj = pymunk.GearJoint(self.space.static_body, body2, 0.0, 1.0)
        foot_gj.max_force = 1e8
        self.space.add(foot_gj)

        # Simulate friction on the hip joint.
        # https://chipmunk-physics.net/forum/viewtopic.php?f=1&t=2916
        # hip_gj = pymunk.GearJoint(body1, body2, 0.0, 1.0)
        # hip_gj.max_force = 1e10
        # self.space.add(hip_gj)

        # Add hard stops to leg pivot so the leg can't rotate through the torso.
        angle_range = pi/4. # 45 deg
        # angle_range = pi/2. # 90 deg
        hip_limit_joint = pymunk.RotaryLimitJoint(body1, body2, -angle_range, angle_range) # -45deg:+45deg
        self.space.add(hip_limit_joint)

        # default angle = 270/2 = 135 deg
        servo = None
        if motor_joint:
            servo = ServoController(motor_joint, max_rate=20.0)
            servo.set_degrees(135)

        # pygame.time.set_timer(USEREVENT+1, 70000) # apply force
        # pygame.time.set_timer(USEREVENT+2, 120000) # reset
        # pygame.event.post(pygame.event.Event(USEREVENT+1))
        # pygame.mouse.set_visible(False)

        self.reset()

        last_body1_pos = None
        last_body1_vel = None
        simulate = False
        self.cnt = 0
        failed = None
        simulate = start
        current_state = None
        # The point at which the torso has toppled over.
        failure_degrees = 45
        while running:
            self.cnt += 1
            # print('angles:', body1.angle, body2.angle)
            # print('torso force:', body1.force)
            # print('body1.position: %.02f %.02f' % (body1.position.x, body1.position.y))

            current_body1_vel = None
            if last_body1_pos:
                current_body1_vel = body1.position - last_body1_pos
                # print('current_body1_vel: %.02f %.02f' % (current_body1_vel.x, current_body1_vel.y))

            current_body1_accel = None
            if last_body1_vel:
                current_body1_accel = current_body1_vel - last_body1_vel
                # print('current_body1_accel: %.02f %.02f' % (current_body1_accel.x, current_body1_accel.y))

            # Angle of the torso, where 0 degrees is the torso perfectly parallel to the ground.
            # A positive angle is clockwise rotation.
            # This simulates the IMU's y euler angle measurement.
            torso_angle = -body1.angle * 180/pi
            # print('torso_angle:', torso_angle)

            # Angle of the hip servo.
            # A positive angle is clockwise rotation.
            # This simulates the estimated hip servo potentiometer.
            hip_angle = (body2.angle - body1.angle) * 180/pi # 0 degrees means leg is angled straight down

            # Angle of the foot, where 0 degrees is the foot perfectly perpedicular to the ground.
            # A positive angle is clockwise rotation.
            foot_angle = -body2.angle * 180/pi
            # print('foot_angle:', foot_angle)

            # Re-frame angle so that 135 degrees indicates the straight down orientation.
            hip_angle += 135
            # print('hip_angle:', hip_angle)

            current_state = [torso_angle/360., hip_angle/360., foot_angle/360.]
            # assert not [_ for _ in current_state if abs(_) > 1]

            # Auto-shift COG using foot angle as proportion of shift
            # iters=163
            # If we lean right, shift COG left, and vice versa.
            # body1.center_of_gravity = (-box_width/2*foot_angle/90.*10, 0)
            # cog_limit = box_width/2
            # print('cog_limit:', cog_limit)
            # print('foot_angle:', foot_angle)
            # dist_from_center = sin(-body2.angle)*leg_length
            # print('dist_from_center:', dist_from_center)
            # max_dist_from_center = sin(pi/4)*leg_length
            # print('max_dist_from_center:', max_dist_from_center)
            # comp_rate = -dist_from_center/max_dist_from_center
            # print('comp_rate:', comp_rate)
            # gamma = 4
            # comp_rate2 = exp(abs(comp_rate*gamma))
            # if comp_rate < 0:
                # comp_rate2 = -comp_rate2
            # print('comp_rate2:', comp_rate2)
            # comp_rate3 = max(min(comp_rate2, cog_limit), -cog_limit)
            # print('comp_rate3:', comp_rate3)
            # body1.center_of_gravity = (comp_rate3, 0)

            if self.verbose:
                print('foot.angular_velocity:', body2.angular_velocity) # positive=CCW, negative=CW
                if isnan(float(body2.angular_velocity)):
                    print('Physics breakdown! Terminating simulation!', file=sys.stderr)
                    return

            # Auto-shift COG using foot angular velocity direction.
            # iters=403
            servo.set_position(1500)
            servo.attach()
            # if body2.angular_velocity > 0:
                # # falling left
                # body1.center_of_gravity = (box_width/2, 0) # shift weight right
            # else:
                # # falling right
                # body1.center_of_gravity = (-box_width/2, 0) # shift weight left

            # Auto-shift COG using foot angular velocity magnitude.
            # body1.center_of_gravity = (box_width/2*body2.angular_velocity/5*1, 0)# iters=158
            # body1.center_of_gravity = (box_width/2*body2.angular_velocity/5*2, 0)# iters=161
            # body1.center_of_gravity = (box_width/2*body2.angular_velocity/5*2.5, 0)# iters=157
            # body1.center_of_gravity = (box_width/2*body2.angular_velocity/5*3, 0)# iters=160
            # body1.center_of_gravity = (box_width/2*body2.angular_velocity/5*5, 0)# iters=153
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*1, box_width/2), -box_width/2), 0)# iters=153
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*2, box_width/2), -box_width/2), 0)# iters=135
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*3, box_width/2), -box_width/2), 0)# iters=132
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*100, box_width/2), -box_width/2), 0)# iters=290
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*50, box_width/2), -box_width/2), 0)# iters=216
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*25, box_width/2), -box_width/2), 0)# iters=278
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*200, box_width/2), -box_width/2), 0)# iters=293
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*1000, box_width/2), -box_width/2), 0)# iters=304
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*10000, box_width/2), -box_width/2), 0)# iters=333
            # body1.center_of_gravity = (max(min(box_width/2*body2.angular_velocity*100000, box_width/2), -box_width/2), 0)# iters=333

            if self.verbose:
                print('cog:', body1.center_of_gravity)

            # Auto-shift hip angle to level.
            #TODO

            # Handle balancer agent input.
            if self.balancer:
                relative_position_change = self.balancer.get_action(state=current_state, actions=self.get_actions(self.balancer))
                if self.verbose:
                    print('balancer action:', relative_position_change)
                # Otherwise convert the relative +/- position into an absolute servo position.
                hip_position = servo.degree_to_position(hip_angle)
                new_hip_position = hip_position + relative_position_change
                servo.set_position(new_hip_position)
                # servo.set_position(1000)#manual
                servo.attach()

            # Handle shifter agent input.
            if self.shifter:
                shifter_state = [body2.angular_velocity/10.]
                cog_shift = self.shifter.get_action(
                    state=list(shifter_state),
                    actions=[-box_width/2., -box_width/4., -box_width/16., -box_width/32., 0, +box_width/2., +box_width/4., +box_width/16., +box_width/32.])
                body1.center_of_gravity = (max(min(cog_shift, box_width/2), -box_width/2), 0)

            # Handle user input.
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    running = False
                elif event.type == KEYDOWN and event.key == K_s:
                    # Start/stop simulation.
                    simulate = not simulate
                elif event.type == KEYDOWN and event.key == K_r:
                    # Reset.
                    self.reset_bodies()
                elif event.type == KEYDOWN and event.key == K_LEFT:
                    # Angle backwards by rotating torso about the hips counter-clockwise.
                    servo.set_degrees(servo.min_degrees)
                    servo.attach()
                elif event.type == KEYDOWN and event.key == K_RIGHT:
                    # Angle forwarads by rotating torso about the hips clockwise.
                    servo.set_degrees(servo.max_degrees)
                    servo.attach()
                elif event.type == KEYUP:
                    # Otherwise, make the servo go unpowered and idle.
                    servo.detach()
                servo.update(current_degrees=hip_angle)

            if not self.headless:
                self.draw()

            last_body1_pos = Vec2d(body1.position)
            if current_body1_vel:
                last_body1_vel = Vec2d(current_body1_vel)

            ### Update physics
            fps = 50
            iterations = 25
            dt = 1.0/float(fps)/float(iterations)
            if simulate:
                for x in range(iterations): # 10 iterations to get a more stable simulation
                    self.space.step(dt)

            if not self.headless:
                pygame.display.flip()
            clock.tick(fps)

            # Give balancer feedback.
            # The balancer's objective is to keep the torso as level as possible, balanced on the foot.
            failed = abs(foot_angle) > failure_degrees
            if self.balancer:
                # Feedback is the inverse distance of the torso's angle from center.
                if failed:
                    # Terminal failure.
                    feedback = -1
                else:
                    # Otherwise, grade performance based on how level the torso is.
                    feedback = valmap(abs(torso_angle), 0, 90, 1, -1)

                # print('failed:', failed)
                self.balancer.reinforce(feedback=feedback+self.cnt, state=current_state, end=failed)

            if self.shifter:
                if failed:
                    feedback = -1
                else:
                    # feedback = 0
                    feedback = (90. - abs(foot_angle))/90.
                self.shifter.reinforce(feedback=feedback, state=shifter_state, end=failed)

            # Check failure criteria.
            if not keep and (failed or self.cnt >= 1000):
                break

        print('Result: %s' % {True: 'failure', False:'aborted', None:'aborted'}[failed])
        print('Iterations: %i' % self.cnt)

        if self.balancer:
            self.balancer.save()
        if self.shifter:
            self.shifter.save()


class SARSALFABalancer(rl.SARSALFAAgent):
    """
    Learns to balance using basic SARSA and linear function approximation.
    """

    filename = 'models/sarsalfa-balancer.yaml'

    def normalize_state(self, state):
        """
        Converts state into a list of numbers that can be used in
        a linear function approximation.
        """
        return state


class SARSALFAShifter(rl.SARSALFAAgent):
    """
    Learns to balance using basic SARSA and linear function approximation.
    """

    filename = 'models/sarsalfa-shifter.yaml'

    def normalize_state(self, state):
        """
        Converts state into a list of numbers that can be used in
        a linear function approximation.
        """
        return state


class ANNShifter(rl.ANNAgent):
    """
    Learns to play using an artificial neural network.
    
    Works by using the ANN to estimate the the expected reward after
    performing each legal action and recommends the action corresponding
    to the highest expected reward. 
    """
    
    filename = 'models/ann-shifter.yaml'

    def normalize_state(self, state):
        """
        Converts state into a list of numbers.
        """
        assert self.color is not None
        
        # Make the board relative to us.
        if not (US in state or THEM in state):
            state = self.relativize_state(state)
        
        # Convert to integers suitable for input into the ANN.
        state = [self.symbol_to_int[_] for _ in state]
        
        # Convert one of 8 possible symmetric versions to the standard.
        boards = sorted(xo_fast.transform_board(state))
        state = boards[0]
        
        return state

    def simulate_action(self, state, action):
        """
        Returns the expected next-state if the given action is performed
        in the given state.
        """
#        print 'state0:',state,action
        state = self.relativize_state(state)
        invalid = set(state).difference([US, THEM, EMPTY])
        assert not invalid, invalid
        assert state[action] == EMPTY
        state = list(state)
        state[action] = US
#        print 'state1:',state
        return state


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Runs balancing simulation.')
    parser.add_argument('--keep', default=False, action='store_true', help='Continue simulation even after failure criteria reached.')
    parser.add_argument('--headless', default=False, action='store_true', help='Hides the gui.')
    parser.add_argument('--start', default=False, action='store_true', help='Automatically starts the simulation.')
    parser.add_argument('--balancer', default=None, help='Class name of the balancer.')
    parser.add_argument('--shifter', default=None, help='Class name of the shifter.')
    parser.add_argument('--verbose', default=False, action='store_true', help='Show verbose output.')
    parser.add_argument('--sessions', default=1, type=int, help='When using a balancer, the number of sessions to run.')
    parser.add_argument('--leg-position', default=None, help='Relative position of the leg.')
    arguments = parser.parse_args()
    if arguments.balancer or arguments.shifter:
        for i in range(arguments.sessions):
            print('-'*80)
            print('Session %i of %i' % (i+1, arguments.sessions))
            print('-'*80)
            sim = Simulator(**arguments.__dict__)
            sim.run_once(**arguments.__dict__)
    else:
        sim = Simulator(**arguments.__dict__)
        sim.run_once(**arguments.__dict__)
