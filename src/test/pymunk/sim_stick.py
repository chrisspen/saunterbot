import sys

import pygame
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_s, K_r, K_q, K_ESCAPE, K_UP, K_DOWN
from pygame.color import THECOLORS

import pymunk
from pymunk import Vec2d
import pymunk.pygame_util

class Simulator(object):

    def __init__(self):
        self.display_flags = 0
        self.display_size = (600, 600)

        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.space.damping = 0.999 # to prevent it from blowing up.

        # Pymunk physics coordinates start from the lower right-hand corner of the screen.
        self.ground_y = 100
        ground = pymunk.Segment(self.space.static_body, (5, self.ground_y), (595, self.ground_y), 1.0)
        ground.friction = 1.0
        self.space.add(ground)

        self.screen = None

        self.draw_options = None

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

    def main(self):

        pygame.init()
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
        leg_length = 100
        
        shape_filter = pymunk.ShapeFilter(group=1)

        mass = 1
        points = [(-100, -1), (0, -1), (0, 1), (-100, 1)]
        moment = pymunk.moment_for_poly(mass, points)
        body1 = pymunk.Body(mass, moment)
        # body1.position = (0, 0)
        body1.position = (self.display_size[0]/2, self.ground_y+100)
        body1.center_of_gravity = (-50, 0)
        body1.start_position = Vec2d(body1.position)
        body1.start_angle = body1.angle
        shape1 = pymunk.Poly(body1, points)
        shape1.filter = shape_filter
        shape1.friction = 0.8
        self.space.add(body1, shape1)

        # Create bar 2 extending from the right to the origin.
        mass = 1
        points = [(100, -1), (0, -1), (0, 1), (100, 1)]
        moment = pymunk.moment_for_poly(mass, points)
        body2 = pymunk.Body(mass, moment)
        # body2.position = (0, 0)
        body2.position = (self.display_size[0]/2, self.ground_y+100)
        body2.center_of_gravity = (50, 0)
        body2.start_position = Vec2d(body2.position)
        body2.start_angle = body2.angle
        shape2 = pymunk.Poly(body2, points)
        shape2.filter = shape_filter
        shape2.friction = 0.8
        self.space.add(body2, shape2)

        # Link bars together at end.
        pj = pymunk.PinJoint(body1, body2, (0, 0), (0, 0))
        self.space.add(pj)

        motor_joint = pymunk.SimpleMotor(body1, body2, 0)
        self.space.add(motor_joint)

        pygame.time.set_timer(USEREVENT+1, 70000) # apply force
        pygame.time.set_timer(USEREVENT+2, 120000) # reset
        pygame.event.post(pygame.event.Event(USEREVENT+1))
        pygame.mouse.set_visible(False)

        simulate = False
        while running:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    #running = False
                    sys.exit(0)
                elif event.type == KEYDOWN and event.key == K_s:
                    # Start/stop simulation.
                    simulate = not simulate
                elif event.type == KEYDOWN and event.key == K_r:
                    # Reset.
                    # simulate = False
                    self.reset_bodies()
                elif event.type == KEYDOWN and event.key == K_UP:
                    motor_joint.rate = 1
                elif event.type == KEYDOWN and event.key == K_DOWN:
                    motor_joint.rate = -1
                elif event.type == KEYUP:
                    motor_joint.rate = 0

            self.draw()

            ### Update physics
            fps = 50
            iterations = 25
            dt = 1.0/float(fps)/float(iterations)
            if simulate:
                for x in range(iterations): # 10 iterations to get a more stable simulation
                    self.space.step(dt)

            pygame.display.flip()
            clock.tick(fps)

if __name__ == '__main__':
    sim = Simulator()
    sim.main()
