"""
A Hack Collab by Josh Jacobs and Ian McNanie
Dank Inverse Pendulum Simulator (D.I.P.S.)
"""

import math
import pygame
import random
from pygame.locals import *
from pygame.color import *
import pymunk
from pymunk import Vec2d
import pymunk.pygame_util

class PyramidDemo:        
    def __init__(self):
        self.running = True
        self.drawing = True
        self.width = 1200
        self.height = self.width
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()

        ### Init pymunk and create space
        self.space = pymunk.Space()
        self.space.gravity = (0.0, -900.0)
        self.space.sleep_time_threshold = 0.3

        self.airdots = []

        self.weather_density = 1 #550*3
        self.hardcoreness = 3.0*1.5
        
        self.make_some_air()

        
        ### draw options for drawing
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)


        #pygame.display.flip()
        
        self.num_streams = 0
        self.increment = 20.0
        self.gas = 0.1

        self.angle_rad = 0.0


        self.gimbalx = 0
        self.gimbaly = 0
        self.vectx = 0
        self.vecty = 0

        
        """ Automate DIPS Vars """
        self.error = 0.0
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.kP = 0.03
        self.kI = 1.0
        self.Integrator = 0.0
        self.kD = 0.0
        self.Derivator = 0.0
        self.lastDerivator = 0.0
        self.PID = 0.0
        self.pid_dips_setpoint = 0.0

        """ Automate Roll Vars """
        self.roll_error = 0.0
        self.roll_P = 0.0
        self.roll_I = 0.0
        self.roll_D = 0.0
        self.roll_kP = 0.03
        self.roll_kI = 1.0
        self.roll_Integrator = 0.0
        self.roll_kD = 0.0
        self.roll_Derivator = 0.0
        self.roll_lastDerivator = 0.0
        self.roll_PID = 0.0
        self.roll_pid_dips_setpoint = 0.0

        
        """ Automate Throttle Vars """
        self.throttle_error = 0.0
        self.throttle_P = 0.0
        self.throttle_I = 0.0
        self.throttle_D = 0.0
        self.throttle_kP = 0.03
        self.throttle_kI = 1.0
        self.throttle_Integrator = 0.0
        self.throttle_kD = 0.0
        self.throttle_Derivator = 0.0
        self.throttle_lastDerivator = 0.0
        self.throttle_PID = 0.0
        self.throttle_pid_dips_setpoint = 0.0

        self.slide_a = 0
        
    def dip_rocket(self):
        x = self.width/2
        y = self.height/2

        p = Vec2d(x, y)
        
        points = [(-5,-30), (-5,30), (5,30), (5,-30)]
        mass = 1.0
        moment = pymunk.moment_for_poly(mass, points, (0, 0))
        self.rocket_body = pymunk.Body(mass, moment)
        self.rocket_body.position = p
        shape = pymunk.Poly(self.rocket_body, points)
        #shape.color = pygame.color.THECOLORS['blue']
        shape.friction = 0
        self.space.add(self.rocket_body, shape)
        
    def lead_filled_snow_shoe(self):
        x = self.width/5.0
        y = self.height/2.0

        p = Vec2d(x, y)

        lfss_size = 30
        lfss_mass = 2.0

        moment = pymunk.moment_for_circle(lfss_mass, 0, lfss_size)
        self.lfss_body = pymunk.Body(lfss_mass, moment)
        self.lfss_body.position = p
        self.lfss_body.moment = pymunk.inf

        self.shape = pymunk.Circle(self.lfss_body, lfss_size)
        self.shape.color = pygame.color.THECOLORS['pink']
        self.shape.friction = 1
                
        self.space.add(self.lfss_body, self.shape)
        
    def make_some_air(self):
        num_parts = self.weather_density
        
        self.num_streams = random.randint(4,10)
        self.stream_heights = []
        
        for x in range(self.num_streams):
            if x != self.num_streams:
                self.stream_heights.append((random.random()+0.5)*self.height/self.num_streams)
            else:
                self.stream_heights.append(self.height-sum(self.stream_heights))
                
        max_stream_speed = 400
        stream_intensities = []
        
        for y in range(self.num_streams):
            stream_intensities.append((random.random()-0.5)*self.hardcoreness*max_stream_speed)

        #print stream_intensities
            
        self.stream_cum = []

        self.stream_cum.append(self.stream_heights[0])
        for y in range(self.num_streams-1):
            self.stream_cum.append(self.stream_cum[y]+self.stream_heights[y+1])
            
        for j in range(num_parts):
            size = 2 + random.random() * 4
            mass = 1 + random.random() 
            
            y = self.height * random.random()
            x = self.width * random.random()
            p = Vec2d(x, y)
            
            moment = pymunk.moment_for_circle(mass, 0, 2)
            newairdot = pymunk.Body(mass, moment)
            newairdot.position = p

            c = int(y/self.height*250)
            
            shape = pymunk.Circle(newairdot, size)
            shape.color = (10,70+c/2,c)
            shape.friction = 1
            
            self.space.add(newairdot, shape)

            for check in range(self.num_streams):
                if y < self.stream_cum[check]:
                    newairdot.apply_impulse_at_local_point(pymunk.Vec2d(stream_intensities[check], 0), (0, 0))
                    break

            self.airdots.append(newairdot)

    def pid_dips_update(self, angle):
        error = self.pid_dips_setpoint-angle
        kP = 0.14
        kI = 0.0
        kD = 0.0

        self.Integrator = self.Integrator + error
        self.Derivator = self.Derivator - self.lastDerivator
        self.lastDerivator = self.Derivator
        
        PID = kP*error + kI*self.Integrator  + kD*self.Derivator

        #self.angle_rad = -PID

        
        roll_error = pygame.mouse.get_pos()[0] -  self.rocket_body.position.x
        kP = 0.0001
        kI = 0.0
        kD = 0.0

        self.roll_Integrator = self.roll_Integrator + roll_error
        self.roll_Derivator = self.roll_Derivator - self.roll_lastDerivator
        self.roll_lastDerivator = self.roll_Derivator
        
        roll_PID = kP*roll_error + kI*self.roll_Integrator  + kD*self.roll_Derivator
        
        
        throttle_error = pymunk.pygame_util.to_pygame(self.rocket_body.position,self.screen)[1] - pygame.mouse.get_pos()[1]+100
        kP = 0.0025
        kI = 0.0
        kD = 0.0

        self.throttle_Integrator = self.throttle_Integrator + throttle_error
        self.throttle_Derivator = self.throttle_Derivator - self.throttle_lastDerivator
        self.throttle_lastDerivator = self.throttle_Derivator
        
        throttle_PID = kP*throttle_error + kI*self.throttle_Integrator  + kD*self.throttle_Derivator

        self.gas = throttle_PID
        
        self.angle_rad = -PID + roll_PID
        
        print pygame.mouse.get_pos()[1], " ", self.rocket_body.position.y*2
            
    def run(self):
        self.dip_rocket()
        self.lead_filled_snow_shoe()

        while self.running:
            if True:
                if self.lfss_body.position.x > self.width:
                    y = self.lfss_body.position.y
                    x = 0
                    p = Vec2d(x, y)
                    self.lfss_body.position = p
                    
                if self.lfss_body.position.x < 0.0:
                    y = self.lfss_body.position.y
                    x = self.width
                    p = Vec2d(x, y)
                    self.lfss_body.position = p

            if self.gas < 0:
                self.gas = 0
                    
            r = self.angle_rad #/16.0
            m = self.gas * 12.0 #* self.increment
            y = math.cos(r)*m
            x = math.sin(r)*m

            
            
            #self.gas = self.gas/2.0
            
            self.lfss_body.apply_impulse_at_local_point(pymunk.Vec2d(0, self.gas), (0, 0))

            self.rocket_body.apply_impulse_at_local_point(pymunk.Vec2d(x, y), (0, 30))
            #self.rocket_body.apply_impulse_at_local_point(pymunk.Vec2d(0, self.gas), (0, 0))

            self.gimbalx, self.gimbaly = pymunk.pygame_util.to_pygame(self.rocket_body.position,self.screen)

            #sohcahtoa
            #print self.rocket_body.angle
            r2 = r + self.rocket_body.angle
            y2 = math.cos(r2)*m
            x2 = math.sin(r2)*m
            self.vectx = self.gimbalx + x2*20
            self.vecty = self.gimbaly + y2*20
            
            #self.vectx, self.vecty = pymunk.pygame_util.to_pygame(self.rocket_body.local_to_world(pymunk.Vec2d(-x, -1*y)),self.screen) #self.gimbalx-x*70
            #self.vecty = self.rocket_body.local_to_world(pymunk.Vec2d(x, y)) #self.gimbaly+y*70
            

            #print self.gas, " ", self.angle_rad
            
            for a in range(self.weather_density):
                if True:
                    for stream in range(len(self.stream_heights)):
                        if abs(self.airdots[a].position.y - self.stream_cum[stream]+10) < 10:
                            x = self.airdots[a].velocity.x
                            y = -self.airdots[a].velocity.y
                            p = Vec2d(x,y)
                            self.airdots[a].velocity = p
                
                if True:
                    if self.airdots[a].position.x < 0.0:
                        y = self.airdots[a].position.y
                        x = self.width
                        p = Vec2d(x,y)
                        self.airdots[a].position = p
                    if self.airdots[a].position.x > self.width:
                        y = self.airdots[a].position.y
                        x = 0
                        p = Vec2d(x,y)
                        self.airdots[a].position = p
                    
                if self.airdots[a].position.y < 0.0 or \
                   self.airdots[a].position.y > self.height:# or \
                   #self.airdots[a].position.x < 0.0 or \
                   #self.airdots[a].position.x > self.width:

                    y = self.height * random.random()
                    shab = random.randint(0,1)
                    x = shab * self.height
                    p = Vec2d(x,y)

                    #vx, vy = 0.0, 0.0
                    #vp = Vec2d(vx,vy)
                    #print self.airdots[a].velocity

                    #print self.airdots[a].velocity

                    self.airdots[a].position = p
                    
                    if True:
                        for check in range(self.num_streams):
                            if y < self.stream_cum[check]:
                                self.airdots[a].apply_impulse_at_local_point(pymunk.Vec2d(stream_intensities[check], 0), (0, 0))
                                break
            #self.life_goals()
            #self.automate_lfss_naive()
            self.pid_dips_update(self.rocket_body.angle)
            self.loop() 

    #def life_goals(self):
        
            
    def automate_lfss_naive(self):
        """ First a PID """
        self.error = pygame.mouse.get_pos()[0] -  self.lfss_body.position.x
        self.P = self.error * self.kP
        ## self.Integrator = self.Integrator + self.error
        ## self.I = self.kI * (self.error + self.Integrator)
        ## self.D = self.kD * (self.error - self.Derivator)
        ## self.Derivator = self.error
        ## self.PID = self.P + self.I + self.D
        ## self.gas = self.PID
        ## print self.PID

        """ Next, just an Integrator """
        self.Integrator = self.Integrator + self.error
        self.I = self.kI * (self.error + self.Integrator)
        #print self.I

        #if -self.I < 0:
        #    self.gas = self.P
        #else:
        #    self.gas = -self.P
        
        #self.gas = self.Integrator


    def loop(self):  
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                self.running = False
            elif event.type == KEYDOWN and event.key == K_p:
                pygame.image.save(self.screen, "box2d_pyramid.png")
            elif event.type == KEYDOWN and event.key == K_d:
                self.drawing = not self.drawing
            elif event.type == KEYDOWN and event.key == K_UP:
                self.gas = self.gas + 0.1

            elif event.type == KEYDOWN and event.key == K_DOWN:
                self.gas = self.gas - 0.1

            elif event.type == KEYDOWN and event.key == K_LEFT:
                self.pid_dips_setpoint = self.pid_dips_setpoint - 0.02 #self.angle_rad - 0.2
                #if self.angle_rad < -math.pi/8: self.angle_rad = -math.pi/8
                
            elif event.type == KEYDOWN and event.key == K_RIGHT:
                self.pid_dips_setpoint = self.pid_dips_setpoint + 0.02 #self.angle_rad + 0.2
                #if self.angle_rad > math.pi/8: self.angle_rad = math.pi/8


        if pygame.mouse.get_pressed()[0] != 0:
            # collision detection also needed here
            a = pygame.mouse.get_pos()[0] - 5
            print "pressed", a
        #pygame.display.update()

                
        fps = 30.
        dt = 1.0/fps/5        
        self.space.step(dt)
        if self.drawing:
            self.draw()
        
        ### Tick clock and update fps in title
        self.clock.tick(fps)
        pygame.display.set_caption("fps: " + str(self.clock.get_fps()))
        
    def draw(self):
        ### Clear the screen
        self.screen.fill(THECOLORS["white"])

        ### Draw space
        self.space.debug_draw(self.draw_options)

        pygame.draw.line(self.screen, (  0, 255,   0), [self.gimbalx, self.gimbaly], [self.vectx,self.vecty], 5)
        
        ### All done, lets flip the display
        pygame.display.flip()        

def main():
    demo = PyramidDemo()
    demo.run()

if __name__ == '__main__':
    main()
