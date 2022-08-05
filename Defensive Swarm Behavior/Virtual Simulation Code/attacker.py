from doctest import REPORTING_FLAGS
import p5
import numpy as np
from p5 import setup, draw, size, background, run


class Attacker():

    def __init__(self, x, y, width, height):
        self.position = p5.Vector(x, y)
        self.trgt_pos = p5.Vector(width/2,height/2)
        self.max_speed = 20
        self.max_force = 100
        self.width = width
        self.height = height
        self.perception = 355 # agents FOV (a circle with radius 355)
        self.radius = 25 # radius of the agent
        vec = (np.random.rand(2)-0.5)*10
        self.velocity = p5.Vector(*vec) # unpacks vec into the vector function: forms a random velocity vector
        vec = (np.random.rand(2)-0.5)/2
        self.acceleration = p5.Vector(15,15) # creates a random acceleration vector


    def update(self):
        self.position += self.velocity # updates position
        self.velocity += self.acceleration # updates velocity

        #limit
        if np.linalg.norm(self.velocity) > self.max_speed:
            self.velocity = self.velocity / np.linalg.norm(self.velocity) * self.max_speed

        self.acceleration = p5.Vector(*np.zeros(2))

    def show(self):

        p5.stroke(255,0,0)
        p5.fill(255,0,0)
        p5.ellipse(self.position.x, self.position.y, self.radius,self.radius)


    def edges(self): # edge case, for wrapping
        if self.position.x > self.width:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = self.width

        if self.position.y > self.height:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = self.height


    def attraction(self,atckrs): # the attacker's attraction to the target
        steering = p5.Vector(*np.zeros(2))
        im_cond = 0
        for atckr in atckrs:
            vec2target = self.trgt_pos - self.position # the vector pointing from each boid to the target
            dist2target = p5.mag(*vec2target)
            uvec = vec2target / dist2target # normalized unit vector in the direction of the target
            steering = uvec * self.max_force 
                # steering = vec2target*(bound_ratio*.5*self.max_force) # we create a vector pushing the agent towards the target
               # steering = -vec2target*(bound_ratio*.5*self.max_force) # we create a vector pushing the agent away from the target
            #if dist2target < 2*self.radius:
               # im_cond = 1

        return [steering,im_cond]


    def collision(self,boids):
        steering = p5.Vector(*np.zeros(2))
        collisions = []
        total = 0
        for boid in boids:
            vec2boid = boid.position - self.position # the direction of us to another boid is that boid's position minus our position
            #print(vec2boid)
            distance = p5.mag(*vec2boid) # distance from each other 
            if distance < 2*self.radius: # only if a swarm agent has collided with an attacker will the attacker be slowed
                direction = vec2boid/distance # unit vector in direction of the boid
                repl = -direction*500*self.max_force # only upon 
                collisions.append(repl)
                total = total + 1

        if total > 0:
            x = 0
            y = 0
            for val in range(len(collisions)):
                x = x + collisions[val][0] # summing the x components of the repulsion vectors
                y = y + collisions[val][1] # summing the y components of the repulsion vectors

            avg_x = x / total
            avg_y = y / total

            steering = p5.Vector(avg_x,avg_y)

        return steering



    def apply_behaviour(self,attackers,boids):
        #alignment = self.align(boids)
        #cohesion = self.cohesion(boids)
        im_cond = self.attraction(attackers)[1]
        if im_cond == 1: # stop because the attacker reached the target
            self.acceleration = 0
            self.velocity = 0

        else:
            collision = self.collision(boids)
            attraction = self.attraction(attackers)[0]
            self.acceleration = collision + attraction
            
