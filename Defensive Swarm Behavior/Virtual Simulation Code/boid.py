from doctest import REPORTING_FLAGS
import p5
import numpy as np
from p5 import setup, draw, size, background, run

class Boid():

    def __init__(self, x, y, width, height):
        self.position = p5.Vector(x, y)
        self.trgt_pos = p5.Vector(width/2,height/2)
        self.max_speed = 20
        self.max_force = 1
        self.width = width
        self.height = height
        self.perception = 355 # FOV for fellow agents
        self.atckr_perception = 200 # FOV for attackers
        self.radius = 25
        vec = (np.random.rand(2)-0.5)*10
        self.velocity = p5.Vector(*vec) # unpacks vec into the vector function: forms a random velocity vector
    
        vec = (np.random.rand(2)-0.5)/2
        self.acceleration = p5.Vector(*vec) # creates a random acceleration vector


    def update(self):
        self.position += self.velocity # updates position
        self.velocity += self.acceleration # updates velocity

        #limit
        if np.linalg.norm(self.velocity) > self.max_speed:
            self.velocity = self.velocity / np.linalg.norm(self.velocity) * self.max_speed

        self.acceleration = p5.Vector(*np.zeros(2))

    def show(self):
        

        p5.stroke(255)
        p5.fill(255)
        p5.ellipse(self.position.x, self.position.y, self.radius,self.radius)

        p5.stroke(255,255,0)
        p5.fill(255,255,0)
        p5.ellipse(self.trgt_pos.x,self.trgt_pos.y,self.radius,self.radius)

    def edges(self):
        if self.position.x > self.width:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = self.width

        if self.position.y > self.height:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = self.height

    # def align(self, boids):
    #     steering = p5.Vector(*np.zeros(2))
    #     total = 0
    #     avg_vec = p5.Vector(*np.zeros(2))
    #     for boid in boids:
    #         if np.linalg.norm(boid.position - self.position) < self.perception:
    #             avg_vec += boid.velocity
    #             total += 1
    #     if total > 0:
    #         avg_vec /= total
    #         avg_vec = p5.Vector(*avg_vec)
    #         avg_vec = (avg_vec /np.linalg.norm(avg_vec)) * self.max_speed
    #         steering = avg_vec - self.velocity

    #     return steering

    def attraction(self,boids):
        steering = p5.Vector(*np.zeros(2))
        total = 0 
        for boid in boids:
            vec2target = self.trgt_pos - self.position # the vector pointing from each boid to the target
            dist2target = p5.mag(*vec2target)
            direction = vec2target/dist2target

            if dist2target > 250: # the farther we are from the target distance, the larger the force is on the agent

                bound_ratio = (dist2target - 250) / 500 # assuming 500 is our maximum FOV for the target
                steering = vec2target*(bound_ratio*self.max_force) # we create a vector pushing the agent towards the target

            elif dist2target < 250:

                bound_ratio = 1 - (dist2target/250)
                steering = -vec2target*(bound_ratio*self.max_force) # we create a vector pushing the agent away from the target

        return steering


    def separation(self, boids):
        steering = p5.Vector(*np.zeros(2))
        total = 0
        #avg_vector = p5.Vector(*np.zeros(2))

        repls = []

        for boid in boids:
            # distance = np.linalg.norm(boid.position - self.position)
            # a list of repulsion vectors for every boid
            
            vec2boid = boid.position - self.position # the direction of us to another boid is that boid's position minus our position
            #print(vec2boid)
            distance = p5.mag(*vec2boid) # distance from each other 
            if self.position != boid.position and distance < self.perception and distance > 2*self.radius: # if the boid is not our current boid and within our FOV
                direction = vec2boid/distance # normalized unit vector
                # direction = np.linalg.norm(*vec2boid) # a normalized vector in the direction of said boid
                repl = 1 - distance/self.perception # the inverse ratio
                repls.append(repl*-direction*10*self.max_force) # a repulsion vector in the opposite direction of the neighboring boid.
                                                             # its magnitude is based on how close the boid is to us
                total = total + 1
                #diff = self.position - boid.position
                #diff /= distance
                #avg_vector += diff
                #total += 1
            if self.position != boid.position and distance < 2*self.radius:
                direction = vec2boid/distance
                repl = -direction*100*self.max_force
                repls.append(repl)
                total = total + 1
            
        if total > 0:
            x = 0
            y = 0
            for val in range(len(repls)):
                x = x + repls[val][0] # summing the x components of the repulsion vectors
                y = y + repls[val][1] # summing the y components of the repulsion vectors
            avg_x = x / total
            avg_y = y / total

            steering = p5.Vector(avg_x,avg_y)
            #avg_repl_vec = p5.Vector(avg_x,avg_y)
            #steering = avg_repl_vec - self.velocity
            # avg_vector /= total
            # avg_vector = p5.Vector(*avg_vector)
            # if np.linalg.norm(steering) > 0:
            #     avg_vector = (avg_vector / np.linalg.norm(steering)) * self.max_speed
            # steering = avg_vector - self.velocity
            # if np.linalg.norm(steering)> self.max_force:
            #     steering = (steering /np.linalg.norm(steering)) * self.max_force

        return steering

    def defend(self,attackers):
        steering = p5.Vector(*np.zeros(2))
        total = 0
        mag_dir = {} # a dictionary containing the distances from a boid to all attackers in its FOV as keys, 
                     # and the direction said attackers are in as values
        mags = [] # a list containing just the distances of the attackers from the boid
        for attacker in attackers:
            vec2attacker = attacker.position - self.position
            distance = p5.mag(*vec2attacker) # distance from boid to attacker  
            if distance < self.atckr_perception: # if the attacker is within the boid's FOV
                direction = vec2attacker/distance # unit vector in direction of attacker
                mag_dir[distance] = direction # append the distance of the attacker as a key, and the direction as a value
                mags.append(distance) 

        if len(mags) > 0:
            EOI = min(mags)# closest enemy
            EOI_dir = mag_dir[EOI] # said enemy's direction
            steering = direction*500*self.max_force
            condition = 1
            return [condition,steering]
        
        else:
            condition = 0
            return [condition]


    def apply_behaviour(self,boids,attackers):
        #alignment = self.align(boids)
        #cohesion = self.cohesion(boids)
        defense = self.defend(attackers)
        if defense[0] == 0: # condition of 0 means no attacker in the vicinity
            separation = self.separation(boids)
            attraction = self.attraction(boids)
            self.acceleration = separation + attraction

        else: # nonzero condition means an attacker is present
            self.acceleration = defense[1]
        
