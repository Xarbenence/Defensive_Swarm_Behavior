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
        self.perception = 355
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
        p5.ellipse(self.position.x, self.position.y, 10,10)

        p5.stroke(255,0,0)
        p5.fill(255,0,0)
        p5.ellipse(self.trgt_pos.x,self.trgt_pos.y,10,10)

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
            if dist2target > 250: # the farther we are from the target distance, the larger the force is on the agent

                bound_ratio = (dist2target - 250) / 250 # assuming 500 is our maximum FOV for the target
                steering = vec2target*(bound_ratio*.5*self.max_force) # we create a vector pushing the agent towards the target

            elif dist2target < 250:

                bound_ratio = 1 - (dist2target/250)
                steering = -vec2target*(bound_ratio*.5*self.max_force) # we create a vector pushing the agent away from the target

        return steering

    # def cohesion(self, boids):
    #     steering = p5.Vector(*np.zeros(2))
    #     total = 0
    #     center_of_mass = p5.Vector(*np.zeros(2))
    #     for boid in boids:
    #         if np.linalg.norm(boid.position - self.position) < self.perception:
    #             center_of_mass += boid.position
    #             total += 1
    #     if total > 0:
    #         center_of_mass /= total
    #         center_of_mass = p5.Vector(*center_of_mass)
    #         vec_to_com = center_of_mass - self.position
    #         if np.linalg.norm(vec_to_com) > 0:
    #             vec_to_com = (vec_to_com / np.linalg.norm(vec_to_com)) * self.max_speed
    #         steering = vec_to_com - self.velocity
    #         if np.linalg.norm(steering)> self.max_force:
    #             steering = (steering /np.linalg.norm(steering)) * self.max_force

    #     return steering

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
            if self.position != boid.position and distance < self.perception: # if the boid is not our current boid and within our FOV
                direction = vec2boid/distance
                # direction = np.linalg.norm(*vec2boid) # a normalized vector in the direction of said boid
                repl = 1 - distance/self.perception # the inverse ratio
                repls.append(repl*-direction*2*self.max_force) # a repulsion vector in the opposite direction of the neighboring boid.
                                                             # its magnitude is based on how close the boid is to us
                total = total + 1
                #diff = self.position - boid.position
                #diff /= distance
                #avg_vector += diff
                #total += 1
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

    def apply_behaviour(self, boids):
        #alignment = self.align(boids)
        #cohesion = self.cohesion(boids)
        separation = self.separation(boids)
        attraction = self.attraction(boids)
        self.acceleration = separation + attraction
        
