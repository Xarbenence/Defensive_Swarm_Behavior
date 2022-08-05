from p5 import *
from p5 import setup, draw, size, background, run
import numpy as np
from boid import Boid
from attacker import Attacker

width = 1000
height = 1000
j = 0

#flock = [Boid(*np.random.rand(2)*1000, width, height) for _ in range(30)]

swarm = [Boid(10*i,250,width,height) for i in range(5,20)] # number of swarm agents
attack = [Attacker(10,10,width,height),Attacker(900,900,width,height)] # number of attackers

def setup():
    #this happens just once
    size(width, height) #instead of create_canvas



def draw():
    #this happens every time
    global j
    j = j + 1
    background(30, 30, 47)
    
    for boid in swarm:
        boid.show()
        boid.apply_behaviour(swarm,attack)
        boid.update()
        boid.edges()
    
    if j > 30:
        for attacker in attack:
            attacker.show()
            attacker.apply_behaviour(attack,swarm)
            attacker.update()
            attacker.edges()
    

run()