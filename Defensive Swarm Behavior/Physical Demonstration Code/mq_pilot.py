from microbit import *
import time
import machine
from lib_maqueen_plus import Robot
from lib_mb_huskylens import Obj_Class,Obj_Track,blocks,processReturnData,count,knock
import math
import neopixel

#FOV = 2*math.sqrt(2) # our agent's Field of View in feet
#FOVRSSI = # the signal strength that corresponds to the edge of our field of view
#TRGTRSSI =  # the signal strength that corresponds to our ideal distance from the target

T150 = 1.3 # amount of time it takes the agent to move one foot at 58.8 % PWM
           # from experimentation, found that this is approximately 1.3 seconds (150 out of 255)


def now():
    now = time.ticks_ms()
    return now

## PID GAINS & VALUES ##

Kp = 0.35 # proportional

Ki = 0.05 # integral

Kd = 0 # derivative

Target_X = 160 # We want to center the attacker's x-coord, on an axis from 0 to 320, so that we drive straight at it

Base_RPM = 55 # if error is 0, drive towards the target at 200 PWM


## INITIALIZATION OF HARDWARE ##

mq = Robot()

pn = neopixel.NeoPixel(pin15,4)

pn[0] = (255,0,0) # blue
pn[1] = (0,0,255) # white
pn[2] = (255,255,255) # red
pn[3] = (255,0,0) # red
pn.show()


# FUNCTIONS FOR CALCULATING MOVEMENT VECTORS AND PROCESSING DATA

def attraction(distance): # calculates the attractive force to the target
    if distance < 30: # if the ultrasonic sensor detects that the agent is less than 30 cm (1 foot) away from the target
        invr = 1 - distance/30 # inverse ratio
        time = invr*T150 # the closer we are, the longer we will spend driving away
        ticks = round(time*1000) # get the time in ticks
        dl = now() + ticks
        #print("Atr dl:" + str(dl))
        # Drive Backward
        mq.run(mq.MG,1,150) # left motor (MG)
        mq.run(mq.MD,1,150) # right motor (MD)
        while (dl - now()) > 0:
            #print("Now:" + str(now()))
            #print("Diff:" + str(dl-now()))
            continue
        mq.stop()

    else: # agent is greater than 30 cm (1 foot) away
        x = 60 - distance
        invr = 1 - x/30
        time = invr*T150 # the farther we are, the longer we will spend driving towards the perimeter
        ticks = round(time*1000) # get the time in ticks
        dl = now() + ticks
        #print("Atr dl:" + str(dl))
        # Drive Forward
        mq.run(mq.MG,2,150) # left motor (MG)
        mq.run(mq.MD,2,150) # right motor (MD)
        while (dl - now()) > 0:
            #print("Now:" + str(now()))
            #print("Diff:" + str(dl-now()))
            continue
        mq.stop()

def separation(distance):
    invr = 1 - distance/(122*math.sqrt(2))
    time = invr * 0.5 # if two agents are right next to each other, they should have a maximum repulsive force of moving backwards at 255 PWM for .5 seconds
    ticks = round(time*1000)
    dl = now() + ticks # deadline
    #print("Sep dl:" + str(dl))
    mq.run(mq.MG,1,255) # drive backward
    mq.run(mq.MD,1,255)
    while (dl - now()) > 0:
        #print("Now:" + str(now()))
        #print("Diff:" + str(dl-now()))
        continue
    mq.run(mq.MG,1,25) # left motor (MG), 1 means backward rotation
    mq.run(mq.MD,2,25) # right motor (MD), 2 means forward rotation
    time.sleep_ms(200) # permit the target to get out of view
    mq.stop()

connected = False

while connected == False: # initialize with the Husky Lens (HL)
    guests = i2c.scan()
    if len(guests) > 1:
        knock()
        connected = True


i = 0
Obj_Class() # intialize object classification mode
Mode = "Classification"

running_time = now()

while True:
    if now() - running_time >= 60000: # reset every 60 seconds
        machine.reset()

    if Mode != "Classification":
        Obj_Class()
        Mode = "Classification"

    try:
        if i % 2 == 0:
            display.set_pixel(0,0,9) # shows the while loop is running
        else:
            display.set_pixel(0,0,0)

        reading = blocks()[0]
        ID = reading.ID

        if ID == 1 and mq.ultrasonic() < 92: # Target Detected
            mq.stop() # Stop Motor
            print("Target Detected")
            t_dist = mq.ultrasonic() # read ultrasonic sensor to determine distance to target
            if t_dist < 60: # if the target is within the FOV
                attraction(t_dist) # move towards/away
                mq.run(mq.MG,1,25) # left motor (MG), 1 means backward rotation
                mq.run(mq.MD,2,25) # right motor (MD), 2 means forward rotation
                time.sleep_ms(200) # permit the target to get out of view
                mq.stop()
            else:
                mq.run(mq.MG,2,255) # left motor (MG), 1 means backward rotation
                mq.run(mq.MD,2,255) # right motor (MD), 2 means forward rotation
                time.sleep(0.5)
                mq.stop()

        reading = blocks()[0]
        ID = reading.ID

        if ID == 2 and mq.ultrasonic() < (30*math.sqrt(2)): # Agent Detected
            mq.stop()
            print("Agent Detected")
            a_dist = mq.ultrasonic()
            separation(a_dist) # move away

        reading = blocks()[0]
        ID = reading.ID

        if ID == 3 and mq.ultrasonic() < (30*math.sqrt(2)): # Attacker Detected
            mq.stop()
            print("Attacker Detected")

            Error = 0 # intialize error for PID
            Prev_Error = 0 # initialize previous error as zero
            Total_Error = 0 # initialize total error

            failures = 0

            Obj_Track() # Change to Object Tracking mode
            Mode = "Tracking"
            time.sleep(2) # give the HL time to reaquire the target in tracking mode
            while True:
                try:
                    reading = blocks()[0]
                    x = reading.x
                    print("x:",x)
                    Error = x - Target_X
                    Total_Error = Total_Error + Error

                    if Total_Error > 100:
                        Total_Error = 100

                    elif Total_Error < -100:
                        Total_Error = -100

                    Derivative = Error - Prev_Error

                    P = Kp * Error
                    I = Ki * Total_Error
                    D = Kd * Derivative

                    PWM = P + I + D # our PID algorithm
                    print(PWM)

                    LM = round(Base_RPM + PWM)

                    if LM > 255:
                        LM = 255

                    elif LM < -255:
                        LM = -255

                    print("LM:",LM)

                    RM = round(Base_RPM - PWM)

                    if RM > 255:
                        RM = 255

                    elif RM < -255:
                        RM = -255

                    print("RM:",RM)
                    if LM > 0:
                        mq.run(mq.MG,2,LM) # left motor (MG), 2 means forward rotation
                    else:
                        mq.run(mq.MG,1,abs(LM)) # left motor (MG), 1 means backward rotation

                    if RM > 0:
                        mq.run(mq.MD,2,RM) # right motor (MD), 2 means forward rotation
                    else:
                        mq.run(mq.MD,1,abs(RM)) # right motor (MD), 1 means backward rotation

                    Prev_Error = Error
                    failures = 0

                except:
                    mq.stop()
                    failures = failures + 1
                    print("Failures:",failures)
                    if failures >= 150:
                        break

            print("Exiting Tracking Mode")
            Obj_Class()
            Mode = "Classification"

        else:
            mq.run(mq.MG,1,25) # left motor (MG), 1 means backward rotation
            mq.run(mq.MD,2,25) # right motor (MD), 2 means forward rotation
            time.sleep_ms(200)
            mq.stop()

        i = i + 1

    except:
        i = i + 1
        ## If communnication with the HL fails, rotate till communication is re-initialized
        display.scroll("Re-Inint",delay=100,wait=False)
        mq.stop()



