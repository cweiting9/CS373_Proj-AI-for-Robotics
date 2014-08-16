# ----------
# Part Three
#
# Now you'll actually track down and recover the runaway Traxbot. 
# In this step, your speed will be about twice as fast the runaway bot,
# which means that your bot's distance parameter will be about twice that
# of the runaway. You can move less than this parameter if you'd 
# like to slow down your bot near the end of the chase. 
#
# ----------
# YOUR JOB
#
# Complete the next_move function. This function will give you access to 
# the position and heading of your bot (the hunter); the most recent 
# measurement received from the runaway bot (the target), the max distance
# your bot can move in a given timestep, and another variable, called 
# OTHER, which you can use to keep track of information.
# 
# Your function will return the amount you want your bot to turn, the 
# distance you want your bot to move, and the OTHER variable, with any
# information you want to keep track of.
# 
# ----------
# GRADING
# 
# We will make repeated calls to your next_move function. After
# each call, we will move the hunter bot according to your instructions
# and compare its position to the target bot's true position
# As soon as the hunter is within 0.01 stepsizes of the target,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot. 
#
# As an added challenge, try to get to the target bot as quickly as 
# possible. 

from robot import *
from math import *
from matrix import *
import random

def find_center(p1,p2,p3):
    ##using Cramer's rule to solve the equation of the circle
    f1=-(p1[0]**2+p1[1]**2)
    f2=-(p2[0]**2+p2[1]**2)
    f3=-(p3[0]**2+p3[1]**2)
    A=[[p1[0],p1[1],1],[p2[0],p2[1],1],[p3[0],p3[1],1]]
    F=[f1,f2,f3]
    D=A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*A[2][1]-A[0][2]*A[1][1]*A[2][0]-A[0][1]*A[1][0]*A[2][2]-A[0][0]*A[1][2]*A[2][1]
    Dd=F[0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*F[2]+A[0][2]*F[1]*A[2][1]-A[0][2]*A[1][1]*F[2]-A[0][1]*F[1]*A[2][2]-F[0]*A[1][2]*A[2][1]
    De=A[0][0]*F[1]*A[2][2]+F[0]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*F[2]-A[0][2]*F[1]*A[2][0]-F[0]*A[1][0]*A[2][2]-A[0][0]*A[1][2]*F[2]
    Df=A[0][0]*A[1][1]*F[2]+A[0][1]*F[1]*A[2][0]+F[0]*A[1][0]*A[2][1]-F[0]*A[1][1]*A[2][0]-A[0][1]*A[1][0]*F[2]-A[0][0]*F[1]*A[2][1]
    d=Dd/D
    e=De/D
    f=Df/D
    r=sqrt((d/2)**2+(e/2)**2-f)
    x=-d/2.
    y=-e/2. 
    return [x ,y ,r]
    
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    if OTHER==None:
        OTHER=[]
        OTHER.append([measurement])
        OTHER.append([]) ##for (x,y,r)
        OTHER.append([]) ##for step_dist
        OTHER.append(False) ##for clockwise
        OTHER.append([]) ##for aver x ,y ,r ,step_size
        xy_estimate=measurement
        return xy_estimate, OTHER 
    elif len(OTHER[0])<30:
        ##collect enough measurements to calculate the center
        OTHER[0].append(measurement)
        xy_estimate=measurement
        return xy_estimate, OTHER 
    else:
        
        Z=measurement
        OTHER[0].append(Z)
        num=len(OTHER[0])
        Z_prev=OTHER[0][num-2]
        step_dist=distance_between(Z,Z_prev)
        OTHER[2].append(step_dist)
        ran1=random.randint(0, num-1)
        ran2=(ran1-9)%num
        ran3=(ran2-9)%num

        while True:
            Z1=OTHER[0][num-1]
            ran2=random.randint(0, num-2)
            ran3=random.randint(0, num-2)
            Z2=OTHER[0][ran2]
            Z3=OTHER[0][ran3]
            d12=distance_between(Z1,Z2)
            d23=distance_between(Z2,Z3)
            d13=distance_between(Z1,Z3)
            if d12 >3*step_dist and d23 >3*step_dist and d13 >3*step_dist:
                break
       
        cen=find_center(Z1,Z2,Z3)
        OTHER[1].append(cen)
       
        sum_x=0.0
        sum_y=0.0
        sum_r=0.0
        len_xy=len(OTHER[1])
        for i in range(len_xy):
            sum_x+=OTHER[1][i][0]
            sum_y+=OTHER[1][i][1]
            sum_r+=OTHER[1][i][2]
        sum_step_dist=0.0
        for i in range(len(OTHER[2])):
            sum_step_dist+=OTHER[2][i]
        aver_cen_x=sum_x/len_xy
        aver_cen_y=sum_y/len_xy
        aver_cen_r=sum_r/len_xy
        aver_step=sum_step_dist/len(OTHER[2])
        OTHER[4]=[ aver_cen_x, aver_cen_y, aver_cen_r,aver_step]
        d_theta=aver_step/aver_cen_r
        Z_prev2=OTHER[0][num-3]
        thetaZ_prev2=atan2(Z_prev2[1]-aver_cen_y,Z_prev2[0]-aver_cen_x)
        thetaZ_prev=atan2(Z_prev[1]-aver_cen_y,Z_prev[0]-aver_cen_x)
        thetaZ=atan2(Z[1]-aver_cen_y,Z[0]-aver_cen_x)
        if thetaZ<0:
            thetaZ+=2*pi
        if thetaZ_prev<0:
            thetaZ_prev+=2*pi
        if thetaZ_prev2<0:
            thetaZ_prev2+=2*pi
        Clockwise=False
        if len(OTHER[0])==32:
            if abs(thetaZ_prev-thetaZ_prev2)<pi: ## if the two points are not on the verge of 0 
                if thetaZ_prev>thetaZ_prev2:
                    Clockwise=False
                else:
                    Clockwise=True
            else:
                if thetaZ>thetaZ_prev:
                    Clockwise=False
                else:
                    Clockwise=True
            OTHER[3]=Clockwise

        if OTHER[3]==False: 
            THETA=thetaZ+d_theta
        else:
            THETA=thetaZ-d_theta
        next_x=aver_cen_x+aver_cen_r*cos(THETA)
        next_y=aver_cen_y+aver_cen_r*sin(THETA)
        xy_estimate=[next_x,next_y]
        
        return xy_estimate, OTHER 
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 
    if OTHER==None:
        OTHER=[]
        target_OTHER=None
        target_est, target_OTHER=estimate_next_pos(target_measurement, target_OTHER)
        OTHER.append(target_OTHER) ##OTHER[0]: target_OTHER
        diff_heading=get_heading(hunter_position, target_est)
        turning=angle_trunc(diff_heading-hunter_heading)
        dist=distance_between(target_est,hunter_position)
        if dist>max_distance:
            distance=max_distance
        else:
            distance=dist
    else:
        target_est, target_OTHER=estimate_next_pos(target_measurement, OTHER[0])
        OTHER[0]=target_OTHER   
        dist=distance_between(target_est,hunter_position)
        diff_heading=get_heading(hunter_position, target_est)
        turning=angle_trunc(diff_heading-hunter_heading)
        if dist>max_distance:
            distance=max_distance
        else:
            distance=dist
       
    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 1.94 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change Size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, next_move)




