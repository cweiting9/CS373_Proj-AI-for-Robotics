# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def Circle_Fitting(List_PointSet):
    #using Least Squares method to obtain the equation of the cirlce 
    #initailing sum
    X1=0.0
    Y1=0.0
    X2=0.0
    Y2=0.0
    X3=0.0
    Y3=0.0
    X1_Y1=0.0
    X1_Y2=0.0
    X2_Y1=0.0
    N=len(List_PointSet)
    for i in range(N):
        X1+=List_PointSet[i][0]
        Y1+=List_PointSet[i][1]
        X2+=List_PointSet[i][0]*List_PointSet[i][0]
        Y2+=List_PointSet[i][1]*List_PointSet[i][1]
        X3+=List_PointSet[i][0]*List_PointSet[i][0]*List_PointSet[i][0]
        Y3+=List_PointSet[i][1]*List_PointSet[i][1]*List_PointSet[i][1]
        X1_Y1+=List_PointSet[i][0]*List_PointSet[i][1]
        X1_Y2+=List_PointSet[i][0]*List_PointSet[i][1]*List_PointSet[i][1]
        X2_Y1+=List_PointSet[i][0]*List_PointSet[i][0]*List_PointSet[i][1]
    C=N*X2 - X1*X1
    D=N*X1_Y1 - X1*Y1
    E=N*X3 + N*X1_Y2 - (X2+Y2)*X1
    G=N*Y2 - Y1*Y1
    H=N*X2_Y1 + N*Y3 - (X2+Y2)*Y1

    a=(H*D-E*G)/(C*G-D*D)
    b=(H*C-E*D)/(D*D-G*C)
    c=-(a*X1 + b*Y1 + X2 + Y2)/N

    Xc=a/(-2)
    Yc= b/(-2)
    R=sqrt(a*a+b*b-4*c)/2
    return [Xc ,Yc ,R]
    
def find_center(p1,p2,p3):
    #using Cramer's rule to solve the equation of the circle
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
        OTHER.append([]) #for (x,y,r)
        OTHER.append([]) #for step_dist
        OTHER.append(False) #for clockwise
        xy_estimate=measurement
        return xy_estimate, OTHER 
    elif len(OTHER[0])<5:
        #collect enough measurements to calculate the center
        OTHER[0].append(measurement)
        xy_estimate=measurement
        return xy_estimate, OTHER 
    else:
        
        Z=measurement
        OTHER[0].append(Z) ##update OTHER[0] measurement
        num=len(OTHER[0]) ## the number of the measurements
        Z_prev=OTHER[0][num-2] ## the previous measurement
        Z_prev2=OTHER[0][num-3] ## the one more previous than Z_prev
        step_dist=distance_between(Z,Z_prev)
        OTHER[2].append(step_dist) ##collect every step_dist for average
        
        Center=Circle_Fitting(OTHER[0])
        print Center
        aver_cen_x=Center[0]
        aver_cen_y=Center[1]
        aver_cen_r=Center[2]
        sum_step_dist=0.0
        for i in range(len(OTHER[2])):
            sum_step_dist+=OTHER[2][i]
        aver_step=sum_step_dist/len(OTHER[2])

        
        ##to make sure the bot moves clockwisely or not
        
        thetaZ_prev2=atan2(Z_prev2[1]-aver_cen_y,Z_prev2[0]-aver_cen_x)
        thetaZ_prev=atan2(Z_prev[1]-aver_cen_y,Z_prev[0]-aver_cen_x)
        thetaZ=atan2(Z[1]-aver_cen_y,Z[0]-aver_cen_x)
        if thetaZ<0:
            thetaZ+=2*pi ##make thetaZ between 0 and 2pi
        if thetaZ_prev<0:
            thetaZ_prev+=2*pi
        if thetaZ_prev2<0:
            thetaZ_prev2+=2*pi
        
        Clockwise=False##set default to False     
        if len(OTHER[0])==16:
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

        d_theta=aver_step/aver_cen_r ## average travel degree
        if OTHER[3]==False: 
            THETA=thetaZ+d_theta
        else:
            THETA=thetaZ-d_theta

        next_x=aver_cen_x+aver_cen_r*cos(THETA)
        next_y=aver_cen_y+aver_cen_r*sin(THETA)
        xy_estimate=[next_x,next_y]
        
        return xy_estimate, OTHER 
    
    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.

    

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized
# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.5, 4.3, 0.5, 2*pi /30, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)




