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
def Circle_Fitting(ThreePoints,PointData=None):
    #using Least Squares method to obtain the equation of the cirlce
    #euation of circle: x^2+y^2+ax+by+c=0
    #Xc=-a/2 Yc=-b/2 R=0.5*sqrt(a^2+b^2-4c)
    #Measurement point set (Xi, Yi)
    #di^2=(Xi-Xc)^2+(Yi-Yc)^2
    #Ji=(di^2-R^2)^2; J=Summation of Ji=sum of (di^2-R^2)^2
    #in order to minimize J,  the partial derivatives of J with respect to a, b, c equal to zero
    #solve a, b, c to get Xc, Yc ,R
    if len(PointData)==0:
        X1=0.0
        Y1=0.0
        X2=0.0
        Y2=0.0
        X3=0.0
        Y3=0.0
        X1_Y1=0.0
        X1_Y2=0.0
        X2_Y1=0.0
        N=len(ThreePoints)
        #calculate the summation of X1 Y1 X2......X2_Y1
        #and store them in the PointData to avoiad repeative addtions
        for i in range(N):
            X1+=ThreePoints[i][0]
            Y1+=ThreePoints[i][1]
            X2+=ThreePoints[i][0]*ThreePoints[i][0]
            Y2+=ThreePoints[i][1]*ThreePoints[i][1]
            X3+=ThreePoints[i][0]*ThreePoints[i][0]*ThreePoints[i][0]
            Y3+=ThreePoints[i][1]*ThreePoints[i][1]*ThreePoints[i][1]
            X1_Y1+=ThreePoints[i][0]*ThreePoints[i][1]
            X1_Y2+=ThreePoints[i][0]*ThreePoints[i][1]*ThreePoints[i][1]
            X2_Y1+=ThreePoints[i][0]*ThreePoints[i][0]*ThreePoints[i][1]
        PointData=[X1,Y1,X2,Y2,X3,Y3,X1_Y1,X1_Y2,X2_Y1,N]
    else:
        #ThreePoints[2] is new measurement
        #retrive the summation from PointData and add neww measurement
        X1=PointData[0]+ThreePoints[2][0]
        Y1=PointData[1]+ThreePoints[2][1]
        X2=PointData[2]+ThreePoints[2][0]*ThreePoints[2][0]
        Y2=PointData[3]+ThreePoints[2][1]*ThreePoints[2][1]
        X3=PointData[4]+ThreePoints[2][0]*ThreePoints[2][0]*ThreePoints[2][0]
        Y3=PointData[5]+ThreePoints[2][1]*ThreePoints[2][1]*ThreePoints[2][1]
        X1_Y1=PointData[6]+ThreePoints[2][0]*ThreePoints[2][1]
        X1_Y2=PointData[7]+ThreePoints[2][0]*ThreePoints[2][1]*ThreePoints[2][1]
        X2_Y1=PointData[8]+ThreePoints[2][0]*ThreePoints[2][0]*ThreePoints[2][1]
        N=PointData[9]+1
        PointData=[X1,Y1,X2,Y2,X3,Y3,X1_Y1,X1_Y2,X2_Y1,N]

    C=N*X2-X1*X1
    D=N*X1_Y1-X1*Y1
    E=N*X3+N*X1_Y2-(X2+Y2)*X1
    G=N*Y2-Y1*Y1
    H=N*X2_Y1+N*Y3-(X2+Y2)*Y1

    a=(H*D-E*G)/(C*G-D*D)
    b=(H*C-E*D)/(D*D-G*C)
    c=-(a*X1+b*Y1+X2+Y2)/N

    Xc=-a/2.0
    Yc=-b/2.0
    R=sqrt(a*a+b*b-4*c)/2.0
        
    return [Xc ,Yc ,R, PointData]
    
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    if OTHER==None:
        OTHER=[]
        OTHER.append([measurement]) # OTHER[0] reserved to record the newest three measurements
        OTHER.append(0.0) #OTHER[1] for total step_dist
        OTHER.append(False) ##OTHER[2] for clockwise
        OTHER.append(1) #OTHER[3] counter
        OTHER.append([]) #OTHER[4] PointData for circle fitting
        xy_estimate=measurement
        return xy_estimate, OTHER 
    elif len(OTHER[0])<2:
        OTHER[3]+=1 #counter
        OTHER[0].append(measurement) # OTHER[0][1]
        OTHER[0].append(measurement) ## OTHER[0][2] reserved in advance
        step_dist=distance_between(OTHER[0][1],OTHER[0][0])
        OTHER[1]+=step_dist
        xy_estimate=measurement
        return xy_estimate, OTHER 
    else:
        OTHER[3]+=1 #counter
        times_of_trial=OTHER[3]
        Z=measurement 
        OTHER[0][2]=Z #update OTHER[0][2] for the newest measurement
        Z_prev=OTHER[0][1] ## the previous measurement
        Z_prev2=OTHER[0][0] ## the one prior to Z_prev

        #calculate average step distance
        step_dist=distance_between(Z,Z_prev)
        OTHER[1]+=step_dist #OTHER[1] total step distance
        aver_step=OTHER[1]/(times_of_trial-1)
        
        Xc, Yc, R, OTHER[4]=Circle_Fitting(OTHER[0],OTHER[4])
        #print Xc, Yc, R, aver_step
        
        ##to make sure the bot moves clockwisely or not
        thetaZ_prev2=atan2(Z_prev2[1]-Yc,Z_prev2[0]-Xc)
        thetaZ_prev=atan2(Z_prev[1]-Yc,Z_prev[0]-Xc)
        thetaZ=atan2(Z[1]-Yc,Z[0]-Xc)
        if thetaZ<0:
            thetaZ+=2*pi ##make thetaZ between 0 and 2pi
        if thetaZ_prev<0:
            thetaZ_prev+=2*pi
        if thetaZ_prev2<0:
            thetaZ_prev2+=2*pi
        
        #if the runaway bot moves "too straight" or the measurement error is too large, it's better to check this every time 
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
        OTHER[2]=Clockwise

        d_theta=aver_step/R ## average travel degree
        if OTHER[2]==False: 
            THETA=thetaZ+d_theta
        else:
            THETA=thetaZ-d_theta

        next_x=Xc+R*cos(THETA)
        next_y=Yc+R*sin(THETA)
        xy_estimate=[next_x,next_y]

        #update OTHER[0][0] OTHER[0][1] to get new measurement for next try
        OTHER[0][0]=OTHER[0][1]
        OTHER[0][1]=OTHER[0][2] 
        
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




