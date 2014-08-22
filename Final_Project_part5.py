# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY 
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    return turning, distance, OTHER

def cte(r,center,position):
    cte=0.0
    dist_from_center=distance_between(center,position)
    cte=dist_from_center-r
    return cte
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

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 
    if OTHER==None:
        OTHER=[]
        target_OTHER=None
        target_est, target_OTHER=estimate_next_pos(target_measurement, target_OTHER)
        OTHER.append(target_OTHER)
        OTHER.append(0) ## for cte
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
        
        if len(OTHER[0][4])==0:    ##when no aver center and aver step_distance
            diff_heading=get_heading(hunter_position, target_est)
            turning=angle_trunc(diff_heading-hunter_heading)
            if dist>max_distance:
                distance=max_distance
            else:
                distance=dist
        else:
            aver_cen_step=OTHER[0][4]
            crosstrack_error=cte(aver_cen_step[2],[aver_cen_step[0],aver_cen_step[1]],hunter_position)
            if OTHER[1]==0:
                OTHER[1]=crosstrack_error
                diff_heading=get_heading(hunter_position, target_est)
                turning=angle_trunc(diff_heading-hunter_heading)
                if dist>max_distance:
                    distance=max_distance
                else:
                    distance=dist
                
            else:
                diff_crosstrack_error=-OTHER[1]
                OTHER[1]=crosstrack_error
                if dist>max_distance:
                    distance=0.5*max_distance
                    diff_crosstrack_error+=crosstrack_error
                    if OTHER[0][3]==False:
                        turning=-1.0*crosstrack_error-1.0*diff_crosstrack_error
                    else:
                        turning=1.0*crosstrack_error+1.0*diff_crosstrack_error
                else:
                    distance=dist
                    diff_heading=get_heading(hunter_position, target_est)
                    turning=angle_trunc(diff_heading-hunter_heading)
            

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
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
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
    size_multiplier = 15.0 #change size of animation
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
measurement_noise = 2.0*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, naive_next_move)





