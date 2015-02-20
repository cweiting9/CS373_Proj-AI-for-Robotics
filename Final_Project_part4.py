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

def cte(r,center,position):
    cte=0.0
    dist_from_center=distance_between(center,position)
    cte=dist_from_center-r
    return cte

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
        OTHER.append([]) #OTHER[5] Xc, Yc, R, aver_step_dist
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
        OTHER[5]=[Xc, Yc, R, aver_step]
              
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


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 
    # First, use PD control to move on the track in the opposite direction
    # When they bump into each other, start move on a Hexagonal track on the same direction
    # If the hunter is hehind the target, then switch to PD control
    if OTHER==None: #first measurement
        OTHER=[]
        target_OTHER=None
        target_est, target_OTHER=estimate_next_pos(target_measurement, target_OTHER)
        OTHER.append(target_OTHER)#OTEHR[0]:target_OTHER for estimate_next_pos
        OTHER.append(False) # OTEHR[1] flag of Path points
        OTHER.append(0) #OTHER[2] counter of staying the same point
        OTHER.append(0) #OTHER[3] index of the Path points
        OTHER.append(True) #OTHER[4] flag of PD control
        OTHER.append(0) # OTEHR[5] for cte
        OTHER.append([]) # OTHER[6] hunter position theta in the end of moving in the opposite direction 
        
        
        #to decide turning degree and moveing distance of the hunter
        #if distance between target and hunter is larger than hunter's max speed,
        #then hunter moves at full speed. Otherwise, move the exact distance between them 
        diff_heading=get_heading(hunter_position, target_est)
        turning=angle_trunc(diff_heading-hunter_heading)
        dist=distance_between(target_est,hunter_position)
        if dist>max_distance:
            distance=max_distance
        else:
            distance=dist

    else:
        #to retrive the necessary information from OTHER
        target_est, target_OTHER=estimate_next_pos(target_measurement, OTHER[0])
        OTHER[0]=target_OTHER
        dist=distance_between(target_est,hunter_position)
        #OTHER[0][5]==0 states that there's still not enough measurements for Xc, Yc, R
        #just simply move toward the target
        if len(OTHER[0][5])==0:
            diff_heading=get_heading(hunter_position, target_est)
            turning=angle_trunc(diff_heading-hunter_heading)
            
            if dist>max_distance:
                distance=max_distance
            else:
                distance=dist
        #When the hunter has the information Xc ,Yc ,R, step distace
        else:
            #to retrive Xc, Yc, R and step_dist from estimate_next_pos
            Xc, Yc ,R, aver_step_dist=OTHER[0][5]
            # PD control part
            if OTHER[4]==True:
                #if the distance between the hunter and the center of the track of the target is not between
                #(R+1.1*aver_step_dist) and (R-1.1*aver_step_dist), then the hunter moves toward the postion
                #which is the nearest point on the track to the hunter
                dist_center=distance_between(hunter_position,[Xc,Yc])
                if dist_center>R+1.1*aver_step_dist or dist_center<R-1.1*aver_step_dist:
                    nearest_theta=atan2(hunter_position[1]-Yc,hunter_position[0]-Xc)
                    nearest_point=[Xc+R*cos(nearest_theta),Yc+R*sin(nearest_theta)]
                    diff_heading=get_heading(hunter_position, nearest_point)
                    turning=angle_trunc(diff_heading-hunter_heading)
                    dist_nearest=distance_between(nearest_point,hunter_position)
                    if dist_nearest>max_distance:
                        distance=max_distance
                    else:
                        distance=dist_nearest
               #when the hunter is near the track of the target, use PD control to move on the track
                else:             
                    crosstrack_error=cte(R,[Xc, Yc],hunter_position)
                    diff_crosstrack_error=-OTHER[5] #OTHER[5]: last measurement's cte 
                    OTHER[5]=crosstrack_error #update OTHER[5] for next move
                    #if the bot is unreachable, then move on the track of the circle in the opposite direction 
                    if dist>max_distance:
                        distance=0.5*max_distance
                        diff_crosstrack_error+=crosstrack_error
                        if OTHER[0][2]==False: ##if the bot moves counterclockwisely
                            turning=-1.0*crosstrack_error-1.0*diff_crosstrack_error
                        else:
                            turning=1.0*crosstrack_error+1.0*diff_crosstrack_error
                    #if the bot is reachable, then go for it
                    else:
                        distance=dist
                        diff_heading=get_heading(hunter_position, target_est)
                        turning=angle_trunc(diff_heading-hunter_heading)
                        OTHER[6]=atan2(hunter_position[1]-Yc,hunter_position[0]-Xc)
                        # Switch to Hexagonal track and initilize the counters
                        OTHER[4]=False
                        OTHER[3]=0
                        OTHER[2]=0

            else:
                # Hexagonal track part
                current_theta=OTHER[6]
                # counterclockwise
                Path_Theta=[2*pi/3.0, 4*pi/3.0, 6*pi/3.0]
                #Path_Theta=[pi/3.0, 2.0*pi/3.0, 3.0*pi/3.0, 4.0*pi/3.0, 5.0*pi/3.0, 6.0*pi/3.0]
                if OTHER[0][2]==True: # clockwise
                    #Path_Theta=[-pi/3.0, -2.0*pi/3.0, -3.0*pi/3.0, -4.0*pi/3.0, -5.0*pi/3.0, -6.0*pi/3.0]
                    Path_Theta=[-2*pi/3.0, -4*pi/3.0, -6*pi/3.0]
                    
                # Make the path points theta relative to the current_theta
                Path_Points=[]
                for theta in Path_Theta:
                    Path_Points.append([Xc+R*cos(current_theta+theta),Yc+R*sin(current_theta+theta)])
                
              
          
                Path_index=OTHER[3]
                Path_position=Path_Points[Path_index]

                dist_Path=distance_between(hunter_position,Path_position)
                dist=distance_between(hunter_position,target_est)

                # If the target is reachable, then go for it or heading to the next point of Hexagonal track
                if dist<max_distance:
                    diff_heading=get_heading(hunter_position, target_est)
                    turning=angle_trunc(diff_heading-hunter_heading)
                    distance=dist
                    # Dont add counter repeatedly
                    if OTHER[1]==False:
                        OTHER[3]=(OTHER[3]+1)%3
                        OTHER[1]=True
                else:
                    
                    diff_heading=get_heading(hunter_position, Path_position)
                    turning=angle_trunc(diff_heading-hunter_heading)
                    if dist_Path>max_distance:
                        OTHER[1]=False
                        distance=max_distance
                    else:
                        distance=dist_Path
                        OTHER[2]+=1
                        # If staying too long, then switch back
                        if OTHER[2]>3:
                            OTHER[4]=True
                            
                                      
    return turning, distance, OTHER
    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
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

target = robot(0.0, 10.0, 0.0, -2*pi / 34.0, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10, -10, 0.0)
print demo_grading(hunter, target, next_move)

#backup
'''
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    if OTHER==None: #first measurement
        OTHER=[]
        target_OTHER=None
        target_est, target_OTHER=estimate_next_pos(target_measurement, target_OTHER)
        OTHER.append(target_OTHER)#OTEHR[0]:target_OTHER for estimate_next_pos
        OTHER.append(0) # OTEHR[1] for cte

        #to decide turning degree and moveing distance of the hunter
        #if distance between target and hunter is larger than hunter's max speed,
        #then hunter moves at full speed. Otherwise, move the exact distance between them 
        diff_heading=get_heading(hunter_position, target_est)
        turning=angle_trunc(diff_heading-hunter_heading)
        dist=distance_between(target_est,hunter_position)
        if dist>max_distance:
            distance=max_distance
        else:
            distance=dist

    else:
        #to retrive the necessary information from OTHER
        target_est, target_OTHER=estimate_next_pos(target_measurement, OTHER[0])
        OTHER[0]=target_OTHER
        dist=distance_between(target_est,hunter_position)
        #OTHER[0][5] states that there's still not enough measurements for Xc, Yc, R
        #just simply move toward the target
        if len(OTHER[0][5])==0:
            diff_heading=get_heading(hunter_position, target_est)
            turning=angle_trunc(diff_heading-hunter_heading)
            
            if dist>max_distance:
                distance=max_distance
            else:
                distance=dist
        #When the hunter has the information Xc ,Yc ,R, step distace
        else:
            #to retrive Xc, Yc, R and step_dist from estimate_next_pos
            Xc, Yc ,R, aver_step_dist=OTHER[0][5] 

            #if the distance between the hunter and the center of the track of the target is not between
            #(R+1.1*aver_step_dist) and (R-1.1*aver_step_dist), then the hunter moves toward the postion
            #which is the nearest point on the track to the hunter
            dist_center=distance_between(hunter_position,[Xc,Yc])
            if dist_center>R+1.1*aver_step_dist or dist_center<R-1.1*aver_step_dist:
                nearest_theta=atan2(hunter_position[1]-Yc,hunter_position[0]-Xc)
                nearest_point=[Xc+R*cos(nearest_theta),Yc+R*sin(nearest_theta)]
                diff_heading=get_heading(hunter_position, nearest_point)
                turning=angle_trunc(diff_heading-hunter_heading)
                dist_nearest=distance_between(nearest_point,hunter_position)
                if dist_nearest>max_distance:
                    distance=max_distance
                else:
                    distance=dist_nearest
            #when the hunter is near the track of the target, use PD control to move on the track
            else:             
                crosstrack_error=cte(R,[Xc, Yc],hunter_position)
                diff_crosstrack_error=-OTHER[1] #OTHER[1]: last measurement's cte 
                OTHER[1]=crosstrack_error #update OTHER[1] for next move
                #if the bot is unreachable, then move on the track of the circle in the opposite direction 
                if dist>max_distance:
                    distance=0.5*max_distance
                    diff_crosstrack_error+=crosstrack_error
                    if OTHER[0][2]==False: ##if the bot moves counterclockwisely
                        turning=-1.0*crosstrack_error-1.0*diff_crosstrack_error
                    else:
                        turning=1.0*crosstrack_error+1.0*diff_crosstrack_error
                #if the bot is reachable, then go for it
                else:
                    distance=dist
                    diff_heading=get_heading(hunter_position, target_est)
                    turning=angle_trunc(diff_heading-hunter_heading)
            
    return turning, distance, OTHER
'''

