import numpy as np
import time

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    high_speed = 1.0
    rock_vel = 0.4
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        print("Rover Nav Angles........====:",np.mean(Rover.nav_angles))
        #start the timer for between modes

#================'Forward' Mode ===================#
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain

            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set

                elif Rover.vel >= Rover.max_vel: # Else coast
                    Rover.throttle = 0
                    Rover.brake = 0

                select_angles_within_range = []
                increment_steering_L = 17
                increment_steering_R = 18
                for i in range(len(Rover.obstacle_angles)):
                    if Rover.obstacle_angles[i] < (50 * (np.pi/180)) and Rover.obstacle_angles[i] > (16*(np.pi/180)):
                        select_angles_within_range.append(Rover.obstacle_dists[i])
                dist_to_wall = np.mean(select_angles_within_range)

                #todo: ideally implement simple PID, but only when works ok.
                if dist_to_wall >= 132:
                    # If too far away, steer away from wall
                    #print("DISTANCE GETTING TOO FAR! +++++ dist: ",dist_to_wall)
                    Rover.steer = np.clip(((np.mean(Rover.nav_angles*180/np.pi)) + increment_steering_L * 1.)/2. ,-15,15)

                elif dist_to_wall < 132:
                    #If too close, steer away from the wall
                    Rover.steer = np.clip(((np.mean(Rover.nav_angles*180/np.pi)) + increment_steering_R * -1.)/2.,-15,15)
                    #print("DISTANCE GETTING CLOSE! +++++ dist: ",dist_to_wall)


                if (len(Rover.obstacle_angles) > 300) and Rover.vel <0.05:
                    Rover.mode = 'stop'

                if Rover.rock_angles is not None:
                    if len(Rover.rock_angles) > 0:
                        Rover.mode = 'rock'
                #todo.. implement a time setting for stuck.. change to 'stop' if stuck
                #todo.. implement repulsive force of obstacles similar to rock mode steering



            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                 # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = 0
                Rover.steer = 0
                Rover.mode = 'stop'
            elif (len(Rover.nav_angles) < Rover.stuck) and Rover.vel < 0.1:
                 # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.mode = 'stop'

#================'STOP' Mode ===================#
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if (len(Rover.nav_angles) >= Rover.go_forward):
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'


#============='ROCK' Mode =================#
        elif Rover.mode == 'rock':
            #if (len(Rover.rock_angles) > 0):
            if Rover.vel > high_speed:
                Rover.max_vel = rock_vel
                Rover.throttle = 0
                #Rover.brake = 0.05
                if Rover.rock_angles is not None:
                    if (len(Rover.rock_angles) > 0) and (len(Rover.nav_angles) > 0):

                        mean_rock_angle = np.mean(Rover.rock_angles * 180/np.pi) #convert to angle in deg
                        mean_rock_dist = np.mean(Rover.rock_dists)
                        mean_nav_angle = np.mean(Rover.nav_angles * 180/np.pi)

                        weighted_rock_nav_angle = np.average((mean_rock_angle,mean_nav_angle),weights=(.7,.30))
                        Rover.steer = np.clip(weighted_rock_nav_angle, -15,15)

            # otherwise if not near a rock, go into stop mode or steer towards rock
            elif Rover.near_sample == 0:
                if Rover.vel == 0 or Rover.vel < 0:   #todo: check this change from == 0 :
                        #check if Rover is picking up?
                        if Rover.picking_up == 0:
                            if len(Rover.rock_angles) == 0:
                                Rover.brake = 0
                                Rover.mode = 'stop'
                                Rover.max_vel = 1

                            elif Rover.rock_angles is not None:
                                #Rover.brake = 0.05
                                if len(Rover.rock_angles) > 0:
                                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15,15)
                                    Rover.brake = 0
                                    Rover.throttle = 0.1
                                    Rover.max_vel = 1

                        # Method of breaking out of Rock mode once it has picked up
                        elif Rover.picking_up == 1:
                            Rover.mode = 'stop'
                            Rover.max_vel = 1
                            Rover.brake = 0

                # steering to avoid getting suck
                elif Rover.vel > rock_vel:
                    Rover.brake = 0
                    Rover.max_vel = rock_vel
                    if Rover.rock_angles is not None:
                        if len(Rover.rock_angles) > 0:
                            Rover.throttle = 0.1 # was 0.1
                            Rover.max_vel = rock_vel

                            if Rover.rock_angles is not None:
                                if (len(Rover.rock_angles) > 0) and (len(Rover.nav_angles) > 0):

                                    mean_rock_angle = np.mean(Rover.rock_angles * 180/np.pi) #convert to angle in deg
                                    mean_rock_dist = np.mean(Rover.rock_dists)
                                    mean_nav_angle = np.mean(Rover.nav_angles * 180/np.pi)
                                    # weighted average towards rock over the path ahead
                                    weighted_rock_nav_angle = np.average((mean_rock_angle,mean_nav_angle),weights=(0.7,0.3))
                                    Rover.steer = np.clip(weighted_rock_nav_angle, -15,15)

                elif (Rover.vel <= rock_vel) and (Rover.vel >= 0.1):
                    Rover.brake = 0
                    Rover.max_vel = rock_vel
                    if Rover.rock_angles is not None:
                        if len(Rover.rock_angles) > 0:
                            Rover.throttle = 0.1 #was 0.1
                            Rover.max_vel = rock_vel
                            if Rover.rock_angles is not None:
                                if (len(Rover.rock_angles) > 0) and (len(Rover.nav_angles) > 0):

                                    mean_rock_angle = np.mean(Rover.rock_angles * 180/np.pi) #convert to angle in deg
                                    mean_rock_dist = np.mean(Rover.rock_dists)
                                    mean_nav_angle = np.mean(Rover.nav_angles * 180/np.pi)
                                    # weighted average towards rock over the path ahead
                                    weighted_rock_nav_angle = np.average((mean_rock_angle,mean_nav_angle),weights=(0.7,0.3))
                                    Rover.steer = np.clip(weighted_rock_nav_angle , -15,15)

                            #todo: put in something to get around being stuck on obstacles whilst seeing a rock!


                    #when no rocks can be seen, Rover needs to go into 'stop' mode
                    elif Rover.rock_angles is None:
                        if Rover.vel == 0:
                            Rover.send_pickup = False
                            Rover.mode = 'stop'
                            Rover.max_vel = 1


            elif Rover.near_sample == 1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                if Rover.picking_up == 0:
                    if Rover.vel == 0:
                        Rover.throttle = 0
                        Rover.brake = 1
                        Rover.send_pickup = True
                        Rover.brake = 0
                        #Rover.mode = 'stop'



    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

    # Just to make the rover do something
    # even if no modifications have been made to the code


