import numpy as np
import random as random

def move_to_sample(Rover):
    delX = 0; delY = 0; 
    if len(Rover.rock_angles) > 0:
        dist_to_rock = np.mean(np.abs(Rover.rock_dist))
        angle_to_rock = np.mean(Rover.rock_angles);
        Rover.steer = np.clip(angle_to_rock* 180/np.pi, -15, 15) 
        if Rover.vel>0.5:
            Rover.brake = 0.1;
        else:
            Rover.brake = 0;
        Rover.throttle = 0;
        
        if Rover.vel <0.2 and Rover.near_sample == 0:
           Rover.throttle = 0.1;
           Rover.brake = 0 ;
    if Rover.Is_Stuck:
        Rover.brake = 0;
        Rover.throttle = Rover.StuckThrottle;
        Rover.steer = Rover.StuckSteering;
        
    if Rover.near_sample:
        Rover.brake = Rover.brake_set;
    return Rover

def is_terrain_navigable(Rover):
    if len(Rover.nav_dists)>0:
        if (len(Rover.nav_angles) < Rover.stop_forward):
            terrain_navigable = 0;
        else:
            terrain_navigable = 1;
    else:
        terrain_navigable = 0;
        
    return terrain_navigable;

def is_rover_stuck(Rover):
    SteerVel = np.mean(np.diff(Rover.SteerVel[0:6]));
    no_new_area_mapped = (np.abs(Rover.new_perc_mapped - Rover.old_perc_mapped) <= 0.25); 
    rover_unstucking = (np.abs(Rover.total_time - Rover.map_time) <= 2);
    rover_steer_not_changing = (np.abs(SteerVel) <= 2);
    is_rover_stuck = no_new_area_mapped and rover_steer_not_changing and rover_unstucking;
    
    if no_new_area_mapped and rover_steer_not_changing and Rover.Is_Stuck == 0:
        # Rover was not stuck before, but is stuck now
        Rover.Is_Stuck = 1;
        Rover.StuckSteering = np.random.randint(-15, 16);
        Rover.StuckThrottle = np.random.randint(-1, 2);
    
    if Rover.Is_Stuck and ~rover_unstucking:
        # Rover unstucking is done
        Rover.Is_Stuck = 0;    
    
# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    # If there are, we'll step through the known sample positions
    # to confirm whether detections are real
    is_rover_stuck(Rover);
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
            #if near a sample, navigate towards the sample and stop
        if (Rover.samples_located > Rover.samples_collected):
            Rover = move_to_sample(Rover);
        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if Rover.Is_Stuck:
                # Rover is stuck, unstuck it
                Rover.brake = 0;
                Rover.throttle = Rover.StuckThrottle;
                Rover.steer = Rover.StuckSteering;
            elif is_terrain_navigable(Rover):
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                
                Weighted_Angles = np.mean(Rover.nav_angles);
                Rover.steer = np.clip(np.mean(Weighted_Angles * 180/np.pi), -15, 15)
            else:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = -15; 
                Rover.mode = 'stop';

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
                    # Release the brake to allow turning
                    Rover.brake = 0;
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if is_terrain_navigable(Rover):
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
       
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode = 'stop';
    
    Rover.SteerVel[0:9] = Rover.SteerVel[1:10];
    Rover.SteerVel[9] = Rover.steer;
    
    if Rover.vel == 0:
        Rover.Ok_To_Map = 0;
    else:
        Rover.Ok_To_Map = 1;
    
    return Rover


