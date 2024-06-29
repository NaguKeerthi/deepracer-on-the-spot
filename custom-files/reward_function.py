import math
def reward_function (params) :
# Extract input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    speed = params['speed']
    steering_angle = abs (params['steering_angle'])
    progress = params['progress']
    steps = params['steps']
    is_offtrack = params['is_offtrack']
    all_wheels_on_track = params ['all_wheels_on_track']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    waypoints = params['waypoints']
    steering_angle_change = params.get('steering_angle_change',0.0)
    prev_speed = params.get('prev_speed', speed)
    # Define markers for distance from center
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width
    # Initialize reward
    reward = 1.0
    # Reward for staying closer to the center line
    # if distance_from_center <= marker_1:
    #     reward *= 1.5
    # elif distance_from_center <= marker_2:
    #     reward *= 1.0
    # elif distance_from_center <= marker_3:
    #     reward *= 0.5 
    # else:
    #     reward *= 0.1 # likely crashed/close to off track
    # Penalize if the car is off track
    stwp=[1,2,3,4,5,6,7,8,22,23,24,25,26,27,28,29,30,31,109,110,111,11,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,143,144,145,146,147,148,149,150,151,152,153,154,155]
    if is_offtrack:
        reward = 1e-3
        return float (reward)
    # Return early if off track
    # Calculate track direction and difference from heading
    next_waypoint = waypoints[closest_waypoints[1]]
    prev_waypoint = waypoints[closest_waypoints[0]]
    track_direction = math.atan2(next_waypoint[1] - prev_waypoint[1], next_waypoint[0] - prev_waypoint[0])
    track_direction = math.degrees(track_direction)
    direction_diff = abs(track_direction - heading)
    
    # Normalize the difference to be within [0, 180] degrees
    if direction_diff > 180:
        direction_diff = 360 - direction_diff
    #speed based on whether the path is straight or curved
    STRAIGHT_PATH_THRESHOLD = 8.0
    # Threshold for straight path
    if closest_waypoints[1] in stwp:
        SPEED_THRESHOLD_STRAIGHT = 2.5
        if speed > SPEED_THRESHOLD_STRAIGHT:
            reward *= 1.2
        elif speed < 1.5:# Penalize if too slow on straight paths
            reward *=0.8
    # Encourage higher speed on straight paths
        
    else:
    # Encourage slower speed on curves
        SPEED_THRESHOLD_CURVE = 3.0
        if direction_diff > 10:
            reward *= 0.8
        else:
            if speed > 1.5:
                reward*=1.2
    # Reward for maintaining optimal speed
    OPTIMAL_SPEED = 3.5
    # if speed == OPTIMAL_SPEED:
    #     reward *= 1.2
    # Penalize excessive braking
    # BRAKING_THRESHOLD = 0.4
    # if prev_speed - speed > BRAKING_THRESHOLD: 
    #     reward *= 0.8
    # Penalize too much steering to prevent zig-zag behavior
    # ABS_STEERING_THRESHOLD = 5.0
    # if steering_angle > ABS_STEERING_THRESHOLD:
    #     reward *= 0.2
    # Reward for smooth steering
    # SMOOTH_STEERING_THRESHOLD = 3.0
    # if steering_angle_change < SMOOTH_STEERING_THRESHOLD:
    #     reward *= 1.6
    # Penalize for oscillation (rapid back-and-forth steering)
    # OSCILLATION_THRESHOLD = 15.0
    # if abs (steering_angle_change) > OSCILLATION_THRESHOLD:
    #     reward *= 0.7
    # Penalize large direction differences
    # DIRECTION_THRESHOLD = 5.0
    # if direction_diff > DIRECTION_THRESHOLD:
    #     reward *= 0.5
    # Progress-based reward
    reward += (progress / 100.0) * 1.5
    # Additional reward for completing the track faster
    # TOTAL_NUM_STEPS = 300
    # if progress == 100:
    #     reward += 100 * (1 - (steps / TOTAL_NUM_STEPS))
    # Reward for consistency in speed
    # SPEED_CONSISTENCY_THRESHOLD = 0.2
    # if abs (speed - prev_speed)< SPEED_CONSISTENCY_THRESHOLD:
    #     reward *= 1.2
    
    return float (reward)
