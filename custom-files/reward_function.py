import math
import numpy as np

# Define the PID controller class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def control(self, error):
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# Initialize the PID controller
pid = PIDController(kp=1.0, ki=0.0, kd=0.1)

def reward_function(params):
    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    speed = params['speed']
    steering_angle = abs(params['steering_angle'])
    progress = params['progress']
    steps = params['steps']
    is_offtrack = params['is_offtrack']
    all_wheels_on_track = params['all_wheels_on_track']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    waypoints = params['waypoints']
    steering_angle_change = params.get('steering_angle_change', 0.0)
    prev_steering_angles = params.get('prev_steering_angles', [steering_angle])
    prev_speed = params.get('prev_speed', speed)
    x = params['x']
    y = params['y']

    # Initialize reward
    reward = 1.0

    # Penalize if the car goes off-track
    if is_offtrack or not all_wheels_on_track:
        return 1e-3  # Minimal reward if off track

    # Define markers for distance from center
    marker_1 = 0.1 * track_width
    marker_2 = 0.2 * track_width
    marker_3 = 0.3 * track_width
    marker_4 = 0.4 * track_width
    marker_5 = 0.5 * track_width

    # Reward staying close to the center
    if distance_from_center <= marker_1:
        reward += 2.0
    elif distance_from_center <= marker_2:
        reward += 1.5
    elif distance_from_center <= marker_3:
        reward += 1.0
    elif distance_from_center <= marker_4:
        reward += 0.5
    elif distance_from_center <= marker_5:
        reward += 0.2
    # else:
    #     reward *= 0.1  # Larger penalty for being far from the center

    # Calculate track direction and direction difference
    next_waypoint = waypoints[closest_waypoints[1]]
    prev_waypoint = waypoints[closest_waypoints[0]]
    track_direction = np.degrees(np.arctan2(next_waypoint[1] - prev_waypoint[1], next_waypoint[0] - prev_waypoint[0]))
    direction_diff = np.abs(track_direction - heading)

    # Calculate curvature
    def calculate_curvature(waypoints, closest_waypoints):
        waypoint_1 = waypoints[closest_waypoints[0]]
        waypoint_2 = waypoints[closest_waypoints[1]]
        waypoint_3 = waypoints[(closest_waypoints[1] + 1) % len(waypoints)]
        curvature = np.abs(np.arctan2(waypoint_3[1] - waypoint_2[1], waypoint_3[0] - waypoint_2[0]) -
                           np.arctan2(waypoint_2[1] - waypoint_1[1], waypoint_2[0] - waypoint_1[0]))
        return curvature

    curvature = calculate_curvature(waypoints, closest_waypoints)
    if curvature < 0.1 :
        if speed > 3:
            if direction_diff< 5: 
                reward *= 1.7
            else:
                reward*=1.5
        if speed > 2.5:
            if direction_diff< 5:
                reward *= 1.3
        elif speed < 1.5:# Penalize :
            reward*=0.8
    else:
        if direction_diff > 5:
            reward *= 0.8
        else:
            if speed > 1.5:
                reward*=1.2
    # if curvature < 0.1:
    #     optimal_speed = 3.0  # Max speed on straight paths
    # else:
    #     optimal_speed = max(1.0, 3.0 - curvature * 8)

    # Reward for maintaining an optimal speed
    # speed_diff = abs(speed - optimal_speed)
    # if speed_diff < 0.1:
    #     reward += 2.0
    # elif speed_diff < 0.2:
    #     reward += 1.0
    # else:
    #     reward += 0.5

    # Penalize for going too fast in curves
    # if speed > optimal_speed and curvature > 0.1:
    #     reward *= 0.5

    # Reward for smooth speed transitions
    # SPEED_STABILITY_THRESHOLD = 0.1
    # if np.abs(speed - prev_speed) < SPEED_STABILITY_THRESHOLD:
    #     reward += 1.5  # Increased emphasis on smooth speed transitions
    # Penalize for excessive braking
    # BRAKING_THRESHOLD = 0.3
    # if prev_speed - speed > BRAKING_THRESHOLD:
    #     reward *= 0.8

    # Penalize for too much steering (to prevent zig-zag behavior)
    ABS_STEERING_THRESHOLD = 0.3
    if steering_angle > ABS_STEERING_THRESHOLD:
        reward *= 0.4

    # Penalize for frequent steering changes
    OSCILLATION_THRESHOLD = 0.2
    if len(prev_steering_angles) > 1:
        steering_deltas = np.abs(np.diff(prev_steering_angles[-5:]))
        if np.any(steering_deltas > OSCILLATION_THRESHOLD):
            reward *= 0.5

    # Reward for smooth steering
    SMOOTH_STEERING_THRESHOLD = 0.2
    if steering_angle_change < SMOOTH_STEERING_THRESHOLD:
        reward += 1.5  # Increased emphasis on smooth steering

    # Penalize if the direction difference is too large
    DIRECTION_THRESHOLD = 3.0  # Reduced to tighten control
    if direction_diff < DIRECTION_THRESHOLD:
        reward *= 1.4

    # Apexing: Reward for being close to the inside edge of the turn (apex)
    apex_threshold = 0.2 * track_width
    if distance_from_center < apex_threshold:
        reward += 1.0

    # Reward for maximizing speed on straight sections
    if curvature < 0.1 and speed > 3:
        reward += 1.0

    # Penalize for unnecessary steering adjustments
    # if steering_angle_change > 0.3:
    #     reward *= 0.8

    # Use PID controller for steering correction
    steering_error = steering_angle_change
    pid_correction = pid.control(steering_error)
    if abs(pid_correction) < 0.2:
        reward += 2.0  # Reward for low PID correction (indicating smooth driving)

    # Reward for higher exit speed rather than entry speed
    if steps > 1 and speed > prev_speed:
        reward += 1.0
    if curvature<0.1 and direction_diff==0:
        if steering_angle_change > 0.1:
            reward *= 0.8
    # Penalize for longer paths (encourage taking the shortest path)
    prev_x, prev_y = waypoints[closest_waypoints[0]]
    # distance_traveled = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
    # OPTIMAL_DISTANCE_PER_STEP = 0.4
    # if distance_traveled <= OPTIMAL_DISTANCE_PER_STEP:
    #     reward += 1.0
    # else:
    #     reward *= 0.8

    # Progress-based reward
    reward += (progress / 100.0) * 1.5

    # Additional reward for completing the track faster
    TOTAL_NUM_STEPS = 240
    if progress == 100:
        reward += 100 * (1 - (steps / TOTAL_NUM_STEPS))
    SPEED_CONSISTENCY_THRESHOLD = 0.1
    if curvature<0.1:
        if np.abs(speed - prev_speed) < SPEED_CONSISTENCY_THRESHOLD:
            reward *= 1.2

    # Reward for consistency in speed
    # SPEED_CONSISTENCY_THRESHOLD = 0.1
    # if np.abs(speed - prev_speed) < SPEED_CONSISTENCY_THRESHOLD:
    #     reward += 1.2

    # Incremental Progress Reward
    reward += (progress / 100.0) * 2.0

    # Time-based milestones
    MILESTONE_REWARD = 5.0
    if progress >= 25 and steps < 90:
        reward += MILESTONE_REWARD
    if progress >= 50 and steps < 180:
        reward += MILESTONE_REWARD
    if progress >= 75 and steps < 240:
        reward += MILESTONE_REWARD

    return float(reward)
                                                                                