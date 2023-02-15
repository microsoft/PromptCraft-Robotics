<b>
Let us assume we are working with three robots. 

One is an embodied agent, which can be controlled through the following functions:

move_forward(): Moves the agent forward by 1 m.
turn_right(): Rotates the agent right by 90 degrees at its current position.
turn_left(): Rotates the agent left by 90 degrees at its current position.

Another is a small car, which can be controlled by:

set_velocity(value): Set the velocity of the car (in m/s).
set_steering_angle(value): Set the steering angle to specified value (in degrees).

One of them is a drone, that navigates in 3D. This can be controlled using the following functions 

set_height(value): Moves the drone to an altitude specified by value (in m).
set_yaw(value): Rotates the drone to a yaw angle specified by value (in degrees).
set_pitch(): Sets the drone pitch to an angle specified by value (in degrees).
set_roll(): Sets the drone roll to an angle specified by value (in degrees).

For the car and the drone, there is another function you can use as well:

get_position(): Outputs the current XYZ position of the robot.

I need you to help me control these agents using the functions specified above. Remember each robot has a different set of constraints on how it is able to move, so you need to account for that when writing code. 

For instance, the drone would have to takeoff, fly, and land in order to get to the target position. Setting the pitch angle would make the drone fly forwards, whereas the roll makes it fly sideways.

The car will only go exactly straight if the steering angle is 0. The yaw angle can only be between -30 and 30 degrees. The car cannot rotate in place so you have to continuously figure out the right steering angle as the car is in motion. Make sure to stop the car once it reaches the target.

The embodied agent can only rotate by 90 degrees and cannot face towards specific angles so several adjustments might be required to get to goal.

Let us assume the X axis is facing east, and Y axis is facing north. All three agents start at (0,0,0), facing north. Give me code that can move all three agents from the XYZ coordinates (0, 0, 0) to (5, 5, 0). Add comments to the code explaining what you are doing, and keep the code compact and correctly formatted.
</b>

---
```
# Embodied agent
def move_embodied_agent_to_target(x, y):
  # rotate to face east
  turn_right()
  
  # move forward until we reach the target x position
  while x > 0:
    move_forward()
    x -= 1
  
  # rotate to face north
  turn_right()
  
  # move forward until we reach the target y position
  while y > 0:
    move_forward()
    y -= 1

move_embodied_agent_to_target(5, 5)


# Small car
def move_car_to_target(x, y):
  # get the initial position
  current_pos = get_position()
  target_pos = (x, y, 0)
  
  # calculate the angle to target position
  angle = math.atan2(target_pos[1]-current_pos[1], target_pos[0]-current_pos[0])
  
  # set initial steering angle
  set_steering_angle(angle)
  set_velocity(1)
  
  # continuously update the steering angle as the car moves
  while current_pos[0] != target_pos[0] or current_pos[1] != target_pos[1]:
    current_pos = get_position()
    angle = math.atan2(target_pos[1]-current_pos[1], target_pos[0]-current_pos[0])
    set_steering_angle(angle)
  
  # stop the car
  set_velocity(0)

move_car_to_target(5, 5)


# Drone
def move_drone_to_target(x, y, z):
  # take off
  set_height(1)
  
  # fly to the target x and y position
  angle = math.atan2(y, x)
  set_yaw(angle)
  set_pitch(45)
  
  # continuously update the drone position
  current_pos = get_position()
  while current_pos[0] != x or current_pos[1] != y:
    current_pos = get_position()
  
  # fly to the target z position
  set_pitch(0)
  set_height(z)
  
  # land
  set_height(0)

move_drone_to_target(5, 5, 0)
```