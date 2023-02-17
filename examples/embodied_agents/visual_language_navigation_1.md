Imagine I am a robot equipped with a camera and a depth sensor. I am trying to perform a task, and you should help me by sending me commands. You are only allowed to give me the following commands:

- turn(angle): turn the robot by a given number of degrees
- move(distance): moves the robot straight forward by a given distance in meters.

On each step, I will provide you with the objects in the scene as a list of \<object name, distance, angle in degrees\>. You should reply with only one command at a time.\
The distance is in meters, and the direction angle in degrees with respect to the robot's orientation. Negative angles are to the left and positive angles are to the right. If a command is not valid, I will ignore it and ask you for another command. If there is no relevant information in the scene, use the available commands to explore the environment.

<img src="img/ex_1-frame_1.jpg" height="150"> 

Task: go to the chairs \
Objects in the scene: \<door, 0.53 m, 22\>, \<chair, 4.84 m, -21\>, \<chair, 2.12 m, -24\>, \<window, 6.74 m, -3\>, \<window, 7.17 m, 6\>, \<door, 0.61 m, 18\>, \<stairs, 1.42 m, 0\>\
Command:  


---------------------------------------------------------------------------

```
move(1.5) # Move forward by 1.5 meters to explore the environment. 
```
<img src="img/ex_1-frame_2.jpg" height="150"> 

---------------------------------------------------------------------------

Objects in the scene: \<seating, 5.05 m, 21\>, \<chair, 3.93 m, -25\>, \<window, 4.55 m, 22\>, \<window, 5.24 m, -4\>, \<stairs, 3.49 m, 13\>, \<window, 5.67 m, 7\>, \<window, 4.37 m, 23\>\
Command:  

---------------------------------------------------------------------------

``` 
turn(-25) # Turn to the left to face the chair that is 3.93 meters away. 
``` 
---------------------------------------------------------------------------

Objects in the scene: \<chair, 4.85 m, -14\>, \<chair, 4.76 m, -9\>, \<chair, 1.57 m, -23\>, \<chair, 2.66 m, -24\>, \<window, 4.96 m, 10\>, \<window, 4.19 m, 25\>\
Command:  

---------------------------------------------------------------------------

``` 
turn(-10) # Turn to the right to explore the environment further. 
``` 

---------------------------------------------------------------------------

Objects in the scene: \<door, 5.07 m, -25\>, \<chair, 5.36 m, -6\>, \<table, 2.17 m, -24\>, \<chair, 5.01 m, -1\>, \<chair, 1.80 m, -16\>, \<chair, 3.15 m, -16\>, \<window, 4.30 m, 19\>\
Command:  

---------------------------------------------------------------------------

```
turn(-6) # Turn to the right to face the chair that is 5.36 meters away. 
``` 

---------------------------------------------------------------------------

Objects in the scene: \<counter, 1.24 m, -24\>, \<door, 5.93 m, -17\>, \<chair, 5.48 m, 1\>, \<table, 2.60 m, -14\>, \<chair, 4.92 m, 6\>, \<chair, 2.02 m, -7\>, \<chair, 3.53 m, -7\>, \<chair, 3.16 m, -18\>\
Command:  

---------------------------------------------------------------------------

```
turn(1) # Turn to the left to explore the environment further. 
```
<img src="img/ex_1-frame_2_.jpg" height="150"> 

---------------------------------------------------------------------------
 

Objects in the scene: \<door, 5.07 m, -25\>, \<chair, 5.36 m, -6\>, \<table, 2.17 m, -24\>, \<chair, 5.01 m, -1\>, \<chair, 1.80 m, -16\>, \<chair, 3.15 m, -16\>, \<window, 4.30 m, 19\>\
Command:  

---------------------------------------------------------------------------

```
move(5.5) # Move forward towards the chair that is 5.36 meters away. 
```

---------------------------------------------------------------------------

Objects in the scene: \<chair, 0.96 m, -10\>, \<chair, 0.99 m, 20\>, \<chair, 1.77 m, -20\>\
Command:  

---------------------------------------------------------------------------

```
move(1) # Move forward to approach the chairs. 
```
<img src="img/ex_1-frame_3.jpg" height="150"> 

---------------------------------------------------------------------------
---------------------------------------------------------------------------
 
</br>
</br>
</br>
Full navigation:

<img src="img/nav_chairs_2.gif" height="150"> 
