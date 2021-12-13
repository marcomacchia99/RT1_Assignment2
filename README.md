Assignment 2 - Research Track 1 
================================

-----------------------

Introduction
------------

In this assignment we have to develop a robot that should `drive autonomously` inside the [Autodromo Nazionale di Monza](https://www.monzanet.it/) , `paying attention to not collide with the track limits`.

Installing and running
----------------------

The simulator requires [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__) to be installed on the machine. In particular, the [Noetic Release of ROS](http://wiki.ros.org/noetic/Installation) was used.

In order to run the simulation, first you have to run ROS (using ```$ roscore &``` and ```$ catkin_make``` ), then you should run this commands, one per console page:

```console
$ rosrun stage_ros stageros $(rospack find RT1_Assignment2)/world/my_world.world
```

(This particular command will open the game circuit)


```console
$ rosrun RT1_Assignment2 controller
```
(This particular command will run the controller node used to drive autonomously)

```console
$ rosrun RT1_Assignment2 server
```
(This particular command will run the service used to increase or decrease speed)

```console
$ rosrun RT1_Assignment2 UI
```
(This particular command will run the UI that can control the velocity)

Game environment
---------

Here's the Monza track used in this game:

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/world/tracciato.png)

ROS generate the track arena based on this image, using the file `.world` contined inside the `world` folder. 

Controller node
--------------

The controller node is capable of driving potentially indefinitely all around the track, automatically detecting straights and turns. When the robot is approaching a turn, the node automatically tells him to slow down, so that he can make the right controls.

The controller uses all the sensors data received by the `/base_scan` publisher after he subscribes to it. this topic is composed by 720 _ranges_, in which there are all the detected distances. the sensor can see from -90 to 90 degrees, so each sensor has 1/4 of degree of view.

After a message from `/base_scan` is recieved, the controller node enters inside the `checkTrackLimits` function, that filters all the ranges taking only the one from:
* -90° to -70°, 
* -10° to 10°,
* 70° to 90°. 

After that the function checks for the minimum value inside each of the three sets, and choose what action has to be done:

* if the front wall is nearer then `f_th = 2`meters, he checks the lateral distances:
  * if the left distance is more then the right distance he slightly turns to the right
  * otherwise he slightly turns to the left
* if the front wall is furthest then the treshold, then the robot goes straight.


When the robot goes straight, he uses the `/Speed_val` value as speed value. `/Speed_val` is managed by the UI node, according to what he receives from the `/accelerator` node. 

After doing that, the controller node publish the data to the `/cmd_vel` topic, used to control the robot movement.


Service node
--------------

The service node manages the robot's speed, and he works in close contact with the UI node, used to interact with the final user. It simply checks the character received by the UI node and increments or decrements the speed by 0,5. Also, if the button R is pressed, the service automatically resets the robot position to its initial position and velocity, using the `/reset_positions` service.

UI node
------

The UI node is responsible of receving the user input from the keyboard. Firstly it reads the input command, then it checks if it is correct, acoording to this table:

<center>

| Input | Action |
|:--------:|:----------:|
|__[a], [A]__|__Accelerate__|
|__[d], [D]__|__Decelerate__|
|__[r], [R]__|__Reset position__|

</center>

If the command is correct, it simply sends it to the service node, and prints to the user the new velocity.
This is the code used to "talk" with service node.

```cpp
service.request.input_char=inputChar;
        

        service_client.waitForExistence();
        service_client.call(service);

        RT1_Assignment2::Speed_val speed;
        speed.speed = service.response.value;
        pub.publish(speed);
        system("clear");
        std::cout << "New speed: "<< service.response.value<<"\n\n";
```

### Flowchart ###

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/assets/diagram.png)

Video demonstration
-------------------

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/assets/video.gif)

