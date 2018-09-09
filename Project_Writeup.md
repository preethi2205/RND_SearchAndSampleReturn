[//]: # (Image References)

[imageRobot]: ./Pictures/imageRobot.jpg
[imageSimulator]: ./Pictures/imageSimulator.jpg
[imagePerspectiveTransform]: ./Pictures/imagePerspectiveTransform.png
[imageTerrain]: ./Pictures/imageTerrain.png
[imageRock]: ./Pictures/imageRock.png
[imageCoords]: ./Pictures/imageCoords.jpg
[imageDecisionTree]: ./Pictures/imageDecisionTree.jpg
[imageAverageHeading]: ./Pictures/imageAverageHeading.png
[imageMappedArea]: ./Pictures/imageMappedArea.png

## Project: Search and Sample Return
### The why? 
This is the first project in the robotics nanodegree program and is intended to provide an entry into the world of robotics. It is based on the NASA's search and sample return program, and gives an introductiheon to two major building blocks of any robot's software - Perception and Planning
### The what?
The robot involved in this project is four-wheeled and has a camera that can sense it's environment. It's aim is to search the terrain around it to map and navigate it. While navigating the terrain, it is also responsible for collecting rock samples it finds on its way. 
![alt text][imageRobot]
### The how?
The project is split into two phases:
1. Training: In this phase we will develop functions that help the robot process the environment sensed by the on-board camera.
2. Autonomous Navigation and Mapping: In this phase, we will use our functions from phase 1 to understand the environment, and further develop functions that help the robot decide where to navigate and how to navigate there. We develop a utility to map the areas navigated by the robot. The robot also finds rocks in the environment and collects them.

---

### Project set up
Udacity provides a simulator set up that is capable of simulating the environment, terrain and the robot motion. It outputs the camera data used for our algorithms. We first operate this simulator manually using our keyboard and collect some sample images. Udacity also provides scaffolding code to get started with the functions needed in this phase. We will test the functions developed on the output images collected from the previous step. In the second phase, we will develop our algorithm such that the rover can autonomously navigate the simulator terrain. Here's a screenshot of what the simulator looks like:
![alt text][imageSimulator]

### Overview of the training phase
The following functions are developed and applied on the image sensed by the robot. By the end of this phase, the rover will be able to understand where it can navigate, and what places it needs to avoid. 
#### 1. Perspective transform
In this step, we convert image from rover's point of view to the top view. The image below summarized the output of the perspective transform. The left image is the direct output of the rover's camera, and the right image is taken after the perspective transform has been applied.
![alt text][imagePerspectiveTransform]

#### 2. Color threshold
The next step is to distinguish the terrain on which the rover can navigate from the surrounding rocks. The road/ surface is much brighter than surrounding mountains in all the (R,G,B) channels. So, in order to isolate the traversable surface, we apply a color thresholding algorithm on the sensed camera image:
##### traversable surface = Image(R>160, G>160, B>160)
![alt text][imageTerrain]

Rocks can also be collected using similar color thresholding:
##### Rocks = Image(R>110, G>110, B<50)
![alt text][imageRock]

#### 4. Transform to rover-centric coordinates
During te perspective transform, we put the rover at an arbitary position in the image coordinate system. The rover centric coordinate system is located to the rover. The origin of the coordinate system is the rover and rover is headed along 0 degrees. We apply a rotation and a translation to the image coordinate frame to transform it to the rover centric coordinates. Refer to the image below to understand the different coordinate systems:
![alt text][imageCoords]

#### 5. Transform to world coordinate system
Each pixel sensed by the rover's camera, is now in the rover-centric coordinate system. These points need to be converted to the world coordinate system in order to map the world. This involves the following operations:
1. Rotate by rover's world yaw angle
2. Scale to map: Each grid square is 1x1 m^2 in real world. In the perspective transform step, we map each grid to 10x10 pixels. Each pixel corresponds to 0.1m in the rover world. However, 1 pixel corresponds to 1m in the real world. Thus our scaling factor is 1/10. 1RS = 1/10WS.
3. Translate by rover's world x and world y position.

### We are now done with the first step in the project. The output of the training phase is tested on a series of images recorded using the simulator. The images are stitched together as a video, which you can see here. Notice the processed image containing the transformed image and identified navigable terrain.
![Video of processed images from simulator](http://img.youtube.com/vi/7Y12V5P-S_g/0.jpg)](http://www.youtube.com/watch?v=7Y12V5P-S_g"Video of processed images from simulator")

### Overview of the autonomous navigation
Most autonomous navigation problems need three main components to function properly:
1. Perception
2. Decision
3. Control

We will reuse most of the functions we developed in the training phase to develop the perception function. We will describe the functionality of the decision making module in the coming paragraphs. The control module in this project is extremely straightforward. The implications of this simplification and suggestions for future improvements are also disccused towards the end.

#### Perception
We perform the steps listed in the training phase to get the coordinates of the rover in the world frame. Now, we extract some mathematical data from this image, which can be used to navigate the rover. We use the average heading angle of all the pixels sensed by the camera to determine the direction in which the rover has to travel. Similarly, we use the average angle of all the rock pixels to figure out the heading angle in which the rover should travel to collect rocks.

#### Decision
The decision module is the brain of the rover. It tells the rover what action to take, based on the rover's position, velocity, orientation and the sensed surrondings. There are four states that the rover can take:
1. Move forward: This is the default state the rover would be in, as long as it sees some navigable terrain ahead of it. This is the state in which the rover continues to move through the terrain, while mapping it.
2. Move to rock: During the move forward mode, if the rover's camera and perception module found a rock, the rover goes into move to rock state, which sets the rover's steering angle to aveage heading angle of all the rock pixels. This helps the rover move to the rock. Once the rover is close enough to a rock sample, it can send a command to the simulator which will then pick up the rock and collect it.
3. Stop and turn: If the rover's camera cannot find any navigable terrain ahead, the rover is probably close to an obstacle (rock wall). In this case, it cannot move forward. It stops completely and turns around till it can see enough navigable terrain again, at which point it returns to the "move forward" mode.
4. Unstuck: This is a more advanced mode, which is necesarry in case the rover gets stuck such that it is unable to turn at all. It also covers the case where the rover is just going in circles, and is not mapping any new areas. In these cases, we introduce a random steering and a random throttle and a reverse throttle command to get the rover "unstuck".

#### Here's a summary of the rover's decision layer logic:
![alt text][imageDecisionTree]

#### Control
The rover's control module is extremely simplified "proportional" controller. The steering command is proportional to the average heading of the navigable terrain. The throttle and brake commands are best guess constants. Due to this, we notice that the rover oscillates quite a bit while navigating the terrain. A more sophisticated approach would be to use a PID controller. The differential term would help dampen the oscillations and the integral term would help reduce any steady state error.
![alt text][imageAverageHeading]

#### Mapping
While moving forward and collecting samples, the rover records the area sensed by it. The recorded locations are added to a database to form the map. The fidelity of the mapping is measured by the percentage of mapped pixels that lie within the ground truth map area. In order to ensure high fidelity of the mapped area, we only record the points when the rover is moving. When the rover is stopped, it does not get much new information, as it is either mapping the same area again and again, or is turning, which causes distortions in the sensing. Thus, we disregard the map data collected when the rover is stopped. Here's a screenshot of the rover after it has run autonomously for a while:
![alt text][imageMappedArea]

#### Potential problems and further suggestions
With this algorithm implentation, the rover does a decent job of mapping atleast 60% of the area, with a 70% fidelity. It is also able to move to, and pick up atleast 3-4 rocks everytime. However, there are a few potential problems that the rover could run into:
1. If there are two rocks located simultaneously, it gets confused about which rock to move to first.
2. The rover often gets stuck. There is more work to be done to identify navigable terrain. 
3. The rover often travels over already mapped areas/ corridors. It's path can be optimized by weighting already mapped areas lower, and using that cost to steer the rover towards unmapped areas.
4. The entire challenge calls for the rover to collect all rocks and return to its starting point. This is not implemented here, but can be added by keeping track of the starting position and including a global route planning algorithm back to the start.
