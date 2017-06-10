## Search and Sample Return Project

### The following is a description of the Rover Sample Return Project
By Robert Magee for the Udacity Robotics Nano Degree. It uses both code and a simulator provided as a platform to test and experiment with solutions to programming a Rover to navigate independently using - Perception - Decision - Action steps.

#### Simulator settings:
- Resolution: 1024 x 768, Quality: 'Good'

---

### Writeup / README
**Project Outline and Goals**

The Training / Calibration steps of this project are the following:

* The Simulator, in my case [MacOS Sumulator Build](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip).
* Set up the environment for python on the mac using [Anaconda Environments](https://github.com/udacity/RoboND-Python-StarterKit/blob/master/doc/configure_via_anaconda.md).
* Set up [Jupyter Notebooks](http://jupyter.org/) and run through the Notebook provided.
* Set up Gitbub using my new account [@robearmagee](https://github.com/robearmagee)
* Add functions to the notebook to detect obstacles and samples (golden rocks).
* Fill in the `process_image()` function to perform - perspective transform, color threshold, coordinate transformations, etc.
* Use `moviepy` to process the images with the `process_image()` function.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

The Autonomous Navigation / Mapping steps of this project include the following:

* Fill in the `perception_step()` and `decision_step()` functions until the rover can simultaneously navigate and map the environment, run from the terminal using:
```sh
python drive_rover.py /Volumes/folder_loc
```



[//]: # (Image References)

[image1]: ./test_images/rover_w_rocks.jpg "Rover with Rocks"
[image2]: ./test_images/source_destination_img.jpg "Perspective Transform"
[image3]: ./test_images/rock_truth_map.jpg "rock truth map"
[image4]: ./test_images/rock_arrow.jpg "Direction Arrow"
[image5]: ./test_images/Rover_Mapped_Issue.jpg "Warp Example"
[image6]: ./writeup_info/Rover_Mapping.jpg "Rover Mapping"
[image7]: ./writeup_info/Weighted_rock_nav.jpg "Steering towards rock"
[image8]: ./writeup_info/dist_to_wall.jpg "Rover Video"
[image9]: ./writeup_info/RoverVideo.jpg "Steering towards rock"
[video1]: ./output/RoverSim5.mp4 "Video"


### Outline
The project has been truly eye-opening and really interesting. While I managed to achieve several runs where the rover collected all the rocks in the space, there is certainly a lot more work that could be done to improve it overall. Through the information learnt in the lessons I'm happy to say I managed to achieve a reasonably good level of autonomous perception, decision making and mapping / driving.

![alt text][image1]

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points


The code for these steps is contained in the Jupyter notebook located in `"../code/Rover_Project_Test_Notebookipynb.ipynb"` (and later migrated into `perception.py` file) also located in `../code/` folder.  

---
### Jupyter Notebook Analysis.


 I edited the `Rover_Project_Test_Notebookipynb` to include the functions that were developed in class. The following is a description of the Jupyter Notebook findings followed by the results from the Autonomous Rover simulation.  

### 1. `Process_Image()` Pipeline

#### 1.1. Perspective transform.
The first stage of the Process_image() function uses open `cv2` to distort the camera image into a top-down 'map'.

```python
def perspect_transform(img, src, dst):     
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    return warped
    ```

The source and destination calibration points were determined in class:
```python
dst_size = 5
bottom_offset = 6
source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
```
Using these coordinates results in the following warped image:

![alt text][image2]


#### 1.2. Color Thresholding.
A challenging aspect of Open CV is it defaults to `BGR` instead of `RGB` color space when reading/writing images.

I created three functions to threshold the source image to generate three output images.
1. Navigable terrain - using default RGB values that were determined in class
2. Obstacles - The opposite to the navigable space. This was a sticking point throughout he project.
3. Rocks - Upper and lower thresholds were required to capture only the yellow color.

| Threshold         | Function   |
|:-----------------:|:-------------:|
| Navigable Terrain  | ```def color_thresh_terrain(img, rgb_thresh=(160, 160, 160)):```  |
| Obstacles         | ```def color_thresh_obstacles(img, rgb_thresh=(160, 160, 160)):```      |
| Rock Samples      | ```def color_thresh_rocks(img):```     |

Python code to find pixels in Rover image that lie within upper and lower threshold limits in the 3 channels:
```python
def color_thresh_rocks(img):
    yellow_lower_thresh = (80,80,0)
    yellow_upper_thresh = (255,255,60)
    color_select_rocks = np.zeros_like(img[:,:,0])

    within_thresh = (img[:,:,0] <= yellow_upper_thresh[0]) \
                  & (img[:,:,0] >= yellow_lower_thresh[0]) \
                  & (img[:,:,1] <= yellow_upper_thresh[1]) \
                  & (img[:,:,1] >= yellow_lower_thresh[1]) \
                  & (img[:,:,2] <= yellow_upper_thresh[2]) \
                  & (img[:,:,2] >= yellow_lower_thresh[2])
    color_select_rocks[within_thresh] = 1
    return color_select_rocks
```
This is not the most elegant code ever produced however I needed to move along quickly and as the navigable terrain and obstacle thresholding functions worked, I simply used a similar `&` logic for the values for RGB being between 80 < value < 255 for example. It returns a 'binary truth image' of 1's and 0's saying whether or not the pixel value in that location meets these rules.

![alt text][image3]

Although barely visible, this image shows the pixels where the above conditions are met. The rock from the images above is resting on the far left of the image. There was a lot of trial and error involved. Later this map is used in the `perception.py` and `decision.py` to flag when a rock is in the Rover's field of vision.

##### Further work:
- I would like to further refine the input and thus results from the obstacles color thresholding image in particular as will become evident, this is a major concern later in the project.


#### 1.3. Coordinate transformations.

The pix_to_world() function takes the pixels output from the above and applies rotation and translation (and clipping)
```python
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
```
The two functions `rotate_pix()` and `translate_pix` take the pixel locations and the yaw of the Rover to provide locations on the world stage.

```python
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = yaw * ((np.pi)/180.)
    xpix_rotated = xpix * np.cos(yaw_rad) - (ypix * np.sin(yaw_rad))
    ypix_rotated = xpix * np.sin(yaw_rad) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated
```
```python
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot/scale)) # /10 for 0.1m in rover pixels to 1m for map pixels
    ypix_translated = np.int_(ypos + (ypix_rot/scale))
    # Return the result  
    return xpix_translated, ypix_translated
```
![alt text][image4]

The `mnumpy.mean()` of the angles pointing towards the rock or navigable space can be calculated which is used for navigation. In this case the arrow superimposed shows the average direction from the Rover's point of view.
In this case it is 15.8 deg.


The following in a step taken from the `process_image(img)` function to perform the `pix_to_world()` conversion.
As can be seen, it was necessary to use `data.xpos[data.count-1]` for the actual x & y position of the rover as this data was taken from the `robot_log.csv` file which recorded the data about the Rover during a test drive on the simulator.

| Path                                       | SteerAngle | Throttle | Brake | Speed | X_Position  |  Y_Position   | Pitch   | Yaw  | Roll |
|:------------------------------------------:|:----------:|:--------:|:-----:|:-----:|:-----------:|:--------:|:--------:|:--------:|
| ../IMG/robocam_2017_06_01_22_59_23_188.jpg | 0          |0        |1       |0      |100.1806     |86.66032  | 0.00023496|64.48|-1.12E-06
| ../IMG/robocam_2017_06_01_22_59_23_250.jpg | 0          |0        |0       |0      |100.1806     |86.66032  |0.00023496| 64.48|-7.97E-07



```python
navigable_x_world, navigable_y_world = pix_to_world(xpix_terrain, ypix_terrain, data.xpos[data.count-1], data.ypos[data.count-1], data.yaw[data.count-1], data.worldmap.shape[0], scale)
obstacles_x_world, obstacles_y_world = pix_to_world(xpix_obstacles, ypix_obstacles, data.xpos[data.count-1], data.ypos[data.count-1], data.yaw[data.count-1], data.worldmap.shape[0], scale)
rocks_x_world, rocks_y_world = pix_to_world(xpix_rocks, ypix_rocks, data.xpos[data.count-1], data.ypos[data.count-1], data.yaw[data.count-1], data.worldmap.shape[0], scale)
```

#### 1.4. Decision Testing.

The `Rover_Project_Test_Notebook.ipynb` became instrumental in testing several aspects of the decision making process of the rover while iterating / optimising the design of the `decision.py` file. This is further explored below.

---
# 2. Rover Autonomous Navigation and Mapping
In order to run the robot autonomously in the Unity simulator the `drive_rover.py` file needs to be called beforehand, as shown above. This contains the Rover class which calls on the `perception.py`, `decision.py` and `supporting_functions.py` files.

#### 2.1. The `perception.py` File.

The transition from the Jupyter notebook across to the `perception.py` file took a little longer than I had hoped. Several small issues compounded together. It was a huge learning curve that involved understanding how image files worked.

The first obstacle was learning how to update the `Rover.vision` image, the final result was:
```python
Rover.vision_image[:,:,0] = threshed_obstacles * 255
Rover.vision_image[:,:,1] = threshed_rocks  * 255
Rover.vision_image[:,:,2] = threshed_terrain * 255
```
The `* 255` took a while to figure out, as the binary image created

![alt text][image5]

As can be ween here there were several issues along the way involving the generation of both the vision images on the left hand side and the world map on the right hand side. This image shows the problems with using the wrong multipliers. The map should progressively add the values for navigable space and obstacles. Instead there is no blue and too much red (obstacles).

```python
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles
# convert rover-centric pixel positions of rocks to polar coordinates
# update rock pixel distances and angles
rock_dists, rock_angles = to_polar_coords(xpix_rocks, ypix_rocks)
```
The `to_polar_coords()` function gives a rover-centric distance and angle to the pixels of rocks/obstacles etc. It was used determine the angle to steer towards the rocks when they were discovered. After much frustration it was later used to help the Rover become a wall-hugger by taking only select angles and distances of only these pixels.



#### 2.2. Fidelity
Locations of the three threshed elements were added to each channel of the `Rover.worldmap`. In a bid to improve fidelity (which was originally terrible), it was recommended in the slack forums to only update the `worldmap` when the rover is level. These values could most likely be optimised however it definitely improved the fidelity of the map.
```python
if ((min(abs((360 - Rover.roll)%360), abs((Rover.roll - 360)%360))) < 1) and ((min(abs((360 - Rover.roll)%360), abs((Rover.roll - 360)%360))) < 1 ): # attempt to fix fidelity within Roll and pitch +- 1 degree
        Rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1
        Rover.worldmap[rocks_y_world, rocks_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
```


#### Improvements:
- Ideally I would have liked to only include the pixels within a certain distance close to the rover, however after the above improvement fidelity seemed to hover around the 70% mark and there were other issues that needed fixing beforehand.
- The ability to set repulsive forces away from the objects, in particular the rocks with overhangs. The rover still gets stuck underneath rocks with overhangs. This is probably a feature of the simulator to make it more challenging, yet it tends to 'kick' the rover away if you get too close (often not even touching) which reduces the fidelity in half and means you have to restart.

#### 2.3. The `decision.py` File.


The skeleton `decision.py` code gave us a good place to start, however as you practice using it you recognise more and more issues.

At first I started making changes without a plan, which wasn't the best idea. I went away and thought about it. There were a series of steps that I needed to go through before Rover could achieve the goals.
They were:
- (a). Safe Navigation
- (b). Recognise Rocks
- (c). Move Towards Rocks
- (d). Collect Rocks
- (e). Wall Hugging
- (f). Become Unstuck!

![alt text][image6]

This tree diagram was created while figuring out how it all comes together. It ended up being far from perfect (eg. I realised braking at all needed to be minimised) however it helped solidify in my mind the fact a 'rock' `Rover.mode` was required and when this decision would take place.

#### (a). Safe Navigation

If Rover constantly crashes, gets stuck or looses fidelity too quickly then the chances of successfully picking up rocks goes out the window.
```python
if (len(Rover.obstacle_angles) > 300) and Rover.vel <0.05:
    Rover.mode = 'stop'
```

A lot of the iterations involved first understanding and then tinkering with figuring out how to get the Rover to enter 'stop' mode when it has become stuck and not simply finished picking up a rock or is going through a slow patch or has just spotted a rock.

#### (b). Recognise Rocks

The recognition of rocks was a big challenge for the Rover because it may spot a quick flash of gold as it turns and it could go out of view.

```python
if Rover.rock_angles is not None:
    if len(Rover.rock_angles) > 0:
        Rover.mode = 'rock'
```

If this happened the Rover was stuck in `'rock'` mode. This meant a few catch statements were required such as this

```python
elif Rover.rock_angles is None:
  if Rover.vel == 0:
    Rover.send_pickup = False
    Rover.mode = 'stop'
    Rover.max_vel = 1
```

#### (c). Move Towards Rocks
One of the biggest sticking points (pun intended) for the project for me was spotting a rock and successfully heading towards it without becoming stuck.
The rocks are always located in the rough edges, as shown below, if you simply steer the Rover DIRECTLY towards the rock as you would with the navigable space then you're guaranteed to crash.
I experimented with weighting the navigable path `mean_nav_angle` and the `mean_rock_angle`.
The result ended up being very reliable, to the point where after implementing this, the chances of picking up a rock without getting stuck improved to over 95%.

```python
if Rover.rock_angles is not None:
  if (len(Rover.rock_angles) > 0) and (len(Rover.nav_angles) > 0):
    mean_rock_angle = np.mean(Rover.rock_angles * 180/np.pi) #convert to angle in deg
    mean_rock_dist = np.mean(Rover.rock_dists)
    mean_nav_angle = np.mean(Rover.nav_angles * 180/np.pi)
    # weighted average towards rock over the path ahead
    weighted_rock_nav_angle = np.average((mean_rock_angle,mean_nav_angle),weights=(0.7,0.3))
    Rover.steer = np.clip(weighted_rock_nav_angle , -15,15)
```
With the noise that is part of the simulation this was not perfect and the rover sometimes steers in a slight zig-zagging path but it seems to work alright.

![alt text][image7]
#### (d). Collect Rocks

There were several requirements that needed to be met before the Rover could pick up a rock. It needed to be `near_sample`, wasn't already `picking_up` a rock and not moving at all, with the brake on. This took a while to figure out.

```python

elif Rover.near_sample == 1:
  Rover.throttle = 0
  Rover.brake = Rover.brake_set
  if Rover.picking_up == 0:
    if Rover.vel == 0:
      Rover.throttle = 0
      Rover.brake = 1
      Rover.send_pickup = True
      Rover.brake = 0
```


#### (e). Wall Hugging
When Rover ended up happily roaming around it became important to implement something (wall-hugging or another method) to avoid going around in circles or leaving it up to chance that it visits every part of the map.
Because of time and complexity I chose hugging the left hand wall.
Using:

I ended up selecting only the `obstacle_angles` on the left hand front segment of the Rover image.
The maximum angle chosen as 50 degrees from zero (straight ahead) and minimum, 16 degrees.

![alt text][image8]

```python

select_angles_within_range = []
increment_steering_L = 17
increment_steering_R = 18
for i in range(len(Rover.obstacle_angles)):
  if Rover.obstacle_angles[i] < (50 * (np.pi/180)) and Rover.obstacle_angles[i] > (16*(np.pi/180)):
    select_angles_within_range.append(Rover.obstacle_dists[i])
dist_to_wall = np.mean(select_angles_within_range)

```

Then taking the average of the `Rover.nav_angles` mean and the direction it needs to turn to maintain contact with the wall.


```python

if dist_to_wall >= 132:
# If too far away, steer towards wall
Rover.steer = np.clip(((np.mean(Rover.nav_angles*180/np.pi)) + increment_steering_L * 1.)/2. ,-15,15)

elif dist_to_wall < 132:
#If too close, steer away from the wall
Rover.steer = np.clip(((np.mean(Rover.nav_angles*180/np.pi)) + increment_steering_R * -1.)/2.,-15,15)
```

Which simply steers the Rover towards or away from the wall depending on the `distance_to_wall`. It certainly isn't perfect however it stopped the Rover going around and around in circles and essentially guaranteed it would visit the whole map, like a maze.

#### Further work:
- The Rover zig-zags left and right, if the wall hugging method was actually the best, I would implement a simple PID controller into the steering to reduce this action.

#### (f). Become Unstuck!

This could fall into further work as I never managed to avoid the rocks. If you travel towards small obstacles and the `Rover.nav_angles` are equal on either side, Rover drives right into them.
There are several methods of going this and I experimented with a rudimentary path planning matrix yet this started chewing up time too fast.

##### Further work:
- Time setting for when stuck.. allows more precise turning away from trouble. Also more powerful throttle setting to be applied.
- While 'seeing' rock becoming stuck is a problem as the Rover needs to go very slow to avoid overshooting, so some checks such as `Rover.vel == 0` is not useful.
- Breaking out into functions. Being self taught my coding is still improving and breaking out some features would have cleaned up my `decision.py` code.
- Instead of wall hugging, rover map stores where it's been.







### Conclusion
I managed to get Rover to navigate and map the environment successfully collecting as many rocks as possible, yet not every time. The main pain point were the obstacles, dark rocks with overhangs in particular!
It was a great project for learning the image manipulation perception pipe-flow then Decision and Action process.
This could be a never ending project, which I fell prey to to some extent.

---

### Autonomous Robot (video)

![alt text][image9]

Here's a [link to my video result](./output/RoverSim5.mp4) in `./output/RoverSim5.mp4` location




---

##### Discussion:

- I would like to properly use .git as I never used it properly before and it seemed too difficult to learn that and do this project in a week. I realised too late the importance / power of it and creating branches. This was a painful lesson.
- My code is pretty messy, as I've been self taught it's a high priority of mine to improve this aspect.
- While there are several things mentioned above I would have loved to implement, I should also be happy with managing to juggle this project with its steep learning curve, my normal work and spending some of the time in hospital, thankfully all good now.


---
