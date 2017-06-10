import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh_terrain(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select_terrain = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select_terrain[above_thresh] = 1
    # Return the binary image
    return color_select_terrain

def color_thresh_rocks(img):
    #yellow_lower_thresh = np.array([5,100,100], dtype = "uint8") # remember it's BGR for CV2 was [20, 100, 100]
    #yellow_upper_thresh = np.array([30,255,182], dtype = "uint8") # was [30,255, 182]
    #img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV,3)
    # Create an array of zeros same xy size as img, but single channel
    # color_select_rocks = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met


    # Index the array of zeros with the boolean array and set to 1
    #color_select_rocks[above_thresh] = 1
    # return the binary image

    yellow_lower_thresh = (80,80,0)
    yellow_upper_thresh = (255,255,50)
    color_select_rocks = np.zeros_like(img[:,:,0])

    within_thresh = (img[:,:,0] <= yellow_upper_thresh[0]) \
                  & (img[:,:,0] > yellow_lower_thresh[0]) \
                  & (img[:,:,1] <= yellow_upper_thresh[1]) \
                  & (img[:,:,1] > yellow_lower_thresh[1]) \
                  & (img[:,:,2] < yellow_upper_thresh[2]) \
                  & (img[:,:,2] >= yellow_lower_thresh[2]) # changed it to <= for some of these

    color_select_rocks[within_thresh] = 1


    return color_select_rocks      ### mask_rock

def color_thresh_obstacles(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select_obstacles = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select_obstacles[above_thresh] = 1
    # Return the binary image
    return color_select_obstacles

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    #
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = yaw * ((np.pi)/180.)
    xpix_rotated = xpix * np.cos(yaw_rad) - (ypix * np.sin(yaw_rad))
    ypix_rotated = xpix * np.sin(yaw_rad) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    #
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot/scale)) # /10 for 0.1m in rover pixels to 1m for map pixels
    ypix_translated = np.int_(ypos + (ypix_rot/scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

 
    dst_size = 5
    bottom_offset = 6 # account for the fact that the bottom of the image is not the position of the rover
                      # but a bit in front of it this is just a rough guess, feel free to change it!
    scale = 2*dst_size
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
        # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

        # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed_terrain = color_thresh_terrain(warped)
    threshed_obstacles = color_thresh_obstacles(warped)
    threshed_rocks = color_thresh_rocks(warped)

        # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = threshed_obstacles * 255
    Rover.vision_image[:,:,1] = threshed_rocks  * 255 ### changed this back to *255 as it's true etc.. multiply by 255
    Rover.vision_image[:,:,2] = threshed_terrain * 255


        # 5) Convert map image pixel values to rover-centric coords
    xpix_terrain, ypix_terrain = rover_coords(threshed_terrain)
    xpix_obstacles, ypix_obstacles = rover_coords(threshed_obstacles)
    xpix_rocks, ypix_rocks = rover_coords(threshed_rocks)

        # 6) Convert rover-centric pixel values to world coordinates
    #world_size = Rover.worldmap.shape[0]
    #(xpos, ypos) = Rover.pos
    #yaw = Rover.yaw

    navigable_x_world, navigable_y_world = pix_to_world(xpix_terrain, ypix_terrain, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    obstacles_x_world, obstacles_y_world = pix_to_world(xpix_obstacles, ypix_obstacles, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    rocks_x_world, rocks_y_world = pix_to_world(xpix_rocks, ypix_rocks, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)

        # 7) Update Rover worldmap (to be displayed on right side of screen)

    if ((min(abs((360 - Rover.roll)%360), abs((Rover.roll - 360)%360))) < 1) and ((min(abs((360 - Rover.roll)%360), abs((Rover.roll - 360)%360))) < 1 ): # attempt to fix fidelity within Roll and pitch +- 1 degree
        Rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1
        Rover.worldmap[rocks_y_world, rocks_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

        # 8) Convert rover-centric pixel positions to polar coordinates
        # Update Rover pixel distances and angles


    rover_dists, rover_angles = to_polar_coords(xpix_terrain, ypix_terrain)
    # convert rover-centric pixel positions of rocks to polar coordinates
    # update rock and obstacle pixel distances and angles
    rock_dists, rock_angles = to_polar_coords(xpix_rocks, ypix_rocks)
    obstacle_dists, obstacle_angles = to_polar_coords(xpix_obstacles,ypix_obstacles)

    # save these values to the Rover
    Rover.rock_angles = rock_angles
    Rover.rock_dists = rock_dists
    Rover.nav_dists = rover_dists
    Rover.nav_angles = rover_angles
    Rover.obstacle_angles = obstacle_angles
    Rover.obstacle_dists = obstacle_dists

    return Rover

