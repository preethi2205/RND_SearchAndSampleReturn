import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(165, 165, 163)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def find_rocks(img, rock_level=(110,110,50)):
    above_thresh = ((img[:,:,0] > rock_level[0])\
                  & (img[:,:,1] > rock_level[1])\
                  & (img[:,:,2] < rock_level[2]))
    color_select = np.zeros_like(img[:,:,0]);
    color_select[above_thresh] = 1;
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
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

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
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
    img = Rover.img;
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    bottom_offset = 6
    dst_size = 5 
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped);
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image 
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = np.absolute(np.float32(threshed)-1)*255;
    Rover.vision_image[:,:,2] = threshed*255;
    Rover.vision_image[:,:,1] = find_rocks(warped)*255;
    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)
    xobs, yobs = rover_coords(np.absolute(np.float32(threshed)-1))
    xrock, yrock = rover_coords(find_rocks(warped));
    # 6) Convert rover-centric pixel values to world coords
    xpos = Rover.pos[0];
    ypos = Rover.pos[1];
    yaw = Rover.yaw;
    world_size = 200;
    scale = 10;
    x_pix_world, y_pix_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    x_obs_world, y_obs_world = pix_to_world(xobs, yobs, xpos, ypos, yaw, world_size, scale)
    x_rock_world, y_rock_world = pix_to_world(xrock, yrock, xpos, ypos, yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1ck
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if Rover.vel != 0 and np.abs(Rover.pitch)<1 and np.abs(Rover.roll)<1:
        Rover.worldmap[y_obs_world, x_obs_world, 0] += 1
        Rover.worldmap[y_pix_world, x_pix_world, 2] += 1
    
    Rover.worldmap[y_rock_world, x_rock_world, 1] += 1
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    dist, angles = to_polar_coords(xpix, ypix)
    mean_dir = np.mean(angles)
    #print("Length of x_pix_world, y_pix_world: ", x_pix.size, y_pix.size);
    idx = np.where(Rover.worldmap[y_pix_world, x_pix_world, 2]>1);
    
    # Update Rover pixel distances and angles
    Rover.nav_dists = dist
    Rover.nav_angles = angles
    
    canny_img = cv2.Canny(threshed*255, 75, 150); 
    xpix_edge, ypix_edge = rover_coords(canny_img) 
    dist_edge, angles_edge = to_polar_coords(xpix_edge, ypix_edge)
                        
    Rock_Pix = Rover.worldmap[:,:,1];
    xpix_rock,ypix_rock = Rock_Pix.nonzero();
    dist_to_rock = 10000;
    prev_dist_to_rock = 10000;
    xpix_closest_rock = 0;
    ypix_closest_rock = 0;
    for i in range(0,len(xpix_rock)):
        dist_to_rock = np.sqrt((Rover.pos[0] - xpix_rock[i])**2 + (Rover.pos[1] - ypix_rock[i])**2);
        if dist_to_rock<prev_dist_to_rock:
            xpix_closest_rock = xpix_rock[i];
            ypix_closest_rock = ypix_rock[i];
        prev_dist_to_rock = dist_to_rock;
    
    dist_to_rock, angles_to_rock = to_polar_coords(xrock, yrock);
    #distrock, angles_to_rock = to_polar_coords(xpix_closest_rock-Rover.pos[0], ypix_closest_rock-Rover.pos[1]);
    Rover.rock_angles = angles_to_rock;
    Rover.rock_dist = dist_to_rock;
    
    Rover.Last_Known_Rock_x = xpix_closest_rock;
    Rover.Last_Known_Rock_y = ypix_closest_rock;
    return Rover