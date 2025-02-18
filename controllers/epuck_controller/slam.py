import numpy as np
import pygame

class SLAM:
    def __init__(self, map_width=2000, map_height=2000):
        #initialize the map with unknown values (-1)
        self.map_width = map_width 
        self.map_height = map_height 
        self.map = np.full((self.map_width, self.map_height), -1)  #grid with unknown areas
    
        #initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.map_width, self.map_height))
        pygame.display.set_caption("SLAM Visualization")

        self.clock = pygame.time.Clock()  #to control the frame rate

    def update(self, lidar_data):
        #iterate through each lidar reading
        for i, distance in enumerate(lidar_data):
            if distance < float('inf') and distance > 0:  #valid reading
                angle = i * (2 * np.pi / len(lidar_data))
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                
                #mark obstacle at the detected point
                self.update_map(int(x), int(y), obstacle=True)
                
                #mark free space between the robot and the obstacle
                self.mark_free_space(x, y)

            elif distance == float('inf'):  #no obstacle detected
                angle = i * (2 * np.pi / len(lidar_data))
                x = 3.5 * np.cos(angle)  #max range (3.5 meters)
                y = 3.5 * np.sin(angle)
                
                #mark free space up to the maximum sensor range
                self.mark_free_space(x, y)

    def mark_free_space(self, x, y):
        #mark all free space between the robot and the obstacle
        #we can do this by stepping from the robot's position towards (x, y)
        
        steps = int(np.linalg.norm([x, y]))  #number of steps from the robot to the obstacle
        for step in range(steps):
            #calculate intermediate points
            step_x = step * (x / steps)
            step_y = step * (y / steps)
            
            #mark this point as free space
            grid_x = int(step_x + self.map_width / 2)
            grid_y = int(step_y + self.map_height / 2)
            self.update_map(grid_x, grid_y, obstacle=False)


    def update_map(self, grid_x, grid_y, obstacle):
        #check coordinates are within map bounds
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            #mark obstacle (1), free space (0), or unknown (-1)
            if obstacle:
                self.map[grid_x, grid_y] = 1  #obstacle
            else:
                self.map[grid_x, grid_y] = 0  #free space

    def visualize(self):
        #white = obstacle (1), black = free space (0), red = unknown (-1)
        #draw the map
        for x in range(self.map_width):
            for y in range(self.map_height):
                color = (255, 0, 0)  #default is red for unknown (-1)
                if self.map[x, y] == 1:
                    color = (255, 255, 255)  #white for obstacle
                elif self.map[x, y] == 0:
                    color = (0, 0, 0)  #black for free space

                #draw a rectangle for each cell
                pygame.draw.rect(self.screen, color, (x, y, 1, 1))

        pygame.display.flip()  #update the display
        self.clock.tick(30)  #control the frame rate (30 FPS)