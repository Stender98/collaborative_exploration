#!/usr/bin/env python3

"""
Webots SLAM Map Evaluation Tools
Created with assistance from Claude (Anthropic) on May 6, 2025
https://claude.ai

This script was generated to help convert Webots world files to ground truth
occupancy grid maps and evaluate SLAM algorithm performance.

Script function: Webots to Occupancy Grid Converter

Usage rights: Free to use, modify, and distribute with attribution.
"""

import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse

class WebotsMapConverter:
    def __init__(self, world_file, resolution=0.05, padding=0.5):
        """
        Initialize the converter with the Webots world file and map parameters
        
        Args:
            world_file (str): Path to the .wbt file
            resolution (float): Map resolution in meters per pixel
            padding (float): Extra padding around the map borders in meters

        Example usage:
        python webots_to_occupancy_grid.py path/to/your/world.wbt --resolution 0.05 --output ground_truth_map --visualize
        """
        self.world_file = world_file
        self.resolution = resolution
        self.padding = padding
        self.walls = []
        self.arena_size = None
        
    def parse_world_file(self):
        """Parse the Webots world file to extract wall positions and sizes"""
        with open(self.world_file, 'r') as f:
            content = f.read()
        
        # Extract arena size
        arena_match = re.search(r'RectangleArena\s*{[^}]*floorSize\s+([0-9.]+)\s+([0-9.]+)', content, re.DOTALL)
        if arena_match:
            self.arena_size = (float(arena_match.group(1)), float(arena_match.group(2)))
            print(f"Arena size: {self.arena_size[0]}x{self.arena_size[1]} meters")
        
        # Find all wall objects in the file
        wall_pattern = r'Wall\s*{([^}]*)}'
        walls_data = re.findall(wall_pattern, content, re.DOTALL)
        
        for wall_data in walls_data:
            # Extract translation
            trans_match = re.search(r'translation\s+([+-]?[0-9.]+)\s+([+-]?[0-9.]+)', wall_data)
            if not trans_match:
                continue
            x = float(trans_match.group(1))
            y = float(trans_match.group(2))
            
            # Extract rotation if present
            rotation = 0
            rot_match = re.search(r'rotation\s+[0-9.]+\s+[0-9.]+\s+[0-9.]+\s+([+-]?[0-9.]+)', wall_data)
            if rot_match and abs(float(rot_match.group(1))) > 1.5:  # Approximately -pi/2 rotation
                rotation = 90  # 90 degrees rotation (perpendicular)
            
            # Extract size
            size_match = re.search(r'size\s+([0-9.]+)\s+([0-9.]+)', wall_data)
            if not size_match:
                continue
                
            width = float(size_match.group(1))
            height = float(size_match.group(2))
            
            # Swap width and height if rotated 90 degrees
            if rotation == 90:
                width, height = height, width
            
            self.walls.append({
                'x': x - width/2,  # Convert to bottom-left corner
                'y': y - height/2,
                'width': width,
                'height': height
            })
            
        print(f"Parsed {len(self.walls)} walls from the world file")

    def create_occupancy_grid(self):
        """Create a 2D occupancy grid from the parsed wall data"""
        if not self.walls:
            raise ValueError("No walls parsed. Call parse_world_file() first")
        
        if not self.arena_size:
            # Estimate map bounds from walls if arena size not found
            x_min = min([wall['x'] for wall in self.walls])
            x_max = max([wall['x'] + wall['width'] for wall in self.walls])
            y_min = min([wall['y'] for wall in self.walls])
            y_max = max([wall['y'] + wall['height'] for wall in self.walls])
            
            map_width = x_max - x_min + 2 * self.padding
            map_height = y_max - y_min + 2 * self.padding
            origin_x = x_min - self.padding
            origin_y = y_min - self.padding
        else:
            # Use arena size
            map_width = self.arena_size[0]
            map_height = self.arena_size[1]
            origin_x = -map_width / 2
            origin_y = -map_height / 2
        
        # Calculate grid dimensions
        grid_width = int(map_width / self.resolution)
        grid_height = int(map_height / self.resolution)
        
        # Create empty grid (0 = free, 1 = occupied)
        grid = np.zeros((grid_height, grid_width), dtype=np.uint8)
        
        # Add walls to the grid
        for wall in self.walls:
            # Convert wall coordinates to grid indices
            x_start = int((wall['x'] - origin_x) / self.resolution)
            y_start = int((wall['y'] - origin_y) / self.resolution)
            x_end = int((wall['x'] + wall['width'] - origin_x) / self.resolution)
            y_end = int((wall['y'] + wall['height'] - origin_y) / self.resolution)
            
            # Ensure indices are within grid bounds
            x_start = max(0, min(x_start, grid_width - 1))
            y_start = max(0, min(y_start, grid_height - 1))
            x_end = max(0, min(x_end, grid_width - 1))
            y_end = max(0, min(y_end, grid_height - 1))
            
            # Mark wall cells as occupied
            grid[y_start:y_end+1, x_start:x_end+1] = 1
        
        # Flip the grid vertically to match standard map orientation
        grid = np.flipud(grid)
        
        return grid, (origin_x, origin_y), self.resolution

    def visualize_map(self, grid):
        """Visualize the occupancy grid"""
        plt.figure(figsize=(10, 8))
        plt.imshow(grid, cmap='gray_r', origin='lower')
        plt.title('Webots World Occupancy Grid')
        plt.colorbar(label='Occupancy (0=free, 1=occupied)')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        plt.tight_layout()
        plt.savefig('webots_map.png', dpi=300)
        plt.show()
        
    def save_map_to_file(self, grid, filename='webots_map.pgm', yaml_filename='webots_map.yaml'):
        """Save the occupancy grid to a PGM file and metadata to YAML file (ROS map format)"""
        # Normalize grid values to 0-255 range (ROS convention: 0=occupied, 255=free, unknown=205)
        ros_grid = np.ones(grid.shape, dtype=np.uint8) * 255  # Default to free space
        ros_grid[grid == 1] = 0  # Occupied cells
        
        # Save as PGM
        with open(filename, 'wb') as f:
            height, width = ros_grid.shape
            # PGM header
            f.write(f"P5\n{width} {height}\n255\n".encode())
            # Write binary data
            f.write(ros_grid.tobytes())
        
        # Save metadata to YAML
        origin_x, origin_y = -self.arena_size[0]/2 if self.arena_size else 0, -self.arena_size[1]/2 if self.arena_size else 0
        with open(yaml_filename, 'w') as f:
            f.write(f"image: {filename}\n")
            f.write(f"resolution: {self.resolution}\n")
            f.write(f"origin: [{origin_x}, {origin_y}, 0.0]\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")
            f.write("negate: 0\n")
            
        print(f"Saved map to {filename} and {yaml_filename}")

def main():
    parser = argparse.ArgumentParser(description='Convert Webots world file to 2D occupancy grid')
    parser.add_argument('world_file', help='Path to the Webots .wbt file')
    parser.add_argument('--resolution', type=float, default=0.05, help='Map resolution in meters per pixel')
    parser.add_argument('--padding', type=float, default=0.5, help='Extra padding around the map in meters')
    parser.add_argument('--output', default='webots_map', help='Output filename base (without extension)')
    parser.add_argument('--visualize', action='store_true', help='Visualize the generated map')
    
    args = parser.parse_args()
    
    converter = WebotsMapConverter(args.world_file, args.resolution, args.padding)
    converter.parse_world_file()
    grid, origin, resolution = converter.create_occupancy_grid()
    
    if args.visualize:
        converter.visualize_map(grid)
    
    converter.save_map_to_file(grid, f"{args.output}.pgm", f"{args.output}.yaml")
    
    print(f"Generated occupancy grid with dimensions {grid.shape[1]}x{grid.shape[0]} pixels")
    print(f"Map origin: {origin}")
    print(f"Map resolution: {resolution} meters/pixel")

if __name__ == "__main__":
    main()