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
        self.arena_wall_thickness = None
        self.arena_wall_height = None
        
    def parse_world_file(self):
        """Parse the Webots world file to extract wall positions and sizes"""
        with open(self.world_file, 'r') as f:
            content = f.read()
        
        # Extract arena parameters
        arena_match = re.search(r'RectangleArena\s*{([^}]*)}', content, re.DOTALL)
        if arena_match:
            arena_content = arena_match.group(1)
            
            # Extract arena size
            size_match = re.search(r'floorSize\s+([0-9.]+)\s+([0-9.]+)', arena_content)
            if size_match:
                self.arena_size = (float(size_match.group(1)), float(size_match.group(2)))
                print(f"Arena size: {self.arena_size[0]}x{self.arena_size[1]} meters")
            
            # Extract wall thickness
            thickness_match = re.search(r'wallThickness\s+([0-9.]+)', arena_content)
            if thickness_match:
                self.arena_wall_thickness = float(thickness_match.group(1))
                print(f"Arena wall thickness: {self.arena_wall_thickness} meters")
            else:
                # Default wall thickness in Webots
                self.arena_wall_thickness = 0.01
                print(f"Using default arena wall thickness: {self.arena_wall_thickness} meters")
            
            # Extract wall height (optional)
            height_match = re.search(r'wallHeight\s+([0-9.]+)', arena_content)
            if height_match:
                self.arena_wall_height = float(height_match.group(1))
                print(f"Arena wall height: {self.arena_wall_height} meters")
        
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
            rot_match = re.search(r'rotation\s+([+-]?[0-9.]+)\s+([+-]?[0-9.]+)\s+([+-]?[0-9.]+)\s+([+-]?[0-9.]+)', wall_data)
            if rot_match:
                # Check for approximate -pi/2 rotation (perpendicular)
                if abs(float(rot_match.group(4))) > 1.5:
                    rotation = 90
            
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
    
    def add_rectangle_arena_walls(self):
        """Add RectangleArena walls to the wall list"""
        if not self.arena_size:
            print("Warning: No RectangleArena found, not adding arena walls")
            return
        
        width, height = self.arena_size
        thickness = self.arena_wall_thickness
        
        # Calculate half-sizes for placement
        half_width = width / 2
        half_height = height / 2
        
        # Add four walls around the arena
        # Wall positions are offset by half thickness to place walls properly
        
        # Bottom wall (along x-axis, at minimum y)
        self.walls.append({
            'x': -half_width - thickness,  # Extend slightly beyond corners
            'y': -half_height - thickness,
            'width': width + 2 * thickness,
            'height': thickness
        })
        
        # Top wall (along x-axis, at maximum y)
        self.walls.append({
            'x': -half_width - thickness,
            'y': half_height,
            'width': width + 2 * thickness,
            'height': thickness
        })
        
        # Left wall (along y-axis, at minimum x)
        self.walls.append({
            'x': -half_width - thickness,
            'y': -half_height,
            'width': thickness,
            'height': height
        })
        
        # Right wall (along y-axis, at maximum x)
        self.walls.append({
            'x': half_width,
            'y': -half_height,
            'width': thickness,
            'height': height
        })
        
        print(f"Added 4 RectangleArena walls with thickness {thickness} meters")

    def create_occupancy_grid(self):
        """Create a 2D occupancy grid from the parsed wall data"""
        if not self.walls:
            raise ValueError("No walls parsed. Call parse_world_file() first")
        
        # Add RectangleArena walls if arena size is available
        if self.arena_size:
            self.add_rectangle_arena_walls()
        
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
        
        # Create a map metadata dictionary
        metadata = {
            'resolution': self.resolution,
            'origin': [origin_x, origin_y, 0.0],
            'dimensions': [grid_width, grid_height],
            'arena_size': self.arena_size,
            'padding': self.padding
        }
        
        return grid, metadata

    def visualize_map(self, grid, metadata=None):
        """Visualize the occupancy grid with wall outlines"""
        plt.figure(figsize=(12, 10))
        
        # Plot the occupancy grid
        plt.imshow(grid, cmap='gray_r', origin='lower')
        
        # Add arena outline if available
        if self.arena_size and metadata:
            origin_x, origin_y = metadata['origin'][0:2]
            res = metadata['resolution']
            arena_width, arena_height = self.arena_size
            
            # Convert arena bounds to pixel coordinates
            arena_x = int((-arena_width/2 - origin_x) / res)
            arena_y = int((-arena_height/2 - origin_y) / res)
            arena_width_px = int(arena_width / res)
            arena_height_px = int(arena_height / res)
            
            # Draw arena outline (flipped for visualization)
            rect = Rectangle((arena_x, grid.shape[0] - arena_y - arena_height_px), 
                             arena_width_px, arena_height_px,
                             linewidth=2, edgecolor='blue', facecolor='none')
            plt.gca().add_patch(rect)
        
        plt.title('Webots World Occupancy Grid')
        plt.colorbar(label='Occupancy (0=free, 1=occupied)')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        plt.tight_layout()
        plt.savefig('webots_map_visualization.png', dpi=300)
        plt.show()
        
    def save_map_to_file(self, grid, metadata, filename='webots_map.pgm', yaml_filename='webots_map.yaml'):
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
        origin_x, origin_y = metadata['origin'][0:2]
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
    parser.add_argument('--no-arena-walls', action='store_true', help='Exclude RectangleArena walls from the map')
    
    args = parser.parse_args()
    
    converter = WebotsMapConverter(args.world_file, args.resolution, args.padding)
    converter.parse_world_file()
    
    if args.no_arena_walls:
        print("Excluding RectangleArena walls from the map")
        converter.arena_size = None
        
    grid, metadata = converter.create_occupancy_grid()
    
    if args.visualize:
        converter.visualize_map(grid, metadata)
    
    converter.save_map_to_file(grid, metadata, f"{args.output}.pgm", f"{args.output}.yaml")
    
    print(f"Generated occupancy grid with dimensions {grid.shape[1]}x{grid.shape[0]} pixels")
    print(f"Map origin: {metadata['origin'][0:2]}")
    print(f"Map resolution: {metadata['resolution']} meters/pixel")

if __name__ == "__main__":
    main()