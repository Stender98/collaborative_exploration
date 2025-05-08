import numpy as np
import cv2
import argparse
import os

parser = argparse.ArgumentParser(description='Crop a map image and update the YAML file.')
parser.add_argument('mode', choices=['c', 'd'], help='Mode: c for centralized, d for decentralized')
parser.add_argument('num_robots', type=int, help='Number of robots')
parser.add_argument('run_count', type=int, help='Current run count')
args = parser.parse_args()

current_directory = os.path.dirname(os.path.abspath(__file__))
current_directory += "/../"  # Adjust path to the parent directory

if args.mode == 'c':
    mode = 'centralised'
elif args.mode == 'd':
    mode = 'decentralised'

args = parser.parse_args()

slam_map_file = os.path.join(current_directory, "logs", mode, str(args.num_robots), str(args.run_count), "slam_map.pgm")
slam_yaml_file = os.path.join(current_directory, "logs", mode, str(args.num_robots), str(args.run_count), "slam_map.yaml")
  

img = cv2.imread(slam_map_file, cv2.IMREAD_UNCHANGED)
if img is None:
    raise FileNotFoundError(f"Image file {args.image} not found.")

mask = img != 205

coords = np.argwhere(mask)
y0, x0 = coords.min(axis=0)
y1, x1 = coords.max(axis=0) + 1

cropped = img[y0:y1, x0:x1]

base_name = slam_map_file.split('.')[0]
cropped_name = f"{base_name}_cropped.pgm"
cv2.imwrite(cropped_name, cropped)

with open(slam_yaml_file, 'r') as f:
    lines = f.readlines()
for i, line in enumerate(lines):
    if 'image:' in line:
        lines[i] = f"image: {cropped_name}\n"
    elif 'resolution:' in line:
        lines[i] = f"resolution: {0.05}\n"  

base_name = cropped_name.split('.')[0]
new_yaml_name = f"{base_name}.yaml"
with open(new_yaml_name, 'w') as f:
    f.writelines(lines)


