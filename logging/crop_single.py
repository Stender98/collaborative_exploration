import numpy as np
import cv2
import argparse

parser = argparse.ArgumentParser(description='Crop a map image and update the YAML file.')
parser.add_argument('image', type=str, help='Path to the map image file')
parser.add_argument('yaml', type=str, help='Path to the YAML file')

args = parser.parse_args()

img = cv2.imread(args.image, cv2.IMREAD_UNCHANGED)
if img is None:
    raise FileNotFoundError(f"Image file {args.image} not found.")

mask = img != 205

coords = np.argwhere(mask)
y0, x0 = coords.min(axis=0)
y1, x1 = coords.max(axis=0) + 1

cropped = img[y0:y1, x0:x1]

base_name = args.image.split('.')[0]
cropped_name = f"{base_name}_cropped.pgm"
cv2.imwrite(cropped_name, cropped)

with open(args.yaml, 'r') as f:
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


