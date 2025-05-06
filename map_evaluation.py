#!/usr/bin/env python3

"""
Webots SLAM Map Evaluation Tools
Created with assistance from Claude (Anthropic) on May 6, 2025
https://claude.ai

This script was generated to help convert Webots world files to ground truth
occupancy grid maps and evaluate SLAM algorithm performance.

Script function: SLAM Map Evaluation

Usage rights: Free to use, modify, and distribute with attribution.
"""

import numpy as np
import matplotlib.pyplot as plt
from skimage.metrics import structural_similarity as ssim
from skimage.transform import resize
import cv2
import argparse
import yaml
import os

class MapEvaluator:
    def __init__(self):
        """Initialize the map evaluator
        
        Example usage:
        python map_evaluation.py path/to/slam_map.pgm path/to/ground_truth_map.pgm --output comparison.png
        """
        pass
        
    def load_pgm_map(self, pgm_file, yaml_file=None):
        """
        Load a PGM map file and its metadata
        
        Args:
            pgm_file (str): Path to the PGM map file
            yaml_file (str): Path to the YAML metadata file
        
        Returns:
            tuple: (map_data, metadata)
        """
        # Load PGM file
        with open(pgm_file, 'rb') as f:
            # Parse PGM header
            header = b''
            while True:
                line = f.readline()
                header += line
                if line.startswith(b'255'):  # End of header
                    break
            
            # Extract dimensions from header
            dimensions = header.split(b'\n')[1].split()
            width = int(dimensions[0])
            height = int(dimensions[1])
            
            # Read binary data
            data = np.frombuffer(f.read(), dtype=np.uint8).reshape((height, width))
        
        # Normalize map values: 0=occupied, 100=unknown, 255=free -> 1=occupied, 0.5=unknown, 0=free
        normalized_map = np.ones(data.shape, dtype=np.float32)
        normalized_map[data == 255] = 0.0  # Free space
        normalized_map[data == 0] = 1.0    # Occupied
        normalized_map[(data > 0) & (data < 255)] = 0.5  # Unknown/unexplored
        
        # Load YAML metadata if provided
        metadata = None
        if yaml_file and os.path.exists(yaml_file):
            with open(yaml_file, 'r') as f:
                metadata = yaml.safe_load(f)
        
        return normalized_map, metadata
    
    def align_maps(self, slam_map, ground_truth_map):
        """
        Align the SLAM map with the ground truth map to ensure they're comparable
        
        Args:
            slam_map (ndarray): The SLAM-generated map
            ground_truth_map (ndarray): The ground truth map
        
        Returns:
            tuple: (aligned_slam_map, aligned_ground_truth_map)
        """
        # Resize SLAM map to match ground truth dimensions if needed
        if slam_map.shape != ground_truth_map.shape:
            slam_map = resize(slam_map, ground_truth_map.shape, 
                             anti_aliasing=True, preserve_range=True)
        
        return slam_map, ground_truth_map
    
    def binarize_maps(self, slam_map, ground_truth_map, occupied_thresh=0.65, free_thresh=0.35):
        """
        Convert maps to binary (occupied/free) for certain metrics
        
        Args:
            slam_map (ndarray): The SLAM-generated map
            ground_truth_map (ndarray): The ground truth map
            occupied_thresh (float): Threshold above which a cell is considered occupied
            free_thresh (float): Threshold below which a cell is considered free
        
        Returns:
            tuple: (binary_slam_map, binary_ground_truth_map)
        """
        binary_slam = np.zeros_like(slam_map, dtype=np.uint8)
        binary_slam[slam_map >= occupied_thresh] = 1  # Occupied
        
        binary_gt = np.zeros_like(ground_truth_map, dtype=np.uint8)
        binary_gt[ground_truth_map >= occupied_thresh] = 1  # Occupied
        
        return binary_slam, binary_gt
    
    def calculate_metrics(self, slam_map, ground_truth_map):
        """
        Calculate various evaluation metrics between SLAM and ground truth maps
        
        Args:
            slam_map (ndarray): The SLAM-generated map
            ground_truth_map (ndarray): The ground truth map
        
        Returns:
            dict: Dictionary of evaluation metrics
        """
        # Align maps
        aligned_slam, aligned_gt = self.align_maps(slam_map, ground_truth_map)
        
        # Binarize maps for some metrics
        binary_slam, binary_gt = self.binarize_maps(aligned_slam, aligned_gt)
        
        # Calculate metrics
        metrics = {}
        
        # Mean Squared Error (lower is better)
        metrics['mse'] = np.mean((aligned_slam - aligned_gt) ** 2)
        
        # Structural Similarity Index (higher is better)
        metrics['ssim'] = ssim(aligned_slam, aligned_gt, data_range=1.0)
        
        # Binary metrics
        # True Positives, False Positives, etc.
        tp = np.sum((binary_slam == 1) & (binary_gt == 1))
        fp = np.sum((binary_slam == 1) & (binary_gt == 0))
        tn = np.sum((binary_slam == 0) & (binary_gt == 0))
        fn = np.sum((binary_slam == 0) & (binary_gt == 1))
        
        # Accuracy (higher is better)
        metrics['accuracy'] = (tp + tn) / (tp + tn + fp + fn) if (tp + tn + fp + fn) > 0 else 0
        
        # Precision (higher is better)
        metrics['precision'] = tp / (tp + fp) if (tp + fp) > 0 else 0
        
        # Recall / Completion Rate (higher is better)
        metrics['recall'] = tp / (tp + fn) if (tp + fn) > 0 else 0
        
        # F1 Score (higher is better)
        metrics['f1_score'] = 2 * tp / (2 * tp + fp + fn) if (2 * tp + fp + fn) > 0 else 0
        
        # IoU / Jaccard Index (higher is better)
        metrics['iou'] = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else 0
        
        return metrics
    
    def calculate_exploration_coverage(self, slam_map, ground_truth_map, unknown_threshold=0.4):
        """
        Calculate how much of the explorable area has been mapped
        
        Args:
            slam_map (ndarray): The SLAM-generated map
            ground_truth_map (ndarray): The ground truth map
            unknown_threshold (float): Threshold for considering a cell as unknown
        
        Returns:
            float: Exploration coverage percentage
        """
        # Align maps
        aligned_slam, aligned_gt = self.align_maps(slam_map, ground_truth_map)
        
        # Free space in ground truth (explorable area)
        free_gt = (aligned_gt < 0.5)
        
        # Mapped space in SLAM map (not unknown)
        mapped_slam = ~((aligned_slam > unknown_threshold) & (aligned_slam < (1 - unknown_threshold)))
        
        # Calculate exploration coverage
        explorable_area = np.sum(free_gt)
        mapped_area = np.sum(free_gt & mapped_slam)
        
        coverage = mapped_area / explorable_area if explorable_area > 0 else 0
        
        return coverage
    
    def visualize_comparison(self, slam_map, ground_truth_map, slam_name="SLAM Map", gt_name="Ground Truth", output_file=None):
        """
        Visualize the comparison between SLAM and ground truth maps
        
        Args:
            slam_map (ndarray): The SLAM-generated map
            ground_truth_map (ndarray): The ground truth map
            slam_name (str): Name of the SLAM map for labeling
            gt_name (str): Name of the ground truth map for labeling
            output_file (str): Path to save the visualization (None for display only)
        """
        # Align maps
        aligned_slam, aligned_gt = self.align_maps(slam_map, ground_truth_map)
        
        # Calculate difference map
        diff_map = np.abs(aligned_slam - aligned_gt)
        
        # Create binary edge maps to highlight walls
        edges_slam = cv2.Canny((aligned_slam * 255).astype(np.uint8), 50, 150)
        edges_gt = cv2.Canny((aligned_gt * 255).astype(np.uint8), 50, 150)
        
        # Create RGB difference visualization
        # Red: Only in ground truth, Green: Only in SLAM, Yellow: In both
        rgb_diff = np.zeros((*aligned_slam.shape, 3), dtype=np.uint8)
        rgb_diff[..., 0] = (edges_gt > 0) * 255  # Red channel
        rgb_diff[..., 1] = (edges_slam > 0) * 255  # Green channel
        
        # Create visualization
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Ground Truth
        axes[0, 0].imshow(aligned_gt, cmap='gray_r')
        axes[0, 0].set_title(gt_name)
        axes[0, 0].axis('off')
        
        # SLAM Map
        axes[0, 1].imshow(aligned_slam, cmap='gray_r')
        axes[0, 1].set_title(slam_name)
        axes[0, 1].axis('off')
        
        # Difference Map
        im = axes[1, 0].imshow(diff_map, cmap='hot', vmin=0, vmax=1)
        axes[1, 0].set_title('Difference Map')
        axes[1, 0].axis('off')
        plt.colorbar(im, ax=axes[1, 0], fraction=0.046, pad=0.04)
        
        # Edge Comparison
        axes[1, 1].imshow(rgb_diff)
        axes[1, 1].set_title('Wall Alignment (Red: GT, Green: SLAM)')
        axes[1, 1].axis('off')
        
        plt.tight_layout()
        
        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"Saved visualization to {output_file}")
        else:
            plt.show()

def main():
    parser = argparse.ArgumentParser(description='Evaluate SLAM map against ground truth')
    parser.add_argument('slam_map', help='Path to the SLAM-generated PGM map file')
    parser.add_argument('ground_truth_map', help='Path to the ground truth PGM map file')
    parser.add_argument('--slam-yaml', help='Path to the SLAM map YAML metadata file')
    parser.add_argument('--gt-yaml', help='Path to the ground truth map YAML metadata file')
    parser.add_argument('--output', help='Path to save the visualization')
    parser.add_argument('--slam-name', default='SLAM Map', help='Name of the SLAM map for visualization')
    parser.add_argument('--gt-name', default='Ground Truth', help='Name of the ground truth map for visualization')
    
    args = parser.parse_args()
    
    evaluator = MapEvaluator()
    
    # Load maps
    slam_map, slam_metadata = evaluator.load_pgm_map(args.slam_map, args.slam_yaml)
    gt_map, gt_metadata = evaluator.load_pgm_map(args.ground_truth_map, args.gt_yaml)
    
    # Calculate metrics
    metrics = evaluator.calculate_metrics(slam_map, gt_map)
    coverage = evaluator.calculate_exploration_coverage(slam_map, gt_map)
    
    # Print metrics
    print("=== Map Evaluation Results ===")
    print(f"Mean Squared Error: {metrics['mse']:.4f} (lower is better)")
    print(f"Structural Similarity: {metrics['ssim']:.4f} (higher is better)")
    print(f"Accuracy: {metrics['accuracy']:.4f} (higher is better)")
    print(f"Precision: {metrics['precision']:.4f} (higher is better)")
    print(f"Recall / Completion Rate: {metrics['recall']:.4f} (higher is better)")
    print(f"F1 Score: {metrics['f1_score']:.4f} (higher is better)")
    print(f"IoU / Jaccard Index: {metrics['iou']:.4f} (higher is better)")
    print(f"Exploration Coverage: {coverage:.2%}")
    
    # Visualize comparison
    evaluator.visualize_comparison(slam_map, gt_map, args.slam_name, args.gt_name, args.output)

if __name__ == "__main__":
    main()