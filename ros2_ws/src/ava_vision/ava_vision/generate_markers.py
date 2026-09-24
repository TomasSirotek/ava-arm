#!/usr/bin/env python3
"""
ArUco Marker Generator Script

This script generates ArUco markers that can be printed and used for detection.
It creates individual marker images and a PDF sheet with multiple markers.
"""

import cv2
import numpy as np
import os
import argparse


def generate_aruco_marker(marker_id: int, dictionary_type: int, size: int = 200) -> np.ndarray:
    """
    Generate a single ArUco marker image.
    
    Args:
        marker_id: The ID of the marker (0-249 for most dictionaries)
        dictionary_type: OpenCV ArUco dictionary type
        size: Size of the marker in pixels
        
    Returns:
        Marker image as numpy array
    """
    # Get the ArUco dictionary (compatible with different OpenCV versions)
    try:
        # OpenCV 4.7+
        aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_type)
    except AttributeError:
        # OpenCV 4.6 and earlier
        aruco_dict = cv2.aruco.Dictionary_get(dictionary_type)
    
    # Generate the marker (compatible with different OpenCV versions)
    try:
        # OpenCV 4.7+
        marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    except AttributeError:
        # OpenCV 4.6 and earlier
        marker_image = np.zeros((size, size), dtype=np.uint8)
        marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, size, marker_image, 1)
    
    # Add white border for easier printing
    border_size = size // 4
    marker_with_border = cv2.copyMakeBorder(
        marker_image,
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT,
        value=255
    )
    
    return marker_with_border


def generate_marker_sheet(marker_ids: list, dictionary_type: int, 
                          marker_size: int = 150, cols: int = 4) -> np.ndarray:
    """
    Generate a sheet with multiple ArUco markers.
    
    Args:
        marker_ids: List of marker IDs to include
        dictionary_type: OpenCV ArUco dictionary type
        marker_size: Size of each marker in pixels
        cols: Number of columns in the sheet
        
    Returns:
        Sheet image as numpy array
    """
    markers = []
    for marker_id in marker_ids:
        marker = generate_aruco_marker(marker_id, dictionary_type, marker_size)
        # Add marker ID label
        labeled_marker = add_label_to_marker(marker, marker_id)
        markers.append(labeled_marker)
    
    # Calculate grid dimensions
    rows = (len(markers) + cols - 1) // cols
    
    # Create blank sheet
    marker_h, marker_w = markers[0].shape[:2]
    sheet_height = rows * marker_h + 100  # Extra space for title
    sheet_width = cols * marker_w
    sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255
    
    # Add title
    cv2.putText(sheet, "ArUco Markers - DICT_4X4_50", (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
    
    # Place markers on sheet
    for idx, marker in enumerate(markers):
        row = idx // cols
        col = idx % cols
        y = row * marker_h + 100
        x = col * marker_w
        sheet[y:y+marker_h, x:x+marker_w] = marker
    
    return sheet


def add_label_to_marker(marker: np.ndarray, marker_id: int) -> np.ndarray:
    """Add ID label below the marker."""
    h, w = marker.shape[:2]
    label_height = 30
    labeled = np.ones((h + label_height, w), dtype=np.uint8) * 255
    labeled[:h, :] = marker
    cv2.putText(labeled, f"ID: {marker_id}", (w // 2 - 30, h + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, 0, 2)
    return labeled


def main():
    parser = argparse.ArgumentParser(description='Generate ArUco markers for printing')
    parser.add_argument('--output-dir', type=str, default='markers',
                        help='Output directory for generated markers')
    parser.add_argument('--marker-ids', type=str, default='0,1,2,3,4,5',
                        help='Comma-separated list of marker IDs to generate')
    parser.add_argument('--size', type=int, default=200,
                        help='Marker size in pixels')
    parser.add_argument('--dictionary', type=str, default='4X4_50',
                        help='ArUco dictionary (4X4_50, 5X5_100, 6X6_250, 7X7_1000)')
    
    args = parser.parse_args()
    
    # Parse marker IDs
    marker_ids = [int(x.strip()) for x in args.marker_ids.split(',')]
    
    # Dictionary mapping
    dict_map = {
        '4X4_50': cv2.aruco.DICT_4X4_50,
        '4X4_100': cv2.aruco.DICT_4X4_100,
        '5X5_50': cv2.aruco.DICT_5X5_50,
        '5X5_100': cv2.aruco.DICT_5X5_100,
        '6X6_250': cv2.aruco.DICT_6X6_250,
        '7X7_1000': cv2.aruco.DICT_7X7_1000,
    }
    
    dictionary_type = dict_map.get(args.dictionary, cv2.aruco.DICT_4X4_50)
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    print(f"Generating ArUco markers with dictionary: DICT_{args.dictionary}")
    print(f"Output directory: {args.output_dir}")
    print(f"Marker IDs: {marker_ids}")
    print("-" * 50)
    
    # Generate individual markers
    for marker_id in marker_ids:
        marker = generate_aruco_marker(marker_id, dictionary_type, args.size)
        filename = os.path.join(args.output_dir, f"aruco_marker_{marker_id}.png")
        cv2.imwrite(filename, marker)
        print(f"Generated: {filename}")
    
    # Generate marker sheet
    sheet = generate_marker_sheet(marker_ids, dictionary_type, args.size // 2)
    sheet_filename = os.path.join(args.output_dir, "aruco_marker_sheet.png")
    cv2.imwrite(sheet_filename, sheet)
    print(f"Generated marker sheet: {sheet_filename}")
    
    print("-" * 50)
    print("Done! Print the markers and measure their physical size.")
    print("You'll need the marker size (in meters) for pose estimation.")
    print("\nRecommended: Print markers at 5-10 cm size for desk testing.")


if __name__ == '__main__':
    main()
