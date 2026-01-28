import numpy as np
import cv2
import matplotlib.pyplot as plt

class CoordinateMapper:
    def __init__(self, frame_width, frame_height, xy_range=None):
        """
        Initialize coordinate mapper between pixel space and x-y plane.
        
        Args:
            frame_width: Width of the pixel frame
            frame_height: Height of the pixel frame  
            xy_range: Tuple (min_val, max_val) for x-y coordinate range.
                     If None, uses symmetric range around center.
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Calculate center of pixel frame
        self.center_x_pixel = (frame_width - 1) / 2.0
        self.center_y_pixel = (frame_height - 1) / 2.0
        
        # Set x-y coordinate range
        if xy_range is None:
            # Default: symmetric range around center
            max_range = max(frame_width, frame_height) / 2.0
            self.xy_min = -max_range
            self.xy_max = max_range
        else:
            self.xy_min, self.xy_max = xy_range
            
        # Calculate scaling factors
        self.scale_x = (frame_width - 1) / (self.xy_max - self.xy_min)
        self.scale_y = (frame_height - 1) / (self.xy_max - self.xy_min)
    
    def pixel_to_xy(self, x_pixel, y_pixel):
        """
        Convert from pixel coordinates to x-y plane coordinates.
        
        Args:
            x_pixel: X coordinate in pixel space (0 to frame_width-1)
            y_pixel: Y coordinate in pixel space (0 to frame_height-1)
            
        Returns:
            tuple: (x_xy, y_xy) coordinates in x-y plane
        """
        # Translate to center and scale
        x_xy = (x_pixel - self.center_x_pixel) / self.scale_x
        # Flip Y axis (pixel Y increases downward, x-y Y increases upward)
        y_xy = (self.center_y_pixel - y_pixel) / self.scale_y
        
        return x_xy, y_xy
    
    def xy_to_pixel(self, x_xy, y_xy):
        """
        Convert from x-y plane coordinates to pixel coordinates.
        
        Args:
            x_xy: X coordinate in x-y plane
            y_xy: Y coordinate in x-y plane
            
        Returns:
            tuple: (x_pixel, y_pixel) coordinates in pixel space
        """
        # Scale and translate from center
        x_pixel = x_xy * self.scale_x + self.center_x_pixel
        # Flip Y axis (x-y Y increases upward, pixel Y increases downward)
        y_pixel = self.center_y_pixel - y_xy * self.scale_y
        
        return x_pixel, y_pixel
    
    def pixel_to_xy_batch(self, pixel_coords):
        """
        Convert batch of pixel coordinates to x-y coordinates.
        
        Args:
            pixel_coords: Array of shape (N, 2) with [x_pixel, y_pixel] pairs
            
        Returns:
            Array of shape (N, 2) with [x_xy, y_xy] pairs
        """
        pixel_coords = np.array(pixel_coords)
        xy_coords = np.zeros_like(pixel_coords, dtype=float)
        
        xy_coords[:, 0] = (pixel_coords[:, 0] - self.center_x_pixel) / self.scale_x
        xy_coords[:, 1] = (self.center_y_pixel - pixel_coords[:, 1]) / self.scale_y
        
        return xy_coords
    
    def xy_to_pixel_batch(self, xy_coords):
        """
        Convert batch of x-y coordinates to pixel coordinates.
        
        Args:
            xy_coords: Array of shape (N, 2) with [x_xy, y_xy] pairs
            
        Returns:
            Array of shape (N, 2) with [x_pixel, y_pixel] pairs
        """
        xy_coords = np.array(xy_coords)
        pixel_coords = np.zeros_like(xy_coords, dtype=float)
        
        pixel_coords[:, 0] = xy_coords[:, 0] * self.scale_x + self.center_x_pixel
        pixel_coords[:, 1] = self.center_y_pixel - xy_coords[:, 1] * self.scale_y
        
        return pixel_coords

# Example usage for your 700x700 frame
def example_usage():
    # Initialize mapper for 700x700 frame
    # Option 1: Assuming frame coordinates go from (0,0) to (700,700) - 701x701 frame
    mapper1 = CoordinateMapper(701, 701)
    
    # Option 2: Assuming 700x700 frame with coordinates (0,0) to (699,699)  
    mapper2 = CoordinateMapper(700, 700)
    
    print("=== Option 1: 701x701 frame ===")
    print(f"Center pixel: ({mapper1.center_x_pixel}, {mapper1.center_y_pixel})")
    print(f"X-Y range: {mapper1.xy_min} to {mapper1.xy_max}")
    
    # Test key points
    test_pixels = [(0, 0), (350, 350), (700, 700)]
    for px, py in test_pixels:
        x_xy, y_xy = mapper1.pixel_to_xy(px, py)
        px_back, py_back = mapper1.xy_to_pixel(x_xy, y_xy)
        print(f"Pixel ({px}, {py}) -> XY ({x_xy:.1f}, {y_xy:.1f}) -> Pixel ({px_back:.1f}, {py_back:.1f})")
    
    print("\n=== Option 2: 700x700 frame ===")
    print(f"Center pixel: ({mapper2.center_x_pixel}, {mapper2.center_y_pixel})")
    print(f"X-Y range: {mapper2.xy_min} to {mapper2.xy_max}")
    
    # Test key points
    test_pixels = [(0, 0), (349.5, 349.5), (699, 699)]
    for px, py in test_pixels:
        x_xy, y_xy = mapper2.pixel_to_xy(px, py)
        px_back, py_back = mapper2.xy_to_pixel(x_xy, y_xy)
        print(f"Pixel ({px}, {py}) -> XY ({x_xy:.1f}, {y_xy:.1f}) -> Pixel ({px_back:.1f}, {py_back:.1f})")

# Motion planning integration example
def motion_planning_example():
    """Example of how to integrate with motion planning workflow"""
    
    # Initialize mapper (adjust based on your actual frame size)
    mapper = CoordinateMapper(700, 700)
    
    # 1. Get input from CV2 (pixel coordinates)
    start_pixel = (100, 150)  # From CV2 detection
    goal_pixel = (600, 550)   # From CV2 detection
    
    # 2. Convert to x-y coordinates for your motion planner
    start_xy = mapper.pixel_to_xy(*start_pixel)
    goal_xy = mapper.pixel_to_xy(*goal_pixel)
    
    print(f"Start: Pixel {start_pixel} -> XY {start_xy}")
    print(f"Goal: Pixel {goal_pixel} -> XY {goal_xy}")
    
    # 3. Run your motion planning algorithm (example path)
    # This would be your actual motion planning algorithm
    path_xy = generate_sample_path(start_xy, goal_xy)
    
    # 4. Convert path back to pixel coordinates for visualization
    path_pixels = mapper.xy_to_pixel_batch(path_xy)
    
    print(f"Generated path with {len(path_xy)} points")
    print(f"First few path points in XY: {path_xy[:3]}")
    print(f"First few path points in pixels: {path_pixels[:3]}")
    
    return path_pixels

def generate_sample_path(start, goal, num_points=10):
    """Generate a simple straight-line path for demonstration"""
    path = []
    for i in range(num_points):
        t = i / (num_points - 1)
        x = start[0] + t * (goal[0] - start[0])
        y = start[1] + t * (goal[1] - start[1])
        path.append([x, y])
    return np.array(path)

if __name__ == "__main__":
    example_usage()
    print("\n" + "="*50)
    motion_planning_example()