#!/usr/bin/env python3

"""
This script tracks a human in the camera feed and looks at their head.

Usage:
    python look_at_human.py

Press 'q' to quit.
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import stretch_body.robot as robot
import time

class HumanTracker:
    def __init__(self):
        # Initialize Stretch robot
        print("Initializing Stretch robot...")
        self.robot = robot.Robot()
        self.robot.startup()
        
        # Initialize YOLO pose model
        print("Loading YOLO pose model...")
        self.model = YOLO('yolov8n-pose.pt')  # Using YOLOv8 pose for keypoint detection
        
        # Initialize RealSense camera
        print("Initializing RealSense camera...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        
        # Camera parameters (after rotation, width and height are swapped)
        self.img_width = 480  # After 90° rotation
        self.img_height = 640  # After 90° rotation
        self.img_center_x = self.img_width / 2
        self.img_center_y = self.img_height / 2
        
        # Control parameters
        self.pan_gain = 0.002  # Proportional gain for pan control
        self.tilt_gain = 0.002  # Proportional gain for tilt control
        self.max_pan_speed = 0.3  # Max pan speed (rad/s)
        self.max_tilt_speed = 0.3  # Max tilt speed (rad/s)
        self.error_threshold = 70  # Dead zone threshold in pixels (don't move if error is smaller)
        
        print("Initialization complete!")
    
    def detect_humans(self, frame):
        """
        Detect humans and their head positions in the frame using YOLO pose.
        Returns list of detections: [x1, y1, x2, y2, head_x, head_y, confidence]
        where head_x, head_y is the nose keypoint position (representing head location).
        """
        results = self.model(frame, verbose=False)
        
        detections = []
        for result in results:
            # Get keypoints (pose estimation)
            if result.keypoints is not None:
                keypoints = result.keypoints.xy.cpu().numpy()
                boxes = result.boxes.xyxy.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                
                for i, kpts in enumerate(keypoints):
                    # Keypoint 0 is the nose (head position)
                    # COCO keypoints: 0-nose, 1-left_eye, 2-right_eye, 3-left_ear, 4-right_ear
                    nose_x, nose_y = kpts[0]
                    
                    # Check if nose keypoint is detected (non-zero)
                    if nose_x > 0 and nose_y > 0:
                        x1, y1, x2, y2 = boxes[i]
                        confidence = float(confidences[i])
                        detections.append([x1, y1, x2, y2, nose_x, nose_y, confidence])
        
        return detections
    
    def calculate_target_position(self, detections):
        """
        Calculate the head position of the closest/largest human.
        Returns (head_x, head_y) or None if no humans detected.
        """
        if not detections:
            return None
        
        # Find the largest bounding box (closest person, typically)
        largest_detection = max(detections, key=lambda d: (d[2] - d[0]) * (d[3] - d[1]))
        
        x1, y1, x2, y2, head_x, head_y, _ = largest_detection
        
        return (head_x, head_y)
    
    def control_head(self, target_pos):
        """
        Control the robot's head to look at the target position.
        Uses a dead zone threshold to prevent overshooting and oscillation.
        """
        if target_pos is None:
            return
        
        target_x, target_y = target_pos
        
        # Calculate error from center
        error_x = target_x - self.img_center_x
        error_y = target_y - self.img_center_y
        
        # Apply dead zone threshold to prevent overshooting
        # Only move if error exceeds the threshold
        if abs(error_x) < self.error_threshold:
            pan_velocity = 0
        else:
            # Calculate desired pan velocity
            # Pan: positive is left, negative is right
            # For camera: if person is on right side of frame (error_x > 0), turn right (negative pan)
            pan_velocity = -error_x * self.pan_gain
            pan_velocity = np.clip(pan_velocity, -self.max_pan_speed, self.max_pan_speed)
        
        if abs(error_y) < self.error_threshold:
            tilt_velocity = 0
        else:
            # Calculate desired tilt velocity
            # Tilt: positive is up, negative is down
            # For camera: if person is below center (error_y > 0), tilt down (negative tilt)
            tilt_velocity = -error_y * self.tilt_gain
            tilt_velocity = np.clip(tilt_velocity, -self.max_tilt_speed, self.max_tilt_speed)
        
        # Set head velocities
        self.robot.head.move_by('head_pan', pan_velocity)
        self.robot.head.move_by('head_tilt', tilt_velocity)
        
        # Push commands to robot
        self.robot.push_command()
    
    def draw_detections(self, frame, detections, target_pos):
        """
        Draw bounding boxes, head keypoints, and target information on the frame.
        """
        # Draw all detections
        for detection in detections:
            x1, y1, x2, y2, head_x, head_y, conf = detection
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            head_x, head_y = int(head_x), int(head_y)
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw head keypoint (nose)
            cv2.circle(frame, (head_x, head_y), 8, (255, 0, 255), -1)
            cv2.putText(frame, 'Head', (head_x + 10, head_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
            
            # Draw confidence
            label = f'Person: {conf:.2f}'
            cv2.putText(frame, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw target position (the head we're tracking)
        if target_pos:
            target_x, target_y = int(target_pos[0]), int(target_pos[1])
            cv2.circle(frame, (target_x, target_y), 12, (0, 0, 255), 2)
            cv2.line(frame, (int(self.img_center_x), int(self.img_center_y)), 
                    (target_x, target_y), (0, 0, 255), 2)
        
        # Draw center crosshair
        cv2.drawMarker(frame, (int(self.img_center_x), int(self.img_center_y)), 
                      (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
        
        return frame
    
    def run(self):
        """
        Main loop to track humans and control the head.
        """
        try:
            print("Starting human tracking...")
            print("Press 'q' to quit")
            
            while True:
                # Get frames from camera
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                
                if not color_frame:
                    continue
                
                # Convert to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                
                # Rotate image 90 degrees clockwise
                color_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE)
                
                # Detect humans
                detections = self.detect_humans(color_image)
                
                # Calculate target position
                target_pos = self.calculate_target_position(detections)
                
                # Control head to look at target
                self.control_head(target_pos)
                
                # Draw detections for visualization
                display_image = self.draw_detections(color_image.copy(), detections, target_pos)
                
                # Display status
                status = f"Humans detected: {len(detections)}"
                cv2.putText(display_image, status, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                # Show if target is locked (within dead zone)
                if target_pos:
                    error_x = abs(target_pos[0] - self.img_center_x)
                    error_y = abs(target_pos[1] - self.img_center_y)
                    if error_x < self.error_threshold and error_y < self.error_threshold:
                        lock_status = "LOCKED ON TARGET"
                        cv2.putText(display_image, lock_status, (10, 70),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        # Draw dead zone circle
                        cv2.circle(display_image, (int(self.img_center_x), int(self.img_center_y)),
                                 int(self.error_threshold), (0, 255, 0), 2)
                
                # Show image
                cv2.imshow('Human Tracking', display_image)
                
                # Check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Quitting...")
                    break
                
                # Small delay to allow robot to move
                time.sleep(0.05)
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """
        Clean up resources.
        """
        print("Cleaning up...")
        
        # Stop the robot head
        self.robot.head.move_by('head_pan', 0)
        self.robot.head.move_by('head_tilt', 0)
        self.robot.push_command()
        
        # Stop the robot
        self.robot.stop()
        
        # Stop camera
        self.pipeline.stop()
        
        # Close windows
        cv2.destroyAllWindows()
        
        print("Cleanup complete!")

def main():
    tracker = HumanTracker()
    tracker.run()

if __name__ == "__main__":
    main()
