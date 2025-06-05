#!/usr/bin/env python3
"""
Isaac Sim 4.5.0 Modular Sphere Following Robot with Joint Converter
BehaviorScript for Isaac Sim
"""

import os
import sys

# Configure paths for module imports
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
sys.path.insert(0, os.path.join(script_dir, 'core'))
sys.path.insert(0, os.path.join(script_dir, 'utils'))

# Import Isaac Sim modules
import omni.kit.app
from omni.kit.scripting import BehaviorScript

# Import custom modules
from core.sphere_manager import SphereManager
from core.robot_controller import RobotController
from core.fabrik_interface import FABRIKInterface
from core.debug_visualizer import DebugVisualizer
from core.connection_points import ConnectionPointExtractor
from core.joint_converter import JointConverter

class SphereFollowingRobotWithDebug(BehaviorScript):
    def on_init(self):
        print("Initializing modular sphere-following robot...")
        
        # ===== ROBOT CONFIGURATION =====
        self.robot_path = "/World/delta_robot_7_00"  
        print(f"Using robot at: {self.robot_path}")
        
        # Initialize components
        self.sphere_manager = SphereManager()
        self.robot_controller = RobotController(robot_path=self.robot_path)
        self.fabrik_interface = FABRIKInterface()
        self.debug_visualizer = DebugVisualizer()
        self.connection_extractor = ConnectionPointExtractor(robot_path=self.robot_path)
        self.joint_converter = JointConverter()
        
        # Movement threshold (0.5cm = 0.005m)
        self.movement_threshold = 0.005
        
        # Monitoring
        self.monitoring_active = False
        self.monitoring_subscription = None
        
        # Visualization data
        self.current_fabrik_joints = []
        self.current_segment_end_effectors = []
        self.current_target = None
        self.current_connection_points = []
        self.current_j_points = []
    
    def on_play(self):
        print("Starting robot system...")
        try:
            # Initialize robot
            if not self.robot_controller.initialize():
                print("Robot setup failed")
                return
            
            # Initialize FABRIK
            if not self.fabrik_interface.initialize():
                print("FABRIK setup failed")
                return
            
            # Create tracking sphere
            if not self.sphere_manager.create_sphere():
                print("Sphere creation failed")
                return
            
            # Extract initial connection points and calculate J points
            self.extract_and_calculate_points()
            
            # Start monitoring
            self.start_monitoring()
            print("System ready")
            
        except Exception as e:
            print(f"Error during startup: {e}")
    
    def on_stop(self):
        print("Stopping robot...")
        try:
            self.stop_monitoring()
            self.debug_visualizer.clear_all()
            self.cleanup()
        except Exception as e:
            print(f"Error during stop: {e}")
    
    def on_destroy(self):
        print("Destroying robot...")
        try:
            self.stop_monitoring()
            self.debug_visualizer.clear_all()
            self.cleanup()
        except Exception as e:
            print(f"Error during destroy: {e}")
    
    def extract_and_calculate_points(self, target_position=None):
        """Extract P points and calculate J points"""
        try:
            if target_position is None:
                target_position = self.sphere_manager.get_position()
            
            # Extract P points (connection points)
            self.current_connection_points = self.connection_extractor.extract_all_connection_points(target_position)
            self.connection_extractor.print_all_points()
            
            # Calculate J points from P points
            self.calculate_j_points_from_p_points()
            
            return self.current_connection_points
            
        except Exception as e:
            print(f"Error extracting and calculating points: {e}")
            return []
    
    def calculate_j_points_from_p_points(self):
        """Calculate J points from current P points using joint converter"""
        try:
            if not self.current_connection_points:
                print("No connection points available for J point calculation")
                return []
            
            # Load P points into joint converter
            self.joint_converter.load_isaac_connection_points(self.current_connection_points)
            
            # Calculate J points
            self.current_j_points = self.joint_converter.calculate_j_points_from_isaac_p_points()
            
            # Print complete summary
            self.joint_converter.print_p_and_j_summary()
            
            print(f"Conversion complete: {len(self.current_connection_points)} P → {len(self.current_j_points)} J points")
            
            return self.current_j_points
            
        except Exception as e:
            print(f"Error calculating J points: {e}")
            return []
    
    def start_monitoring(self):
        """Start monitoring sphere position"""
        try:
            self.stop_monitoring()
            self.monitoring_active = True
            
            update_stream = omni.kit.app.get_app().get_update_event_stream()
            self.monitoring_subscription = update_stream.create_subscription_to_pop(self._monitor_and_control)
            
            print("Started monitoring sphere position")
            return True
            
        except Exception as e:
            print(f"Error starting monitoring: {e}")
            return False
    
    def stop_monitoring(self):
        """Stop monitoring sphere position"""
        self.monitoring_active = False
        if self.monitoring_subscription:
            self.monitoring_subscription.unsubscribe()
            self.monitoring_subscription = None
    
    def _monitor_and_control(self, dt):
        """Monitor sphere and control robot - extract current state before moving"""
        if not self.monitoring_active:
            return
        
        if not (self.robot_controller.is_initialized() and self.fabrik_interface.is_initialized()):
            return
        
        try:
            current_pos = self.sphere_manager.get_position()
            if current_pos is None:
                return
            
            # Initialize last position on first run
            if self.sphere_manager.last_position is None:
                self.sphere_manager.update_last_position(current_pos)
                self.move_robot_to_target(current_pos)
                self.extract_and_calculate_points(current_pos)
                return
            
            # Check if sphere moved enough to trigger robot movement
            if self.sphere_manager.has_moved_enough(current_pos, self.movement_threshold):
                # Extract current state before moving
                print("Extracting current state before movement...")
                current_state_points = self.connection_extractor.extract_current_state_points()
                
                if current_state_points:
                    # Calculate current state J points
                    self.joint_converter.load_isaac_connection_points(current_state_points)
                    current_j_points = self.joint_converter.calculate_j_points_from_isaac_p_points()
                    print(f"Current state: {len(current_state_points)} P → {len(current_j_points)} J points")
                
                # Move robot to new target
                success = self.move_robot_to_target(current_pos)
                if success:
                    self.sphere_manager.update_last_position(current_pos)
                    # Extract target state after movement
                    self.extract_and_calculate_points(current_pos)
        
        except Exception as e:
            print(f"Error in monitoring loop: {e}")
    
    def move_robot_to_target(self, target_position):
        """Use FABRIK to move robot to target position"""
        try:
            # Store current target for debug visualization
            self.current_target = target_position
            
            # Solve FABRIK
            result = self.fabrik_interface.calculate_motors(
                target_position[0], target_position[1], target_position[2]
            )
            
            if result is None:
                return False
            
            # Extract FABRIK data for visualization
            self.current_fabrik_joints, self.current_segment_end_effectors = \
                self.fabrik_interface.extract_visualization_data(result)
            
            # Update debug visualization
            if self.debug_visualizer.is_enabled():
                self.debug_visualizer.visualize_complete_system(
                    self.current_fabrik_joints,
                    self.current_segment_end_effectors,
                    self.current_target,
                    self.current_connection_points,
                    self.current_j_points
                )
            
            # Move robot using FABRIK result
            success = self.robot_controller.move_to_target(result)
            
            return success
            
        except Exception as e:
            print(f"Error moving robot: {e}")
            return False
    
    def toggle_debug_visualization(self):
        """Toggle debug visualization on/off"""
        self.debug_visualizer.toggle()
    
    def cleanup(self):
        """Clean up resources"""
        self.sphere_manager.remove_sphere()

# Utility function
def move_sphere_to(x, y, z):
    """Manually move the sphere to a specific position"""
    temp_manager = SphereManager()
    return temp_manager.move_to(x, y, z)

def main():
    """Main function - not used in BehaviorScript mode"""
    print("Run as BehaviorScript in Isaac Sim")

# Instructions when module is loaded
print("=== Modular Sphere Following Robot ===")
print("1. Set robot path in on_init() (line 28)")
print("2. Press PLAY to start")
print("3. Move blue sphere to control robot")
print("4. Use move_sphere_to(x, y, z) for manual control")
print("5. Use robot_instance.toggle_debug_visualization() to toggle debug")
print("Movement threshold: 5mm")