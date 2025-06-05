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
        
        # Robot configuration
        self.robot_path = "/World/delta_robot_7_00"  
        print(f"Using robot at: {self.robot_path}")
        
        # Initialize components
        self.sphere_manager = SphereManager()
        self.robot_controller = RobotController(robot_path=self.robot_path)
        self.fabrik_interface = FABRIKInterface()
        self.debug_visualizer = DebugVisualizer()
        self.connection_extractor = ConnectionPointExtractor(robot_path=self.robot_path)
        self.joint_converter = JointConverter()
        
        # State variables
        self.movement_threshold = 0.005  # 5mm
        self.monitoring_active = False
        self.monitoring_subscription = None
        
        # Current state for visualization
        self.current_data = {
            'fabrik_joints': [],
            'segment_end_effectors': [],
            'target': None,
            'connection_points': [],
            'j_points': []
        }
    
    def on_play(self):
        print("Starting robot system...")
        try:
            # Initialize all systems
            if not all([
                self.robot_controller.initialize(),
                self.fabrik_interface.initialize(),
                self.sphere_manager.create_sphere()
            ]):
                print("System initialization failed")
                return
            
            # Extract initial points and start monitoring
            self.extract_and_calculate_points()
            self.start_monitoring()
            print("System ready")
            
        except Exception as e:
            print(f"Error during startup: {e}")
    
    def on_stop(self):
        print("Stopping robot...")
        self._cleanup()
    
    def on_destroy(self):
        print("Destroying robot...")
        self._cleanup()
    
    def _cleanup(self):
        """Centralized cleanup"""
        try:
            self.stop_monitoring()
            self.debug_visualizer.clear_all()
            self.sphere_manager.remove_sphere()
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    def extract_and_calculate_points(self, target_position=None):
        """Extract P points and calculate J points in one operation"""
        try:
            if target_position is None:
                target_position = self.sphere_manager.get_position()
            
            # Extract P points
            self.current_data['connection_points'] = self.connection_extractor.extract_all_connection_points(target_position)
            
            if self.current_data['connection_points']:
                self.connection_extractor.print_all_points()
                
                # Calculate J points from P points
                self.joint_converter.load_isaac_connection_points(self.current_data['connection_points'])
                self.current_data['j_points'] = self.joint_converter.calculate_j_points_from_isaac_p_points()
                self.joint_converter.print_p_and_j_summary()
                
                print(f"Conversion: {len(self.current_data['connection_points'])} P → {len(self.current_data['j_points'])} J points")
            
            return self.current_data['connection_points']
            
        except Exception as e:
            print(f"Error extracting and calculating points: {e}")
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
        """Monitor sphere and control robot"""
        if not self._is_system_ready():
            return
        
        try:
            current_pos = self.sphere_manager.get_position()
            if current_pos is None:
                return
            
            # Initialize on first run
            if self.sphere_manager.last_position is None:
                self.sphere_manager.update_last_position(current_pos)
                self._process_movement(current_pos, is_initial=True)
                return
            
            # Check if movement threshold exceeded
            if self.sphere_manager.has_moved_enough(current_pos, self.movement_threshold):
                self._process_movement(current_pos)
                self.sphere_manager.update_last_position(current_pos)
        
        except Exception as e:
            print(f"Error in monitoring loop: {e}")
    
    def _is_system_ready(self):
        """Check if all systems are initialized and monitoring is active"""
        return (self.monitoring_active and 
                self.robot_controller.is_initialized() and 
                self.fabrik_interface.is_initialized())
    
    def _process_movement(self, target_position, is_initial=False):
        """Process robot movement to target position"""
        try:
            if not is_initial:
                # Extract current state before moving
                print("Extracting current state before movement...")
                current_state_points = self.connection_extractor.extract_current_state_points()
                
                if current_state_points:
                    self.joint_converter.load_isaac_connection_points(current_state_points)
                    current_j_points = self.joint_converter.calculate_j_points_from_isaac_p_points()
                    print(f"Current state: {len(current_state_points)} P → {len(current_j_points)} J points")
            
            # Move robot and update visualization
            if self.move_robot_to_target(target_position):
                self.extract_and_calculate_points(target_position)
        
        except Exception as e:
            print(f"Error processing movement: {e}")
    
    def move_robot_to_target(self, target_position):
        """Move robot to target using FABRIK and update visualization"""
        try:
            # Store target and solve FABRIK
            self.current_data['target'] = target_position
            
            result = self.fabrik_interface.calculate_motors(
                target_position[0], target_position[1], target_position[2]
            )
            
            if result is None:
                return False
            
            # Extract FABRIK data and update visualization
            self.current_data['fabrik_joints'], self.current_data['segment_end_effectors'] = \
                self.fabrik_interface.extract_visualization_data(result)
            
            self._update_visualization()
            
            # Move robot
            return self.robot_controller.move_to_target(result)
            
        except Exception as e:
            print(f"Error moving robot: {e}")
            return False
    
    def _update_visualization(self):
        """Update debug visualization with current data"""
        if self.debug_visualizer.is_enabled():
            self.debug_visualizer.visualize_complete_system(
                self.current_data['fabrik_joints'],
                self.current_data['segment_end_effectors'],
                self.current_data['target'],
                self.current_data['connection_points'],
                self.current_data['j_points']
            )
    
    def toggle_debug_visualization(self):
        """Toggle debug visualization on/off"""
        self.debug_visualizer.toggle()

# Utility functions
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