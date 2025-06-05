#!/usr/bin/env python3
"""
Isaac Sim 4.5.0 Modular Sphere Following Robot
BehaviorScript for Isaac Sim
"""

import os
import sys

# STEP 1: Configure paths for module imports
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
sys.path.insert(0, os.path.join(script_dir, 'core'))
sys.path.insert(0, os.path.join(script_dir, 'utils'))

# STEP 2: Import Isaac Sim modules (already available in BehaviorScript context)
import omni.kit.app
from omni.kit.scripting import BehaviorScript

# STEP 3: Import our custom modules AFTER path setup
from core.sphere_manager import SphereManager
from core.robot_controller import RobotController
from core.fabrik_interface import FABRIKInterface
from core.debug_visualizer import DebugVisualizer

class SphereFollowingRobotWithDebug(BehaviorScript):
    def on_init(self):
        print("Initializing modular sphere-following robot...")
        
        # Initialize components
        self.sphere_manager = SphereManager()
        self.robot_controller = RobotController()
        self.fabrik_interface = FABRIKInterface()  # UPDATE PATH HERE IF NEEDED
        self.debug_visualizer = DebugVisualizer()
        
        # Movement threshold (0.5cm = 0.005m)
        self.movement_threshold = 0.005
        
        # Monitoring
        self.monitoring_active = False
        self.monitoring_subscription = None
        
        # Visualization data
        self.current_fabrik_joints = []
        self.current_segment_end_effectors = []
        self.current_target = None
    
    def on_play(self):
        print("Starting modular robot system...")
        try:
            # Initialize robot
            success = self.robot_controller.initialize()
            if not success:
                print("Robot setup failed")
                return
            
            # Initialize FABRIK
            success = self.fabrik_interface.initialize()
            if not success:
                print("FABRIK setup failed")
                return
            
            # Create tracking sphere
            success = self.sphere_manager.create_sphere()
            if not success:
                print("Sphere creation failed")
                return
            
            # Start monitoring
            self.start_monitoring()
            print("Modular system ready")
            
        except Exception as e:
            print(f"Error during startup: {e}")
    
    def on_stop(self):
        print("Stopping modular robot...")
        try:
            self.stop_monitoring()
            self.debug_visualizer.clear_all()
            self.cleanup()
        except Exception as e:
            print(f"Error during stop: {e}")
    
    def on_destroy(self):
        print("Destroying modular robot...")
        try:
            self.stop_monitoring()
            self.debug_visualizer.clear_all()
            self.cleanup()
        except Exception as e:
            print(f"Error during destroy: {e}")
    
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
        """Main monitoring loop - checks sphere position and controls robot"""
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
                # Move robot to initial position
                self.move_robot_to_target(current_pos)
                return
            
            # Check if sphere moved enough to trigger robot movement
            if self.sphere_manager.has_moved_enough(current_pos, self.movement_threshold):
                # Move robot to new target
                success = self.move_robot_to_target(current_pos)
                if success:
                    self.sphere_manager.update_last_position(current_pos)
        
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
                self.debug_visualizer.visualize_fabrik_data(
                    self.current_fabrik_joints,
                    self.current_segment_end_effectors,
                    self.current_target
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

# Utility functions for manual control
def move_sphere_to(x, y, z):
    """Manually move the sphere to a specific position"""
    # Create a temporary sphere manager for manual control
    temp_manager = SphereManager()
    return temp_manager.move_to(x, y, z)

def toggle_debug():
    """Toggle debug visualization for the active robot"""
    print("To toggle debug, call robot_instance.toggle_debug_visualization()")

def main():
    """Main function for standalone execution - not used in BehaviorScript mode"""
    print("This script is designed to run as a BehaviorScript in Isaac Sim")
    print("Instructions:")
    print("   1. Update FABRIK path in core/fabrik_interface.py if needed")
    print("   2. Press PLAY to start system")
    print("   3. Move blue sphere to control robot")
    print("   4. Use move_sphere_to(x, y, z) for manual testing")
    print("   Debug: GREEN=segments | BLUE=FABRIK | YELLOW=target | RED=base")
    print(f"   Movement threshold: {0.005*1000:.1f}mm")

# Print instructions when module is loaded
print("Enhanced Modular Sphere Following Delta Robot Script Loaded")
print("Instructions:")
print("   1. Update FABRIK path in core/fabrik_interface.py if needed")
print("   2. Press PLAY to start system")
print("   3. Move blue sphere to control robot")
print("   4. Use move_sphere_to(x, y, z) for manual testing")
print("   Debug: GREEN=segments | BLUE=FABRIK | YELLOW=target | RED=base")
print(f"   Movement threshold: {0.005*1000:.1f}mm")