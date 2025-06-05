# Try to import debug draw interface (Isaac Sim 4.5.0+)
try:
    # Use the NEW import path for Isaac Sim 4.5.0+
    from isaacsim.util.debug_draw import _debug_draw
    debug_draw = _debug_draw.acquire_debug_draw_interface()
    DEBUG_DRAW_AVAILABLE = True
    print("Debug draw interface acquired successfully (Isaac Sim 4.5.0+)")
    
except ImportError as e:
    print(f"Failed to import isaacsim.util.debug_draw: {e}")
    print("Trying fallback to deprecated omni.isaac.debug_draw...")
    try:
        # Fallback to old API for older Isaac Sim versions
        from omni.isaac.debug_draw import _debug_draw
        debug_draw = _debug_draw.acquire_debug_draw_interface()
        DEBUG_DRAW_AVAILABLE = True
        print("Debug draw interface acquired using deprecated API")
    except Exception as fallback_e:
        print(f"Both debug draw imports failed: {fallback_e}")
        DEBUG_DRAW_AVAILABLE = False
        debug_draw = None

except Exception as e:
    print(f"Failed to acquire debug draw interface: {e}")
    DEBUG_DRAW_AVAILABLE = False
    debug_draw = None

class DebugVisualizer:
    def __init__(self):
        self.debug_draw = None
        self.enabled = True
        self.available = False
        self.setup_debug_draw()
        
        # Colors (RGBA format)
        self.GREEN = (0.0, 1.0, 0.0, 1.0)   # Segment end-effectors
        self.BLUE = (0.0, 0.5, 1.0, 1.0)    # FABRIK joints
        self.YELLOW = (1.0, 1.0, 0.0, 1.0)  # Target
        self.RED = (1.0, 0.0, 0.0, 1.0)     # Base
    
    def setup_debug_draw(self):
        """Initialize debug drawing system"""
        if DEBUG_DRAW_AVAILABLE and debug_draw:
            try:
                self.debug_draw = debug_draw
                self.available = True
                print("Debug draw interface ready")
            except Exception as e:
                print(f"Error setting up debug draw interface: {e}")
                self.debug_draw = None
                self.available = False
        else:
            print("Debug draw not available")
            self.debug_draw = None
            self.available = False
    
    def is_available(self):
        """Check if debug drawing is available"""
        return self.available
    
    def is_enabled(self):
        """Check if debug visualization is enabled"""
        return self.enabled
    
    def toggle(self):
        """Toggle debug visualization on/off"""
        self.enabled = not self.enabled
        if not self.enabled:
            self.clear_all()
        print(f"Debug visualization: {'ON' if self.enabled else 'OFF'}")
    
    def clear_all(self):
        """Clear all debug visualization"""
        if self.debug_draw and self.available:
            try:
                self.debug_draw.clear_lines()
                self.debug_draw.clear_points()
            except Exception as e:
                print(f"Error clearing debug visualization: {e}")
    
    def visualize_fabrik_data(self, fabrik_joints, segment_end_effectors, target_position):
        """Update debug visualization using Isaac Sim debug draw"""
        if not self.enabled or not self.available or not self.debug_draw:
            return
        
        try:
            self.clear_all()
            
            # Draw target point
            if target_position:
                target_pos = list(target_position)
                self.debug_draw.draw_points([target_pos], [self.YELLOW], [15.0])
            
            # Draw FABRIK joint chain (BLUE)
            if len(fabrik_joints) > 1:
                # Draw connecting lines
                for i in range(len(fabrik_joints) - 1):
                    start_pos = list(fabrik_joints[i])
                    end_pos = list(fabrik_joints[i + 1])
                    self.debug_draw.draw_lines([start_pos], [end_pos], [self.BLUE], [3.0])
                
                # Draw joint points
                joint_positions = [list(pos) for pos in fabrik_joints]
                joint_sizes = [8.0] * len(joint_positions)
                joint_colors = [self.BLUE] * len(joint_positions)
                
                if len(joint_colors) > 0:
                    joint_colors[0] = self.RED  # Base
                if len(joint_colors) > 1:
                    joint_colors[-1] = self.YELLOW  # Final joint
                
                self.debug_draw.draw_points(joint_positions, joint_colors, joint_sizes)
            
            # Draw segment end-effector chain (GREEN)
            if len(segment_end_effectors) > 1:
                # Draw connecting lines
                for i in range(len(segment_end_effectors) - 1):
                    start_pos = list(segment_end_effectors[i])
                    end_pos = list(segment_end_effectors[i + 1])
                    self.debug_draw.draw_lines([start_pos], [end_pos], [self.GREEN], [4.0])
                
                # Draw segment points
                seg_positions = [list(pos) for pos in segment_end_effectors]
                seg_sizes = [10.0] * len(seg_positions)
                seg_colors = [self.GREEN] * len(seg_positions)
                
                if len(seg_colors) > 0:
                    seg_colors[0] = self.RED  # Base
                
                self.debug_draw.draw_points(seg_positions, seg_colors, seg_sizes)
            
        except Exception as e:
            print(f"Error updating debug visualization: {e}")
    
    def draw_custom_points(self, positions, colors=None, sizes=None):
        """Draw custom points"""
        if not self.enabled or not self.available or not self.debug_draw:
            return
        
        try:
            if colors is None:
                colors = [self.BLUE] * len(positions)
            if sizes is None:
                sizes = [8.0] * len(positions)
            
            point_positions = [list(pos) for pos in positions]
            self.debug_draw.draw_points(point_positions, colors, sizes)
            
        except Exception as e:
            print(f"Error drawing custom points: {e}")
    
    def draw_custom_lines(self, start_positions, end_positions, colors=None, widths=None):
        """Draw custom lines"""
        if not self.enabled or not self.available or not self.debug_draw:
            return
        
        try:
            if colors is None:
                colors = [self.BLUE] * len(start_positions)
            if widths is None:
                widths = [3.0] * len(start_positions)
            
            start_pos = [list(pos) for pos in start_positions]
            end_pos = [list(pos) for pos in end_positions]
            self.debug_draw.draw_lines(start_pos, end_pos, colors, widths)
            
        except Exception as e:
            print(f"Error drawing custom lines: {e}")
