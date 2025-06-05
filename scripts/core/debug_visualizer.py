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
        
        # Colors (RGBA format) - Enhanced for better visibility
        self.GREEN = (0.0, 1.0, 0.0, 1.0)   # Segment end-effectors
        self.BLUE = (0.0, 0.5, 1.0, 1.0)    # FABRIK joints
        self.YELLOW = (1.0, 1.0, 0.0, 1.0)  # Target
        self.RED = (1.0, 0.0, 0.0, 1.0)     # Base
        self.PURPLE = (0.8, 0.0, 1.0, 1.0)  # Connection points (brighter purple)
        self.ORANGE = (1.0, 0.6, 0.0, 1.0)  # Extension points (brighter orange)
        self.CYAN = (0.0, 1.0, 1.0, 1.0)    # Alternative connection color
        self.DARK_PURPLE = (0.4, 0.0, 0.6, 1.0)  # NEW: Dark purple for J points
    
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
    
    def visualize_j_points(self, j_points):
        """Visualize J points with dark purple color and connecting lines"""
        if not self.enabled or not self.available or not self.debug_draw or not j_points:
            return
        
        try:
            # Convert J points to list format if they're numpy arrays
            j_positions = []
            for j in j_points:
                if hasattr(j, 'shape'):  # numpy array
                    j_positions.append([float(j[0]), float(j[1]), float(j[2])])
                else:
                    j_positions.append([float(j[0]), float(j[1]), float(j[2])])
            
            # Draw J points as dark purple points (larger size for visibility)
            j_colors = [self.DARK_PURPLE] * len(j_positions)
            j_sizes = [12.0] * len(j_positions)  # Larger than other points
            self.debug_draw.draw_points(j_positions, j_colors, j_sizes)
            
            # Draw connecting lines between consecutive J points
            if len(j_positions) > 1:
                for i in range(len(j_positions) - 1):
                    self.debug_draw.draw_lines([j_positions[i]], [j_positions[i + 1]], 
                                             [self.DARK_PURPLE], [3.5])  # Slightly thicker lines
            
            print(f"Drawing {len(j_positions)} J points with {len(j_positions)-1} connecting lines")
            
        except Exception as e:
            print(f"Error visualizing J points: {e}")
    
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
    
    def visualize_connection_points(self, connection_points):
        """Visualize connection points with enhanced visibility"""
        if not self.enabled or not self.available or not self.debug_draw or not connection_points:
            return
        
        try:
            # Separate different types of connection points
            base_extensions = []
            inter_connections = []
            end_extensions = []
            
            for name, point in connection_points:
                if "base_extension" in name:
                    base_extensions.append(list(point))
                elif "end_extension" in name:
                    end_extensions.append(list(point))
                else:
                    inter_connections.append(list(point))
            
            # Draw base extensions (ORANGE) - larger size for visibility
            if base_extensions:
                self.debug_draw.draw_points(base_extensions, [self.ORANGE] * len(base_extensions), [16.0] * len(base_extensions))
                print(f"Drawing {len(base_extensions)} base extension points")
            
            # Draw inter-segment connections (PURPLE) - larger size for visibility
            if inter_connections:
                self.debug_draw.draw_points(inter_connections, [self.PURPLE] * len(inter_connections), [14.0] * len(inter_connections))
                print(f"Drawing {len(inter_connections)} inter-segment connection points")
            
            # Draw end extensions (CYAN) - different color for end extensions
            if end_extensions:
                self.debug_draw.draw_points(end_extensions, [self.CYAN] * len(end_extensions), [16.0] * len(end_extensions))
                print(f"Drawing {len(end_extensions)} end extension points")
            
            # Draw connecting lines between connection points - thicker lines
            all_points = base_extensions + inter_connections + end_extensions
            if len(all_points) > 1:
                for i in range(len(all_points) - 1):
                    self.debug_draw.draw_lines([all_points[i]], [all_points[i + 1]], [self.PURPLE], [4.0])
                print(f"Drawing {len(all_points)-1} connection lines")
            
        except Exception as e:
            print(f"Error visualizing connection points: {e}")
    
    def visualize_fabrik_and_connections(self, fabrik_joints, segment_end_effectors, target_position, connection_points):
        """Combined visualization of FABRIK data and connection points (legacy method)"""
        self.visualize_complete_system(fabrik_joints, segment_end_effectors, target_position, connection_points, [])
    
    def visualize_complete_system(self, fabrik_joints, segment_end_effectors, target_position, connection_points, j_points):
        """Complete visualization: FABRIK data, connection points, and J points"""
        if not self.enabled or not self.available or not self.debug_draw:
            return
        
        try:
            self.clear_all()
            
            # Draw target point first
            if target_position:
                target_pos = list(target_position)
                self.debug_draw.draw_points([target_pos], [self.YELLOW], [18.0])  # Larger target
            
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
            
            # Draw connection points (P points)
            if connection_points:
                # Separate different types of connection points
                base_extensions = []
                inter_connections = []
                end_extensions = []
                
                for name, point in connection_points:
                    if "base_extension" in name:
                        base_extensions.append(list(point))
                    elif "end_extension" in name:
                        end_extensions.append(list(point))
                    else:
                        inter_connections.append(list(point))
                
                # Draw base extensions (ORANGE)
                if base_extensions:
                    self.debug_draw.draw_points(base_extensions, [self.ORANGE] * len(base_extensions), [16.0] * len(base_extensions))
                
                # Draw inter-segment connections (PURPLE)
                if inter_connections:
                    self.debug_draw.draw_points(inter_connections, [self.PURPLE] * len(inter_connections), [14.0] * len(inter_connections))
                
                # Draw end extensions (CYAN)
                if end_extensions:
                    self.debug_draw.draw_points(end_extensions, [self.CYAN] * len(end_extensions), [16.0] * len(end_extensions))
                
                # Draw connecting lines between connection points
                all_connection_points = base_extensions + inter_connections + end_extensions
                if len(all_connection_points) > 1:
                    for i in range(len(all_connection_points) - 1):
                        self.debug_draw.draw_lines([all_connection_points[i]], [all_connection_points[i + 1]], [self.PURPLE], [4.0])
            
            # Draw J points (NEW!)
            if j_points:
                self.visualize_j_points(j_points)
            
            print("Complete system visualization updated (FABRIK + P points + J points)")
            
        except Exception as e:
            print(f"Error in complete system visualization: {e}")