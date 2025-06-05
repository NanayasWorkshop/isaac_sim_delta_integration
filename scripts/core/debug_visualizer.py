# Try to import debug draw interface (Isaac Sim 4.5.0+)
try:
    from isaacsim.util.debug_draw import _debug_draw
    debug_draw = _debug_draw.acquire_debug_draw_interface()
    DEBUG_DRAW_AVAILABLE = True
    print("Debug draw interface acquired successfully (Isaac Sim 4.5.0+)")
except ImportError:
    try:
        from omni.isaac.debug_draw import _debug_draw
        debug_draw = _debug_draw.acquire_debug_draw_interface()
        DEBUG_DRAW_AVAILABLE = True
        print("Debug draw interface acquired using deprecated API")
    except Exception:
        DEBUG_DRAW_AVAILABLE = False
        debug_draw = None
except Exception:
    DEBUG_DRAW_AVAILABLE = False
    debug_draw = None

class DebugVisualizer:
    def __init__(self):
        self.debug_draw = debug_draw if DEBUG_DRAW_AVAILABLE else None
        self.enabled = True
        self.available = DEBUG_DRAW_AVAILABLE and debug_draw is not None
        
        # Colors (RGBA format)
        self.colors = {
            'green': (0.0, 1.0, 0.0, 1.0),      # Segment end-effectors
            'blue': (0.0, 0.5, 1.0, 1.0),       # FABRIK joints
            'yellow': (1.0, 1.0, 0.0, 1.0),     # Target
            'red': (1.0, 0.0, 0.0, 1.0),        # Base
            'purple': (0.8, 0.0, 1.0, 1.0),     # Connection points
            'orange': (1.0, 0.6, 0.0, 1.0),     # Extension points
            'cyan': (0.0, 1.0, 1.0, 1.0),       # Alternative connection
            'dark_purple': (0.4, 0.0, 0.6, 1.0) # J points
        }
        
        if self.available:
            print("Debug draw interface ready")
        else:
            print("Debug draw not available")
    
    def is_available(self):
        return self.available
    
    def is_enabled(self):
        return self.enabled
    
    def toggle(self):
        self.enabled = not self.enabled
        if not self.enabled:
            self.clear_all()
        print(f"Debug visualization: {'ON' if self.enabled else 'OFF'}")
    
    def clear_all(self):
        if self.debug_draw and self.available:
            try:
                self.debug_draw.clear_lines()
                self.debug_draw.clear_points()
            except Exception as e:
                print(f"Error clearing debug visualization: {e}")
    
    def _draw_points(self, positions, color_key, size=8.0):
        """Helper to draw points with consistent formatting"""
        if not self.enabled or not self.available or not self.debug_draw or not positions:
            return
        
        try:
            point_positions = []
            for pos in positions:
                if hasattr(pos, 'shape'):  # numpy array
                    point_positions.append([float(pos[0]), float(pos[1]), float(pos[2])])
                else:
                    point_positions.append([float(pos[0]), float(pos[1]), float(pos[2])])
            
            colors = [self.colors[color_key]] * len(point_positions)
            sizes = [size] * len(point_positions)
            self.debug_draw.draw_points(point_positions, colors, sizes)
        except Exception as e:
            print(f"Error drawing points: {e}")
    
    def _draw_lines(self, start_positions, end_positions, color_key, width=3.0):
        """Helper to draw lines with consistent formatting"""
        if not self.enabled or not self.available or not self.debug_draw:
            return
        
        try:
            start_pos = [list(pos) for pos in start_positions]
            end_pos = [list(pos) for pos in end_positions]
            colors = [self.colors[color_key]] * len(start_positions)
            widths = [width] * len(start_positions)
            self.debug_draw.draw_lines(start_pos, end_pos, colors, widths)
        except Exception as e:
            print(f"Error drawing lines: {e}")
    
    def _draw_chain(self, positions, point_color, line_color, point_size=8.0, line_width=3.0, special_colors=None):
        """Helper to draw a chain of points with connecting lines"""
        if not positions or len(positions) < 2:
            return
        
        # Draw connecting lines
        for i in range(len(positions) - 1):
            self._draw_lines([positions[i]], [positions[i + 1]], line_color, line_width)
        
        # Draw points with special colors if provided
        if special_colors:
            for i, pos in enumerate(positions):
                color = special_colors.get(i, point_color)
                self._draw_points([pos], color, point_size)
        else:
            self._draw_points(positions, point_color, point_size)
    
    def draw_custom_points(self, positions, colors=None, sizes=None):
        if not self.enabled or not self.available or not self.debug_draw:
            return
        
        try:
            if colors is None:
                colors = [self.colors['blue']] * len(positions)
            if sizes is None:
                sizes = [8.0] * len(positions)
            
            point_positions = [list(pos) for pos in positions]
            self.debug_draw.draw_points(point_positions, colors, sizes)
        except Exception as e:
            print(f"Error drawing custom points: {e}")
    
    def draw_custom_lines(self, start_positions, end_positions, colors=None, widths=None):
        if not self.enabled or not self.available or not self.debug_draw:
            return
        
        try:
            if colors is None:
                colors = [self.colors['blue']] * len(start_positions)
            if widths is None:
                widths = [3.0] * len(start_positions)
            
            start_pos = [list(pos) for pos in start_positions]
            end_pos = [list(pos) for pos in end_positions]
            self.debug_draw.draw_lines(start_pos, end_pos, colors, widths)
        except Exception as e:
            print(f"Error drawing custom lines: {e}")
    
    def visualize_j_points(self, j_points):
        if not j_points:
            return
        
        try:
            # Draw J points and connecting lines
            self._draw_chain(j_points, 'dark_purple', 'dark_purple', 12.0, 3.5)
            print(f"Drawing {len(j_points)} J points with {len(j_points)-1} connecting lines")
        except Exception as e:
            print(f"Error visualizing J points: {e}")
    
    def visualize_connection_points(self, connection_points):
        if not connection_points:
            return
        
        try:
            # Separate connection point types
            base_extensions = []
            inter_connections = []
            end_extensions = []
            
            for name, point in connection_points:
                point_list = list(point)
                if "base_extension" in name:
                    base_extensions.append(point_list)
                elif "end_extension" in name:
                    end_extensions.append(point_list)
                else:
                    inter_connections.append(point_list)
            
            # Draw different types with different colors and sizes
            self._draw_points(base_extensions, 'orange', 16.0)
            self._draw_points(inter_connections, 'purple', 14.0)
            self._draw_points(end_extensions, 'cyan', 16.0)
            
            # Draw connecting lines between all connection points
            all_points = base_extensions + inter_connections + end_extensions
            if len(all_points) > 1:
                for i in range(len(all_points) - 1):
                    self._draw_lines([all_points[i]], [all_points[i + 1]], 'purple', 4.0)
            
            print(f"Drawing {len(base_extensions)} base, {len(inter_connections)} inter, {len(end_extensions)} end points")
        except Exception as e:
            print(f"Error visualizing connection points: {e}")
    
    def visualize_complete_system(self, fabrik_joints, segment_end_effectors, target_position, connection_points, j_points):
        """Main visualization method - draws everything"""
        if not self.enabled or not self.available or not self.debug_draw:
            return
        
        try:
            self.clear_all()
            
            # Draw target point (largest)
            if target_position:
                self._draw_points([target_position], 'yellow', 18.0)
            
            # Draw FABRIK joint chain (blue with red base, yellow end)
            if len(fabrik_joints) > 1:
                special_colors = {0: 'red'}
                if len(fabrik_joints) > 1:
                    special_colors[len(fabrik_joints) - 1] = 'yellow'
                self._draw_chain(fabrik_joints, 'blue', 'blue', 8.0, 3.0, special_colors)
            
            # Draw segment end-effector chain (green with red base)
            if len(segment_end_effectors) > 1:
                special_colors = {0: 'red'}
                self._draw_chain(segment_end_effectors, 'green', 'green', 10.0, 4.0, special_colors)
            
            # Draw connection points (P points)
            if connection_points:
                self.visualize_connection_points(connection_points)
            
            # Draw J points
            if j_points:
                self.visualize_j_points(j_points)
            
            print("Complete system visualization updated (FABRIK + P points + J points)")
        except Exception as e:
            print(f"Error in complete system visualization: {e}")
    
    # Legacy methods for backward compatibility
    def visualize_fabrik_data(self, fabrik_joints, segment_end_effectors, target_position):
        self.visualize_complete_system(fabrik_joints, segment_end_effectors, target_position, [], [])
    
    def visualize_fabrik_and_connections(self, fabrik_joints, segment_end_effectors, target_position, connection_points):
        self.visualize_complete_system(fabrik_joints, segment_end_effectors, target_position, connection_points, [])