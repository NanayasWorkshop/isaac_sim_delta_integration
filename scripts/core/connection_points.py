import math
from isaac_utils import get_stage_and_robot, get_position, search_for_link

class ConnectionPointExtractor:
    def __init__(self, robot_path):
        self.robot_path = robot_path
        self.base_to_first_link_distance = None
        self.segments_found = 0
        self.connection_points = []
        
    def _scan_for_segments(self, prim, segments):
        """Recursively scan for segment links"""
        prim_name = prim.GetName()
        
        # Check if this is a segment link1
        if prim_name.startswith('seg') and prim_name.endswith('_link1'):
            segment_num = prim_name.split('_')[0]
            if segment_num not in segments:
                segments.append(segment_num)
        
        # Recursively check children
        for child in prim.GetChildren():
            self._scan_for_segments(child, segments)
    
    def scan_robot_structure(self):
        """Scan the robot to find segments and links"""
        try:
            stage, robot_prim = get_stage_and_robot(self.robot_path)
            if not stage or not robot_prim:
                return False
            
            segments = []
            self._scan_for_segments(robot_prim, segments)
            
            self.segments_found = len(segments)
            print(f"Found {self.segments_found} segments: {segments}")
            return self.segments_found > 0
            
        except Exception as e:
            print(f"Error scanning robot structure: {e}")
            return False
    
    def find_link_in_robot(self, link_name):
        """Find a link anywhere in the robot hierarchy"""
        try:
            stage, robot_prim = get_stage_and_robot(self.robot_path)
            if not stage or not robot_prim:
                return None
            
            return search_for_link(robot_prim, link_name)
            
        except Exception as e:
            print(f"Error finding link {link_name}: {e}")
            return None
    
    def calculate_base_distance(self):
        """Calculate distance from robot base to seg1_link1"""
        try:
            # Get positions
            base_pos = get_position(self.robot_path)
            seg1_link1_path = self.find_link_in_robot("seg1_link1")
            
            if not base_pos or not seg1_link1_path:
                print("Could not get required positions for base distance calculation")
                return False
            
            seg1_link1_pos = get_position(seg1_link1_path)
            if not seg1_link1_pos:
                return False
            
            # Calculate distance
            dx = seg1_link1_pos[0] - base_pos[0]
            dy = seg1_link1_pos[1] - base_pos[1]
            dz = seg1_link1_pos[2] - base_pos[2]
            self.base_to_first_link_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            print(f"Base position: {base_pos}")
            print(f"seg1_link1 position: {seg1_link1_pos}")
            print(f"Base to first link distance: {self.base_to_first_link_distance:.6f}m")
            return True
            
        except Exception as e:
            print(f"Error calculating base distance: {e}")
            return False
    
    def _calculate_direction(self, from_pos, to_pos):
        """Calculate normalized direction vector"""
        direction = (to_pos[0] - from_pos[0], to_pos[1] - from_pos[1], to_pos[2] - from_pos[2])
        length = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
        return (direction[0]/length, direction[1]/length, direction[2]/length) if length > 0 else (0, 0, 0)
    
    def _extend_position(self, position, direction, distance):
        """Extend position in given direction by distance"""
        return (
            position[0] + direction[0] * distance,
            position[1] + direction[1] * distance,
            position[2] + direction[2] * distance
        )
    
    def extract_current_state_points(self):
        """Extract connection points from CURRENT robot state"""
        return self.extract_all_connection_points(target_position=None)
    
    def extract_all_connection_points(self, target_position=None):
        """Extract all connection points from the robot"""
        if not self.scan_robot_structure() or not self.calculate_base_distance():
            return []
        
        connection_points = []
        
        try:
            # Get base positions
            base_pos = get_position(self.robot_path)
            seg1_link1_path = self.find_link_in_robot("seg1_link1")
            seg1_link1_pos = get_position(seg1_link1_path)
            
            if not base_pos or not seg1_link1_pos:
                return []
            
            # 1. Base extension point (opposite direction from seg1_link1)
            base_direction = self._calculate_direction(base_pos, seg1_link1_pos)
            base_extension = self._extend_position(base_pos, 
                                                 (-base_direction[0], -base_direction[1], -base_direction[2]), 
                                                 self.base_to_first_link_distance)
            connection_points.append(("base_extension", base_extension))
            print(f"Base extension point: {base_extension}")
            
            # 2. Inter-segment connection points
            for seg_num in range(1, self.segments_found):
                segN_link6_path = self.find_link_in_robot(f"seg{seg_num}_link6")
                segNext_link1_path = self.find_link_in_robot(f"seg{seg_num+1}_link1")
                
                segN_link6_pos = get_position(segN_link6_path)
                segNext_link1_pos = get_position(segNext_link1_path)
                
                if segN_link6_pos and segNext_link1_pos:
                    # Calculate midpoint
                    midpoint = (
                        (segN_link6_pos[0] + segNext_link1_pos[0]) / 2.0,
                        (segN_link6_pos[1] + segNext_link1_pos[1]) / 2.0,
                        (segN_link6_pos[2] + segNext_link1_pos[2]) / 2.0
                    )
                    connection_points.append((f"seg{seg_num}_to_seg{seg_num+1}", midpoint))
                    print(f"Connection seg{seg_num}_link6 to seg{seg_num+1}_link1: {midpoint}")
            
            # 3. End-effector extension point
            last_seg_link6_path = self.find_link_in_robot(f"seg{self.segments_found}_link6")
            last_seg_link6_pos = get_position(last_seg_link6_path)
            
            if last_seg_link6_pos:
                if target_position:
                    # Extend toward target
                    direction = self._calculate_direction(last_seg_link6_pos, target_position)
                    extension_name = "end_extension"
                else:
                    # Extend in current orientation (link5 to link6 direction)
                    last_seg_link5_path = self.find_link_in_robot(f"seg{self.segments_found}_link5")
                    last_seg_link5_pos = get_position(last_seg_link5_path)
                    
                    if last_seg_link5_pos:
                        direction = self._calculate_direction(last_seg_link5_pos, last_seg_link6_pos)
                    else:
                        # Fallback: use base direction
                        direction = self._calculate_direction(base_pos, seg1_link1_pos)
                    extension_name = "end_extension_current"
                
                end_extension = self._extend_position(last_seg_link6_pos, direction, self.base_to_first_link_distance)
                connection_points.append((extension_name, end_extension))
                print(f"End extension point: {end_extension}")
            
            self.connection_points = connection_points
            return connection_points
            
        except Exception as e:
            print(f"Error extracting connection points: {e}")
            return []
    
    def print_all_points(self):
        """Print all extracted connection points"""
        print(f"\n=== Connection Points Summary ===")
        print(f"Robot: {self.robot_path}")
        print(f"Segments found: {self.segments_found}")
        print(f"Base distance: {self.base_to_first_link_distance:.6f}m")
        print(f"Total connection points: {len(self.connection_points)}")
        
        for name, point in self.connection_points:
            print(f"  {name}: ({point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f})")
    
    def get_connection_points_for_cpp(self):
        """Get connection points in format suitable for C++ module"""
        return [point for name, point in self.connection_points]