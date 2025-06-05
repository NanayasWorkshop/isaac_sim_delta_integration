import omni.usd
from pxr import UsdGeom
import math

class ConnectionPointExtractor:
    def __init__(self, robot_path):
        """
        Initialize connection point extractor
        robot_path: Path to the robot in the USD scene (no default - must be provided)
        """
        self.robot_path = robot_path
        self.base_to_first_link_distance = None
        self.segments_found = 0
        self.connection_points = []
        
    def scan_robot_structure(self):
        """Scan the robot to find segments and links"""
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                print("No USD stage available")
                return False
            
            robot_prim = stage.GetPrimAtPath(self.robot_path)
            if not robot_prim or not robot_prim.IsValid():
                print(f"Robot not found at path: {self.robot_path}")
                return False
            
            # Find segments by scanning for seg*_link1 patterns
            segments = []
            self._scan_for_segments(robot_prim, segments)
            
            self.segments_found = len(segments)
            print(f"Found {self.segments_found} segments: {segments}")
            
            if self.segments_found == 0:
                print("No segments found")
                return False
            
            return True
            
        except Exception as e:
            print(f"Error scanning robot structure: {e}")
            return False
    
    def _scan_for_segments(self, prim, segments):
        """Recursively scan for segment links"""
        try:
            prim_name = prim.GetName()
            
            # Check if this is a segment link1 (e.g., seg1_link1, seg2_link1, etc.)
            if prim_name.startswith('seg') and prim_name.endswith('_link1'):
                segment_num = prim_name.split('_')[0]  # Extract 'seg1', 'seg2', etc.
                if segment_num not in segments:
                    segments.append(segment_num)
            
            # Recursively check children
            for child in prim.GetChildren():
                self._scan_for_segments(child, segments)
                
        except Exception as e:
            print(f"Error scanning prim {prim.GetPath()}: {e}")
    
    def get_link_world_position(self, link_path):
        """Get world position of a link"""
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return None
            
            prim = stage.GetPrimAtPath(link_path)
            if not prim or not prim.IsValid():
                print(f"Link not found: {link_path}")
                return None
                
            xformable = UsdGeom.Xformable(prim)
            world_transform = xformable.ComputeLocalToWorldTransform(0)
            translation = world_transform.ExtractTranslation()
            
            return (float(translation[0]), float(translation[1]), float(translation[2]))
            
        except Exception as e:
            print(f"Error getting position for {link_path}: {e}")
            return None
    
    def find_link_in_robot(self, link_name):
        """Find a link anywhere in the robot hierarchy"""
        try:
            stage = omni.usd.get_context().get_stage()
            robot_prim = stage.GetPrimAtPath(self.robot_path)
            
            found_path = self._search_for_link(robot_prim, link_name)
            return found_path
            
        except Exception as e:
            print(f"Error finding link {link_name}: {e}")
            return None
    
    def _search_for_link(self, prim, target_name):
        """Recursively search for a link by name"""
        try:
            if prim.GetName() == target_name:
                return str(prim.GetPath())
            
            for child in prim.GetChildren():
                result = self._search_for_link(child, target_name)
                if result:
                    return result
            
            return None
            
        except Exception:
            return None
    
    def calculate_base_distance(self):
        """Calculate distance from robot base to seg1_link1"""
        try:
            # Get robot base position
            base_pos = self.get_link_world_position(self.robot_path)
            if base_pos is None:
                print("Could not get robot base position")
                return False
            
            # Find seg1_link1
            seg1_link1_path = self.find_link_in_robot("seg1_link1")
            if seg1_link1_path is None:
                print("Could not find seg1_link1")
                return False
            
            seg1_link1_pos = self.get_link_world_position(seg1_link1_path)
            if seg1_link1_pos is None:
                print("Could not get seg1_link1 position")
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
    
    def extract_all_connection_points(self, target_position=None):
        """Extract all connection points from the robot"""
        if not self.scan_robot_structure():
            return []
        
        if not self.calculate_base_distance():
            return []
        
        connection_points = []
        
        try:
            # 1. Base extension point (base -> ground direction)
            base_pos = self.get_link_world_position(self.robot_path)
            seg1_link1_path = self.find_link_in_robot("seg1_link1")
            seg1_link1_pos = self.get_link_world_position(seg1_link1_path)
            
            if base_pos and seg1_link1_pos:
                # Calculate direction from base to seg1_link1
                direction = (
                    seg1_link1_pos[0] - base_pos[0],
                    seg1_link1_pos[1] - base_pos[1],
                    seg1_link1_pos[2] - base_pos[2]
                )
                # Normalize direction
                length = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
                if length > 0:
                    direction = (direction[0]/length, direction[1]/length, direction[2]/length)
                    
                    # Extend in opposite direction (toward ground)
                    base_extension = (
                        base_pos[0] - direction[0] * self.base_to_first_link_distance,
                        base_pos[1] - direction[1] * self.base_to_first_link_distance,
                        base_pos[2] - direction[2] * self.base_to_first_link_distance
                    )
                    connection_points.append(("base_extension", base_extension))
                    print(f"Base extension point: {base_extension}")
            
            # 2. Inter-segment connection points
            for seg_num in range(1, self.segments_found):
                # Get segN_link6 position
                segN_link6_path = self.find_link_in_robot(f"seg{seg_num}_link6")
                segN_link6_pos = self.get_link_world_position(segN_link6_path)
                
                # Get seg(N+1)_link1 position
                segNext_link1_path = self.find_link_in_robot(f"seg{seg_num+1}_link1")
                segNext_link1_pos = self.get_link_world_position(segNext_link1_path)
                
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
            if target_position:
                # Get last segment's link6
                last_seg_link6_path = self.find_link_in_robot(f"seg{self.segments_found}_link6")
                last_seg_link6_pos = self.get_link_world_position(last_seg_link6_path)
                
                if last_seg_link6_pos:
                    # Calculate direction from last link to target
                    direction = (
                        target_position[0] - last_seg_link6_pos[0],
                        target_position[1] - last_seg_link6_pos[1],
                        target_position[2] - last_seg_link6_pos[2]
                    )
                    # Normalize direction
                    length = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
                    if length > 0:
                        direction = (direction[0]/length, direction[1]/length, direction[2]/length)
                        
                        # Extend by fixed distance toward target
                        end_extension = (
                            last_seg_link6_pos[0] + direction[0] * self.base_to_first_link_distance,
                            last_seg_link6_pos[1] + direction[1] * self.base_to_first_link_distance,
                            last_seg_link6_pos[2] + direction[2] * self.base_to_first_link_distance
                        )
                        connection_points.append(("end_extension", end_extension))
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
        # Return just the coordinates as a flat list or structured format
        points_only = [point for name, point in self.connection_points]
        return points_only