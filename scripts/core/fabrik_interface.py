import sys
import numpy as np
from math_utils import meters_to_mm, mm_to_meters

class FABRIKInterface:
    def __init__(self, delta_path="/home/yuuki/Documents/FABRIK_CPP/delta_unit"):
        self.delta_path = delta_path
        self.delta_robot = None
        self.initialized = False
        
    def initialize(self):
        """Initialize FABRIK solver"""
        try:
            if self.delta_path not in sys.path:
                sys.path.append(self.delta_path)
            
            import delta_robot
            self.delta_robot = delta_robot
            
            print("FABRIK solver initialized")
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"FABRIK setup failed: {e}")
            print(f"Please update the delta_path: {self.delta_path}")
            return False
    
    def is_initialized(self):
        """Check if FABRIK is initialized"""
        return self.initialized
    
    def calculate_motors(self, target_x_m, target_y_m, target_z_m, current_j_points=None, collision_data=None):
        """Calculate motor positions for target (input in meters, converts to mm internally)"""
        if not self.initialized:
            print("FABRIK not initialized")
            return None
        
        try:
            # Convert target to millimeters
            target_x_mm = meters_to_mm(target_x_m)
            target_y_mm = meters_to_mm(target_y_m)
            target_z_mm = meters_to_mm(target_z_m)
            
            # Phase 1: Convert collision data to C++ format (Phase 2: C++ will use it)
            cpp_collision_data = None
            if collision_data:
                cpp_collision_data = self._convert_collision_data_to_cpp_format(collision_data)
            
            # Determine FABRIK method and execute
            if current_j_points and len(current_j_points) > 0:
                fabrik_j_points = self._convert_j_points_to_fabrik_format(current_j_points)
                if fabrik_j_points is not None:
                    collision_msg = " + collision data" if cpp_collision_data else ""
                    print(f"Target: ({target_x_mm:.1f}, {target_y_mm:.1f}, {target_z_mm:.1f})mm - FABRIK: From J points{collision_msg}")
                    
                    # Note: Current C++ FABRIK doesn't accept J points parameter yet
                    # For Phase 1, we'll use the basic 3-parameter version
                    # Phase 2 will modify C++ to accept J points and collision data
                    result = self.delta_robot.motor.MotorModule.calculate_motors(
                        target_x_mm, target_y_mm, target_z_mm
                    )
                else:
                    collision_msg = " + collision data" if cpp_collision_data else ""
                    print(f"Target: ({target_x_mm:.1f}, {target_y_mm:.1f}, {target_z_mm:.1f})mm - FABRIK: J points invalid, straight-up{collision_msg}")
                    result = self.delta_robot.motor.MotorModule.calculate_motors(target_x_mm, target_y_mm, target_z_mm)
            else:
                collision_msg = " + collision data" if cpp_collision_data else ""
                print(f"Target: ({target_x_mm:.1f}, {target_y_mm:.1f}, {target_z_mm:.1f})mm - FABRIK: Straight-up{collision_msg}")
                result = self.delta_robot.motor.MotorModule.calculate_motors(target_x_mm, target_y_mm, target_z_mm)
            
            if result:
                print(f"FABRIK converged: {result.fabrik_converged}")
            
            return result
            
        except Exception as e:
            print(f"Error calculating motors: {e}")
            return None
    
    def _convert_collision_data_to_cpp_format(self, collision_data):
        """Convert collision data to C++ compatible format with rotation support"""
        try:
            if not collision_data:
                return None
            
            cpp_format = {
                'spheres': [],
                'sphere_count': collision_data.get('sphere_count', 0),
                'boxes': [], 
                'box_count': collision_data.get('box_count', 0),
                'cylinders': [],
                'cylinder_count': collision_data.get('cylinder_count', 0)
            }
            
            # Convert sphere data
            for sphere in collision_data.get('spheres', []):
                cpp_sphere = {
                    'center_x': float(sphere['center'][0]),  # mm
                    'center_y': float(sphere['center'][1]),  # mm
                    'center_z': float(sphere['center'][2]),  # mm
                    'radius': float(sphere['radius']),       # mm
                    # Rotation matrix as flat array (row-major order)
                    'rotation_matrix': [
                        float(sphere['rotation_matrix'][0][0]), float(sphere['rotation_matrix'][0][1]), float(sphere['rotation_matrix'][0][2]),
                        float(sphere['rotation_matrix'][1][0]), float(sphere['rotation_matrix'][1][1]), float(sphere['rotation_matrix'][1][2]),
                        float(sphere['rotation_matrix'][2][0]), float(sphere['rotation_matrix'][2][1]), float(sphere['rotation_matrix'][2][2])
                    ],
                    # Quaternion (x, y, z, w) - better for C++
                    'quat_x': float(sphere['quaternion'][0]),
                    'quat_y': float(sphere['quaternion'][1]),
                    'quat_z': float(sphere['quaternion'][2]),
                    'quat_w': float(sphere['quaternion'][3]),
                    'is_uniform_scale': bool(sphere['is_uniform_scale'])
                }
                cpp_format['spheres'].append(cpp_sphere)
            
            # Convert box data (OBB support)
            for box in collision_data.get('boxes', []):
                cpp_box = {
                    'center_x': float(box['center'][0]),     # mm
                    'center_y': float(box['center'][1]),     # mm
                    'center_z': float(box['center'][2]),     # mm
                    'size_x': float(box['size'][0]),         # mm
                    'size_y': float(box['size'][1]),         # mm
                    'size_z': float(box['size'][2]),         # mm
                    # Rotation matrix as flat array (row-major order)
                    'rotation_matrix': [
                        float(box['rotation_matrix'][0][0]), float(box['rotation_matrix'][0][1]), float(box['rotation_matrix'][0][2]),
                        float(box['rotation_matrix'][1][0]), float(box['rotation_matrix'][1][1]), float(box['rotation_matrix'][1][2]),
                        float(box['rotation_matrix'][2][0]), float(box['rotation_matrix'][2][1]), float(box['rotation_matrix'][2][2])
                    ],
                    # Quaternion (x, y, z, w) - better for C++
                    'quat_x': float(box['quaternion'][0]),
                    'quat_y': float(box['quaternion'][1]),
                    'quat_z': float(box['quaternion'][2]),
                    'quat_w': float(box['quaternion'][3]),
                    'is_axis_aligned': bool(box['is_axis_aligned'])  # AABB vs OBB
                }
                cpp_format['boxes'].append(cpp_box)
            
            # Convert cylinder data
            for cylinder in collision_data.get('cylinders', []):
                cpp_cylinder = {
                    'center_x': float(cylinder['center'][0]),  # mm
                    'center_y': float(cylinder['center'][1]),  # mm
                    'center_z': float(cylinder['center'][2]),  # mm
                    'radius': float(cylinder['radius']),       # mm
                    'height': float(cylinder['height']),       # mm
                    # Rotation matrix as flat array (row-major order)
                    'rotation_matrix': [
                        float(cylinder['rotation_matrix'][0][0]), float(cylinder['rotation_matrix'][0][1]), float(cylinder['rotation_matrix'][0][2]),
                        float(cylinder['rotation_matrix'][1][0]), float(cylinder['rotation_matrix'][1][1]), float(cylinder['rotation_matrix'][1][2]),
                        float(cylinder['rotation_matrix'][2][0]), float(cylinder['rotation_matrix'][2][1]), float(cylinder['rotation_matrix'][2][2])
                    ],
                    # Quaternion (x, y, z, w) - better for C++
                    'quat_x': float(cylinder['quaternion'][0]),
                    'quat_y': float(cylinder['quaternion'][1]),
                    'quat_z': float(cylinder['quaternion'][2]),
                    'quat_w': float(cylinder['quaternion'][3]),
                    'is_circular': bool(cylinder['is_circular']),
                    'is_axis_aligned': bool(cylinder['is_axis_aligned'])
                }
                cpp_format['cylinders'].append(cpp_cylinder)
            
            total_objects = cpp_format['sphere_count'] + cpp_format['box_count'] + cpp_format['cylinder_count']
            aabb_count = sum(1 for box in cpp_format['boxes'] if box['is_axis_aligned'])
            obb_count = cpp_format['box_count'] - aabb_count
            
            print(f"Collision data formatted: {cpp_format['sphere_count']} spheres, "
                  f"{aabb_count} AABB boxes, {obb_count} OBB boxes, "
                  f"{cpp_format['cylinder_count']} cylinders ({total_objects} total)")
            return cpp_format
            
        except Exception as e:
            print(f"Error converting collision data: {e}")
            return None
    
    def _convert_j_points_to_fabrik_format(self, j_points):
        """Convert J points from meters to millimeters for FABRIK C++"""
        try:
            fabrik_points = []
            for i, j_point in enumerate(j_points):
                # Handle both numpy arrays and tuples/lists
                if hasattr(j_point, 'shape'):  # numpy array
                    point_mm = np.array([meters_to_mm(float(j_point[0])), 
                                       meters_to_mm(float(j_point[1])), 
                                       meters_to_mm(float(j_point[2]))])
                elif hasattr(j_point, '__len__') and len(j_point) == 3:  # tuple/list
                    point_mm = np.array([meters_to_mm(float(j_point[0])), 
                                       meters_to_mm(float(j_point[1])), 
                                       meters_to_mm(float(j_point[2]))])
                else:
                    print(f"Warning: J point {i} has unexpected format")
                    return None
                
                fabrik_points.append(point_mm)
            
            # Ensure base is at origin for FABRIK (within 1mm tolerance)
            if len(fabrik_points) > 0:
                base_distance = np.linalg.norm(fabrik_points[0])
                if base_distance > 1.0:
                    fabrik_points[0] = np.array([0.0, 0.0, 0.0])
            
            return fabrik_points
            
        except Exception as e:
            print(f"Error converting J points: {e}")
            return None
    
    def extract_visualization_data(self, motor_result):
        """Extract FABRIK joint positions and segment end-effectors for visualization"""
        fabrik_joints = []
        segment_end_effectors = []
        
        try:
            # Extract FABRIK joint positions (convert from mm to m)
            if hasattr(motor_result, 'fabrik_joint_positions'):
                for joint_pos in motor_result.fabrik_joint_positions:
                    pos_m = self._extract_position_to_meters(joint_pos)
                    if pos_m:
                        fabrik_joints.append(pos_m)
            
            # Extract segment end-effector positions (convert from mm to m)
            if hasattr(motor_result, 'original_segment_positions'):
                # Add base position
                segment_end_effectors.append((0, 0, 0))
                
                for seg_pos in motor_result.original_segment_positions:
                    pos_m = self._extract_position_to_meters(seg_pos)
                    if pos_m:
                        segment_end_effectors.append(pos_m)
            
        except Exception as e:
            print(f"Error extracting FABRIK data: {e}")
        
        return fabrik_joints, segment_end_effectors
    
    def _extract_position_to_meters(self, position):
        """Extract position from various formats and convert to meters"""
        try:
            if hasattr(position, 'shape'):  # numpy array
                return (mm_to_meters(position[0]), mm_to_meters(position[1]), mm_to_meters(position[2]))
            else:
                # Try to access as object with x, y, z attributes
                return (mm_to_meters(position.x), mm_to_meters(position.y), mm_to_meters(position.z))
        except:
            return None