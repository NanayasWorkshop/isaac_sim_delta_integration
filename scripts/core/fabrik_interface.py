import sys
import numpy as np

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
    
    def calculate_motors(self, target_x_m, target_y_m, target_z_m, current_j_points=None):
        """Calculate motor positions for target (input in meters, converts to mm internally)"""
        if not self.initialized:
            print("FABRIK not initialized")
            return None
        
        try:
            # Convert position to millimeters for FABRIK
            target_x_mm = target_x_m * 1000  # m to mm
            target_y_mm = target_y_m * 1000
            target_z_mm = target_z_m * 1000
            
            # Choose FABRIK method based on J points availability
            if current_j_points is not None and len(current_j_points) > 0:
                # Convert J points to FABRIK format (m to mm)
                fabrik_j_points = self._convert_j_points_to_fabrik_format(current_j_points)
                if fabrik_j_points is not None:
                    print(f"Target: ({target_x_mm:.1f}, {target_y_mm:.1f}, {target_z_mm:.1f})mm - FABRIK: From J points")
                    result = self.delta_robot.motor.MotorModule.calculate_motors(
                        target_x_mm, target_y_mm, target_z_mm, fabrik_j_points
                    )
                else:
                    print(f"Target: ({target_x_mm:.1f}, {target_y_mm:.1f}, {target_z_mm:.1f})mm - FABRIK: J points invalid, straight-up")
                    result = self.delta_robot.motor.MotorModule.calculate_motors(target_x_mm, target_y_mm, target_z_mm)
            else:
                # Use straight-up initialization
                print(f"Target: ({target_x_mm:.1f}, {target_y_mm:.1f}, {target_z_mm:.1f})mm - FABRIK: Straight-up")
                result = self.delta_robot.motor.MotorModule.calculate_motors(target_x_mm, target_y_mm, target_z_mm)
            
            if result:
                print(f"FABRIK converged: {result.fabrik_converged}")
            
            return result
            
        except Exception as e:
            print(f"Error calculating motors: {e}")
            return None
    
    def _convert_j_points_to_fabrik_format(self, j_points):
        """Convert J points from meters to millimeters for FABRIK C++"""
        try:
            fabrik_points = []
            for i, j_point in enumerate(j_points):
                if hasattr(j_point, 'shape'):  # numpy array
                    # Convert m to mm
                    point_mm = np.array([
                        float(j_point[0]) * 1000.0,
                        float(j_point[1]) * 1000.0,
                        float(j_point[2]) * 1000.0
                    ])
                    fabrik_points.append(point_mm)
                elif hasattr(j_point, '__len__') and len(j_point) == 3:  # tuple/list
                    # Convert m to mm
                    point_mm = np.array([
                        float(j_point[0]) * 1000.0,
                        float(j_point[1]) * 1000.0,
                        float(j_point[2]) * 1000.0
                    ])
                    fabrik_points.append(point_mm)
                else:
                    print(f"Warning: J point {i} has unexpected format")
                    return None
            
            # Ensure base is at origin for FABRIK
            if len(fabrik_points) > 0:
                base_distance = np.linalg.norm(fabrik_points[0])
                if base_distance > 1.0:  # 1mm tolerance
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
                    if hasattr(joint_pos, 'shape'):  # numpy array
                        pos_m = (joint_pos[0] / 1000.0, joint_pos[1] / 1000.0, joint_pos[2] / 1000.0)
                        fabrik_joints.append(pos_m)
                    else:
                        try:
                            pos_m = (joint_pos.x / 1000.0, joint_pos.y / 1000.0, joint_pos.z / 1000.0)
                            fabrik_joints.append(pos_m)
                        except:
                            pass
            
            # Extract segment end-effector positions (convert from mm to m)
            if hasattr(motor_result, 'original_segment_positions'):
                base_pos = (0, 0, 0)
                segment_end_effectors.append(base_pos)
                
                for seg_pos in motor_result.original_segment_positions:
                    if hasattr(seg_pos, 'shape'):  # numpy array
                        pos_m = (seg_pos[0] / 1000.0, seg_pos[1] / 1000.0, seg_pos[2] / 1000.0)
                        segment_end_effectors.append(pos_m)
                    else:
                        try:
                            pos_m = (seg_pos.x / 1000.0, seg_pos.y / 1000.0, seg_pos.z / 1000.0)
                            segment_end_effectors.append(pos_m)
                        except:
                            pass
            
        except Exception as e:
            print(f"Error extracting FABRIK data: {e}")
        
        return fabrik_joints, segment_end_effectors