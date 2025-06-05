import numpy as np

class RobotController:
    def __init__(self, robot_path, name="sphere_following_robot"):
        """
        Initialize robot controller
        robot_path: Path to the robot in the USD scene (no default - must be provided)
        name: Name for the robot view
        """
        self.robot_path = robot_path
        self.robot_name = name
        self.robot_view = None
        self.initialized = False
        
    def initialize(self):
        """Initialize the delta robot"""
        try:
            from omni.isaac.core.articulations import ArticulationView
            
            self.robot_view = ArticulationView(
                prim_paths_expr=self.robot_path, 
                name=self.robot_name
            )
            self.robot_view.initialize()
            
            print(f"Robot initialized at path: {self.robot_path}")
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"Robot setup failed: {e}")
            return False
    
    def is_initialized(self):
        """Check if robot is initialized"""
        return self.initialized
    
    def get_dof_count(self):
        """Get number of degrees of freedom"""
        if not self.initialized or not self.robot_view:
            return 0
        return len(self.robot_view.dof_names)
    
    def get_dof_names(self):
        """Get DOF names"""
        if not self.initialized or not self.robot_view:
            return []
        return self.robot_view.dof_names
    
    def get_max_segments(self):
        """Calculate maximum number of segments based on robot DOF count"""
        dof_count = self.get_dof_count()
        if dof_count == 0:
            return 0
        # Assuming 6 DOF per segment
        return dof_count // 6
    
    def apply_joint_targets(self, joint_targets):
        """Apply joint position targets to the robot"""
        if not self.initialized or not self.robot_view:
            print("Robot not initialized")
            return False
        
        try:
            self.robot_view.set_joint_position_targets(joint_targets)
            self.robot_view.set_joint_positions(joint_targets)
            return True
        except Exception as e:
            print(f"Error applying joint targets: {e}")
            return False
    
    def convert_fabrik_to_joint_targets(self, fabrik_result):
        """Convert FABRIK result to joint targets for delta robot"""
        try:
            joint_targets = np.zeros(self.get_dof_count())
            
            # Calculate maximum segments we can handle based on robot DOF
            max_segments = self.get_max_segments()
            
            # Process only as many segments as the robot supports and FABRIK provides
            segments_to_process = min(max_segments, len(fabrik_result.levels))
            
            print(f"Robot DOF: {self.get_dof_count()}, Max segments: {max_segments}, "
                  f"FABRIK levels: {len(fabrik_result.levels)}, Processing: {segments_to_process}")
            
            for level_idx in range(segments_to_process):
                level = fabrik_result.levels[level_idx]
                base_idx = level_idx * 6
                
                if base_idx + 5 < len(joint_targets):
                    joint_targets[base_idx + 0] = np.deg2rad(level.roll_joint)
                    joint_targets[base_idx + 1] = np.deg2rad(level.pitch_joint)
                    joint_targets[base_idx + 2] = level.prismatic_joint
                    joint_targets[base_idx + 3] = level.prismatic_joint
                    joint_targets[base_idx + 4] = np.deg2rad(level.pitch_joint)
                    joint_targets[base_idx + 5] = np.deg2rad(level.roll_joint)
                else:
                    print(f"Warning: Segment {level_idx} would exceed joint array bounds")
                    break
            
            return joint_targets
            
        except Exception as e:
            print(f"Error converting FABRIK to joint targets: {e}")
            return None
    
    def move_to_target(self, fabrik_result):
        """Move robot using FABRIK result"""
        joint_targets = self.convert_fabrik_to_joint_targets(fabrik_result)
        if joint_targets is None:
            return False
        
        return self.apply_joint_targets(joint_targets)