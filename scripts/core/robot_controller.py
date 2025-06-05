import numpy as np

class RobotController:
    def __init__(self, robot_path="/World/delta_robot_3_00", name="sphere_following_robot"):
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
            
            print("Robot initialized")
            self.initialized = True
            
            # Debug DOF information
            self.debug_dof_info()
            
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
    
    def debug_dof_info(self):
        """Print detailed DOF information"""
        dof_count = self.get_dof_count()
        dof_names = self.get_dof_names()
        
        print(f"\n=== ROBOT DOF DEBUG INFO ===")
        print(f"Total DOF Count: {dof_count}")
        print(f"DOF Names ({len(dof_names)}):")
        
        for i, name in enumerate(dof_names):
            print(f"  [{i:2d}] {name}")
        
        # Analyze DOF pattern
        segments = {}
        for i, name in enumerate(dof_names):
            if 'seg' in name.lower():
                parts = name.split('_')
                if len(parts) >= 2:
                    seg_num = parts[0]  # e.g., 'seg1', 'seg2'
                    if seg_num not in segments:
                        segments[seg_num] = []
                    segments[seg_num].append((i, name))
        
        print(f"\nDetected Segments:")
        for seg_name in sorted(segments.keys()):
            joints = segments[seg_name]
            print(f"  {seg_name}: {len(joints)} joints")
            for idx, joint_name in joints:
                print(f"    [{idx:2d}] {joint_name}")
        
        return dof_count, dof_names, segments
    
    def apply_joint_targets(self, joint_targets):
        """Apply joint position targets to the robot"""
        if not self.initialized or not self.robot_view:
            print("Robot not initialized")
            return False
        
        try:
            expected_dof = self.get_dof_count()
            if len(joint_targets) != expected_dof:
                print(f"⚠️  Joint target mismatch: expected {expected_dof}, got {len(joint_targets)}")
                # Resize array to match expected DOF
                if len(joint_targets) < expected_dof:
                    joint_targets = np.pad(joint_targets, (0, expected_dof - len(joint_targets)))
                else:
                    joint_targets = joint_targets[:expected_dof]
            
            # Print some debug info
            print(f"Applying {len(joint_targets)} joint targets")
            for i, target in enumerate(joint_targets):
                if abs(target) > 0.001:  # Only print non-zero targets
                    dof_name = self.get_dof_names()[i] if i < len(self.get_dof_names()) else f"DOF_{i}"
                    print(f"  {dof_name}: {target:.3f}")
            
            self.robot_view.set_joint_position_targets(joint_targets)
            self.robot_view.set_joint_positions(joint_targets)
            return True
            
        except Exception as e:
            print(f"Error applying joint targets: {e}")
            return False
    
    def convert_fabrik_to_joint_targets(self, fabrik_result):
        """Convert FABRIK result to joint targets for delta robot"""
        try:
            total_dof = self.get_dof_count()
            joint_targets = np.zeros(total_dof)
            
            print(f"\n=== FABRIK TO JOINT CONVERSION ===")
            print(f"Total DOF: {total_dof}")
            print(f"FABRIK levels: {len(fabrik_result.levels) if hasattr(fabrik_result, 'levels') else 'N/A'}")
            
            # More flexible mapping - detect actual number of segments
            if hasattr(fabrik_result, 'levels'):
                num_levels = len(fabrik_result.levels)
                joints_per_level = 6  # Assumed pattern
                
                for level_idx in range(min(num_levels, total_dof // joints_per_level)):
                    level = fabrik_result.levels[level_idx]
                    base_idx = level_idx * joints_per_level
                    
                    # Ensure we don't exceed the DOF array
                    if base_idx + 5 < total_dof:
                        joint_targets[base_idx + 0] = np.deg2rad(level.roll_joint)
                        joint_targets[base_idx + 1] = np.deg2rad(level.pitch_joint)
                        joint_targets[base_idx + 2] = level.prismatic_joint
                        joint_targets[base_idx + 3] = level.prismatic_joint
                        joint_targets[base_idx + 4] = np.deg2rad(level.pitch_joint)
                        joint_targets[base_idx + 5] = np.deg2rad(level.roll_joint)
                        
                        print(f"Level {level_idx}: DOF {base_idx}-{base_idx+5}")
                        print(f"  Roll: {level.roll_joint:.2f}°")
                        print(f"  Pitch: {level.pitch_joint:.2f}°")
                        print(f"  Prismatic: {level.prismatic_joint:.4f}")
                    else:
                        print(f"⚠️  Level {level_idx} would exceed DOF array (base_idx={base_idx}, total_dof={total_dof})")
            
            # Check if we're using all DOF
            used_dof = 0
            for target in joint_targets:
                if abs(target) > 0.001:
                    used_dof += 1
            
            print(f"Used DOF: {used_dof}/{total_dof}")
            if used_dof < total_dof:
                print(f"⚠️  {total_dof - used_dof} DOF are not being actuated")
            
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