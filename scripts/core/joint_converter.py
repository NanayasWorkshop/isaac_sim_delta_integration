import numpy as np
from scipy.optimize import minimize_scalar

class JointConverter:
    """
    Integrate Isaac Sim connection points with Joint Optimizer
    Takes P points from Isaac Sim and calculates J points using optimization
    """
    def __init__(self):
        self.points = []
        self.joints = []
        
    def load_isaac_connection_points(self, connection_points):
        """
        Load connection points from Isaac Sim ConnectionPointExtractor
        connection_points: list of tuples [(name, (x, y, z)), ...]
        """
        self.points = []
        
        for name, point in connection_points:
            # Convert to numpy array for calculations
            point_array = np.array([point[0], point[1], point[2]])
            self.points.append(point_array)
            print(f"Added {name}: ({point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f})")
        
        print(f"Loaded {len(self.points)} P points from Isaac Sim")
        return len(self.points)
    
    def load_points_from_coordinates(self, coordinates):
        """
        Load P points from coordinate list
        coordinates: list of (x, y, z) tuples
        """
        self.points = []
        
        for i, (x, y, z) in enumerate(coordinates):
            point_array = np.array([x, y, z])
            self.points.append(point_array)
            print(f"P{i+1}: ({x:.6f}, {y:.6f}, {z:.6f})")
        
        print(f"Loaded {len(self.points)} P points")
        return len(self.points)
    
    def angle_between_vectors(self, v1, v2):
        """Calculate angle between two vectors in degrees"""
        dot_product = np.dot(v1, v2)
        norms = np.linalg.norm(v1) * np.linalg.norm(v2)
        if norms == 0:
            return 0
        return np.arccos(np.clip(dot_product / norms, -1, 1)) * 180 / np.pi
    
    def optimize_first_joint(self, bounds=(-0.5, 0.5)):
        """Optimize J1 position on Z-axis for first triangle (P1-J1-P2)"""
        if len(self.points) < 2:
            raise ValueError("Need at least 2 points to optimize first joint")
            
        p1, p2 = self.points[0], self.points[1]
        
        def angle_difference(z):
            j1 = np.array([0, 0, z])
            
            # Vectors for angle calculation
            p1_j1 = j1 - p1
            p1_p2 = p2 - p1
            p2_j1 = j1 - p2
            p2_p1 = p1 - p2
            
            angle1 = self.angle_between_vectors(p1_j1, p1_p2)
            angle2 = self.angle_between_vectors(p2_j1, p2_p1)
            
            return abs(angle1 - angle2)
        
        result = minimize_scalar(angle_difference, bounds=bounds, method='bounded')
        j1 = np.array([0, 0, result.x])
        self.joints.append(j1)
        return j1
    
    def optimize_joint(self, joint_index, bounds=(-2, 2)):
        """Optimize joint position along the line from previous joint through current pivot point"""
        if joint_index == 0:
            return self.optimize_first_joint()
            
        if len(self.points) < joint_index + 2:
            raise ValueError(f"Need at least {joint_index + 2} points to optimize joint {joint_index + 1}")
        
        # Get the relevant points
        p_prev = self.points[joint_index]      # Previous point (pivot of current triangle)
        p_curr = self.points[joint_index + 1]  # Current point (base vertex 1)
        j_prev = self.joints[joint_index - 1]  # Previous joint
        
        # Direction from previous joint through previous point
        direction = p_prev - j_prev
        direction_unit = direction / np.linalg.norm(direction)
        
        def angle_difference(t):
            # Joint position along the line
            j_curr = j_prev + t * direction_unit
            
            # Vectors from current point
            p_curr_j = j_curr - p_curr
            p_curr_p_prev = p_prev - p_curr
            
            # Vectors from previous point  
            p_prev_j = j_curr - p_prev
            p_prev_p_curr = p_curr - p_prev
            
            angle1 = self.angle_between_vectors(p_curr_j, p_curr_p_prev)
            angle2 = self.angle_between_vectors(p_prev_j, p_prev_p_curr)
            
            return abs(angle1 - angle2)
        
        result = minimize_scalar(angle_difference, bounds=bounds, method='bounded')
        j_curr = j_prev + result.x * direction_unit
        self.joints.append(j_curr)
        return j_curr
    
    def calculate_j_points_from_isaac_p_points(self):
        """
        Main function: Calculate J points from Isaac Sim P points
        Returns list of J point coordinates
        """
        if len(self.points) < 2:
            print("Need at least 2 P points to calculate J points")
            return []
        
        self.joints = []
        
        print(f"\n=== Calculating J points from {len(self.points)} P points ===")
        
        # Calculate (n-1) joints for n points using optimization
        for i in range(len(self.points) - 1):
            joint = self.optimize_joint(i)
            print(f"J{i+1} position: ({joint[0]:.6f}, {joint[1]:.6f}, {joint[2]:.6f})")
        
        # Add final J point equal to last P point (end effector)
        final_j_point = self.points[-1].copy()  # J_n = P_n
        self.joints.append(final_j_point)
        print(f"J{len(self.joints)} position: ({final_j_point[0]:.6f}, {final_j_point[1]:.6f}, {final_j_point[2]:.6f}) [= P{len(self.points)}]")
        
        print(f"\nCalculated {len(self.joints)} J points (including end effector)")
        return self.joints.copy()
    
    def get_j_points_as_tuples(self):
        """Return J points as list of (x, y, z) tuples"""
        return [(float(j[0]), float(j[1]), float(j[2])) for j in self.joints]
    
    def print_p_and_j_summary(self):
        """Print summary of both P and J points"""
        print(f"\n=== P and J Points Summary ===")
        print(f"P points (Isaac Sim connection points): {len(self.points)}")
        for i, p in enumerate(self.points):
            print(f"  P{i+1}: ({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f})")
        
        print(f"\nJ points (Optimized joints): {len(self.joints)}")
        for i, j in enumerate(self.joints):
            if i == len(self.joints) - 1:
                # Last J point equals last P point
                print(f"  J{i+1}: ({j[0]:.6f}, {j[1]:.6f}, {j[2]:.6f}) [= P{len(self.points)}]")
            else:
                print(f"  J{i+1}: ({j[0]:.6f}, {j[1]:.6f}, {j[2]:.6f})")