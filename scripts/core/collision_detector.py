#!/usr/bin/env python3
"""
Collision Detection Module for Isaac Sim Scene
Extracts collision data from Isaac Sim scene for C++ FABRIK integration
Phase 1: Spheres, boxes, and cylinders with full rotation support
"""

import math
from pxr import UsdGeom, Gf
from isaac_utils import get_stage, get_valid_prim, get_position
from math_utils import meters_to_mm

class CollisionDetector:
    def __init__(self, obstacles_path="/World/Obstacles"):
        """Initialize collision detector"""
        self.obstacles_path = obstacles_path
        self.collision_data = {
            'spheres': [],
            'boxes': [],
            'cylinders': [],
            'sphere_count': 0,
            'box_count': 0,
            'cylinder_count': 0
        }
        print(f"Collision detector initialized for path: {obstacles_path}")
    
    def scan_obstacles(self):
        """Scan /World/Obstacles directory for collision primitives"""
        sphere_prims = []
        box_prims = []
        cylinder_prims = []
        
        try:
            obstacles_prim = get_valid_prim(self.obstacles_path)
            if not obstacles_prim:
                return sphere_prims, box_prims, cylinder_prims
            
            # Iterate through direct children only
            for child in obstacles_prim.GetChildren():
                prim_type = child.GetTypeName()
                if prim_type == "Sphere":
                    sphere_prims.append(child)
                elif prim_type == "Cube":
                    box_prims.append(child)
                elif prim_type == "Cylinder":
                    cylinder_prims.append(child)
            
        except Exception as e:
            print(f"Error scanning obstacles: {e}")
        
        return sphere_prims, box_prims, cylinder_prims
    
    def extract_transform_data(self, prim):
        """
        Extract position and rotation from prim
        Returns: dict with 'position_mm', 'rotation_matrix', 'euler_degrees' or None
        """
        try:
            prim_path = str(prim.GetPath())
            
            # Get world transform matrix
            xformable = UsdGeom.Xformable(prim)
            world_transform = xformable.ComputeLocalToWorldTransform(0)
            
            # Extract translation
            translation = world_transform.ExtractTranslation()
            position_mm = (
                meters_to_mm(translation[0]),
                meters_to_mm(translation[1]),
                meters_to_mm(translation[2])
            )
            
            # Extract rotation matrix (3x3)
            rotation_matrix = world_transform.ExtractRotationMatrix()
            
            # Convert to 3x3 matrix format for C++
            rot_matrix_3x3 = [
                [rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2]],
                [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2]],
                [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]]
            ]
            
            # Extract quaternion for C++ (much better than Euler)
            rotation_quat = world_transform.ExtractRotationQuat()
            try:
                # Extract quaternion components in standard format (x, y, z, w)
                if hasattr(rotation_quat, 'real'):
                    # USD Quatd format
                    quat_xyzw = (rotation_quat.real[0], rotation_quat.real[1], rotation_quat.real[2], rotation_quat.real[3])
                elif hasattr(rotation_quat, 'GetReal'):
                    # Alternative USD format
                    w = rotation_quat.GetReal()
                    imaginary = rotation_quat.GetImaginary()
                    quat_xyzw = (imaginary[0], imaginary[1], imaginary[2], w)
                else:
                    # Try direct access (x, y, z, w)
                    quat_xyzw = (rotation_quat[0], rotation_quat[1], rotation_quat[2], rotation_quat[3])
            except:
                # Fallback: identity quaternion
                quat_xyzw = (0.0, 0.0, 0.0, 1.0)
            
            return {
                'position_mm': position_mm,
                'rotation_matrix': rot_matrix_3x3,
                'quaternion': quat_xyzw  # (x, y, z, w) format for C++
            }
            
        except Exception as e:
            print(f"Error extracting transform data: {e}")
            return None
    
    def extract_sphere_data(self, sphere_prim):
        """Extract sphere collision data (center + radius + rotation)"""
        try:
            # Get transform data
            transform_data = self.extract_transform_data(sphere_prim)
            if not transform_data:
                return None
            
            # Get radius from USD sphere primitive
            sphere_geom = UsdGeom.Sphere(sphere_prim)
            radius_attr = sphere_geom.GetRadiusAttr()
            if not radius_attr:
                return None
            
            radius_meters = radius_attr.Get()
            if radius_meters is None:
                return None
            
            # For spheres, check if there's non-uniform scaling that would make it an ellipsoid
            try:
                xformable = UsdGeom.Xformable(sphere_prim)
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                scale = world_transform.ExtractScale()
                
                # Apply maximum scale to radius (conservative approach)
                max_scale = max(scale[0], scale[1], scale[2])
                actual_radius = radius_meters * max_scale
                
                # Check if scaling is non-uniform (would need ellipsoid representation)
                is_uniform = abs(scale[0] - scale[1]) < 0.001 and abs(scale[1] - scale[2]) < 0.001
                
            except:
                actual_radius = radius_meters
                is_uniform = True
            
            return {
                'center_mm': transform_data['position_mm'],
                'radius_mm': meters_to_mm(actual_radius),
                'rotation_matrix': transform_data['rotation_matrix'],
                'quaternion': transform_data['quaternion'],
                'is_uniform_scale': is_uniform
            }
            
        except Exception as e:
            print(f"Error extracting sphere data: {e}")
            return None
    
    def extract_box_data(self, box_prim):
        """Extract box collision data (center + dimensions + rotation) - OBB support"""
        try:
            # Get transform data
            transform_data = self.extract_transform_data(box_prim)
            if not transform_data:
                return None
            
            # Get size from USD cube primitive
            cube_geom = UsdGeom.Cube(box_prim)
            size_attr = cube_geom.GetSizeAttr()
            if not size_attr:
                return None
            
            size_meters = size_attr.Get()
            if size_meters is None:
                return None
            
            # Apply scale transforms to get actual dimensions
            try:
                xformable = UsdGeom.Xformable(box_prim)
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                scale = world_transform.ExtractScale()
                
                # Apply scale to base size for each dimension
                actual_size = (
                    size_meters * scale[0],  # Width (X)
                    size_meters * scale[1],  # Height (Y) 
                    size_meters * scale[2]   # Depth (Z)
                )
            except:
                # Fallback: uniform size
                actual_size = (size_meters, size_meters, size_meters)
            
            # Check if rotation is identity (can use AABB optimization)
            is_axis_aligned = self._is_rotation_identity(transform_data['rotation_matrix'])
            
            return {
                'center_mm': transform_data['position_mm'],
                'size_mm': (
                    meters_to_mm(actual_size[0]),
                    meters_to_mm(actual_size[1]),
                    meters_to_mm(actual_size[2])
                ),
                'rotation_matrix': transform_data['rotation_matrix'],
                'quaternion': transform_data['quaternion'],
                'is_axis_aligned': is_axis_aligned  # True = can use AABB, False = needs OBB
            }
            
        except Exception as e:
            print(f"Error extracting box data: {e}")
            return None
    
    def extract_cylinder_data(self, cylinder_prim):
        """
        Extract cylinder collision data (center + radius + height + rotation)
        Returns: dict with 'center_mm', 'radius_mm', 'height_mm', 'rotation_matrix' or None if failed
        """
        try:
            # Get transform data
            transform_data = self.extract_transform_data(cylinder_prim)
            if not transform_data:
                return None
            
            cylinder_geom = UsdGeom.Cylinder(cylinder_prim)
            
            # Get radius
            radius_attr = cylinder_geom.GetRadiusAttr()
            if not radius_attr:
                return None
            radius_meters = radius_attr.Get()
            if radius_meters is None:
                return None
            
            # Get height
            height_attr = cylinder_geom.GetHeightAttr()
            if not height_attr:
                return None
            height_meters = height_attr.Get()
            if height_meters is None:
                return None
            
            # Check for scale transforms
            try:
                xformable = UsdGeom.Xformable(cylinder_prim)
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                scale = world_transform.ExtractScale()
                
                # Apply scale to radius and height
                actual_radius = radius_meters * max(scale[0], scale[1])  # XY scale for radius
                actual_height = height_meters * scale[2]  # Z scale for height
                
                # Check if XY scaling is uniform for radius
                is_circular = abs(scale[0] - scale[1]) < 0.001
                
            except:
                actual_radius = radius_meters
                actual_height = height_meters
                is_circular = True
            
            # Check if cylinder is axis-aligned (Z-up)
            is_axis_aligned = self._is_cylinder_axis_aligned(transform_data['rotation_matrix'])
            
            return {
                'center_mm': transform_data['position_mm'],
                'radius_mm': meters_to_mm(actual_radius),
                'height_mm': meters_to_mm(actual_height),
                'rotation_matrix': transform_data['rotation_matrix'],
                'quaternion': transform_data['quaternion'],
                'is_circular': is_circular,  # True = circular, False = elliptical
                'is_axis_aligned': is_axis_aligned  # True = Z-aligned, False = rotated
            }
            
        except Exception as e:
            print(f"Error extracting cylinder data: {e}")
            return None
    
    def _is_rotation_identity(self, rotation_matrix, tolerance=0.001):
        """Check if rotation matrix is approximately identity (for AABB optimization)"""
        try:
            identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            for i in range(3):
                for j in range(3):
                    if abs(rotation_matrix[i][j] - identity[i][j]) > tolerance:
                        return False
            return True
        except:
            return False
    
    def _is_cylinder_axis_aligned(self, rotation_matrix, tolerance=0.001):
        """Check if cylinder is Z-axis aligned (for optimization)"""
        try:
            # Check if Z-axis [0,0,1] is still pointing up after rotation
            z_axis_rotated = [rotation_matrix[0][2], rotation_matrix[1][2], rotation_matrix[2][2]]
            return (abs(z_axis_rotated[0]) < tolerance and 
                    abs(z_axis_rotated[1]) < tolerance and 
                    abs(z_axis_rotated[2] - 1.0) < tolerance)
        except:
            return False
    
    def update_collision_data(self):
        """Update collision data by rescanning obstacles (event-driven)"""
        try:
            sphere_prims, box_prims, cylinder_prims = self.scan_obstacles()
            
            # Clear and rebuild collision data
            self.collision_data['spheres'].clear()
            self.collision_data['boxes'].clear()
            self.collision_data['cylinders'].clear()
            
            # Extract sphere data
            for sphere_prim in sphere_prims:
                sphere_data = self.extract_sphere_data(sphere_prim)
                if sphere_data:
                    self.collision_data['spheres'].append(sphere_data)
            
            # Extract box data
            for box_prim in box_prims:
                box_data = self.extract_box_data(box_prim)
                if box_data:
                    self.collision_data['boxes'].append(box_data)
            
            # Extract cylinder data
            for cylinder_prim in cylinder_prims:
                cylinder_data = self.extract_cylinder_data(cylinder_prim)
                if cylinder_data:
                    self.collision_data['cylinders'].append(cylinder_data)
            
            # Update counts
            self.collision_data['sphere_count'] = len(self.collision_data['spheres'])
            self.collision_data['box_count'] = len(self.collision_data['boxes'])
            self.collision_data['cylinder_count'] = len(self.collision_data['cylinders'])
            
            return True
            
        except Exception as e:
            print(f"Error updating collision data: {e}")
            return False
    
    def get_collision_data_for_cpp(self):
        """Get collision data formatted for C++ FABRIK handoff with rotation support"""
        try:
            cpp_data = {
                'spheres': [],
                'sphere_count': self.collision_data['sphere_count'],
                'boxes': [],
                'box_count': self.collision_data['box_count'],
                'cylinders': [],
                'cylinder_count': self.collision_data['cylinder_count']
            }
            
            # Format sphere data for C++
            for sphere in self.collision_data['spheres']:
                cpp_sphere = {
                    'center': sphere['center_mm'],  # (x, y, z) in mm
                    'radius': sphere['radius_mm'],  # radius in mm
                    'rotation_matrix': sphere['rotation_matrix'],  # 3x3 matrix
                    'quaternion': sphere['quaternion'],  # (x, y, z, w) quaternion
                    'is_uniform_scale': sphere['is_uniform_scale']
                }
                cpp_data['spheres'].append(cpp_sphere)
            
            # Format box data for C++
            for box in self.collision_data['boxes']:
                cpp_box = {
                    'center': box['center_mm'],  # (x, y, z) in mm
                    'size': box['size_mm'],      # (width, height, depth) in mm
                    'rotation_matrix': box['rotation_matrix'],  # 3x3 matrix
                    'quaternion': box['quaternion'],  # (x, y, z, w) quaternion
                    'is_axis_aligned': box['is_axis_aligned']  # AABB vs OBB optimization
                }
                cpp_data['boxes'].append(cpp_box)
            
            # Format cylinder data for C++
            for cylinder in self.collision_data['cylinders']:
                cpp_cylinder = {
                    'center': cylinder['center_mm'],  # (x, y, z) in mm
                    'radius': cylinder['radius_mm'],  # radius in mm
                    'height': cylinder['height_mm'],  # height in mm
                    'rotation_matrix': cylinder['rotation_matrix'],  # 3x3 matrix
                    'quaternion': cylinder['quaternion'],  # (x, y, z, w) quaternion
                    'is_circular': cylinder['is_circular'],  # circular vs elliptical
                    'is_axis_aligned': cylinder['is_axis_aligned']  # Z-aligned optimization
                }
                cpp_data['cylinders'].append(cpp_cylinder)
            
            return cpp_data
            
        except Exception as e:
            print(f"Error formatting collision data for C++: {e}")
            return None