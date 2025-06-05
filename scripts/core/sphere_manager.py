import omni.usd
from pxr import Gf, UsdGeom, Sdf
from isaac_utils import get_position, get_valid_prim

class SphereManager:
    def __init__(self, sphere_path="/World/Sphere_Tracker/tracked_sphere", 
                 radius=0.025, color=[0, 0, 1], default_position=(0, 0, 1.2)):
        self.sphere_path = sphere_path
        self.sphere_radius = radius
        self.sphere_color = color
        self.default_position = default_position
        self.sphere_prim = None
        self.last_position = None
        
    def create_sphere(self, position=None):
        """Create the tracking sphere"""
        if position is None:
            position = self.default_position
        
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                print("No USD stage available")
                return False
            
            # Create parent group if needed
            parent_path = "/World/Sphere_Tracker"
            parent_prim = stage.GetPrimAtPath(parent_path)
            if not parent_prim or not parent_prim.IsValid():
                UsdGeom.Xform.Define(stage, parent_path)
            
            # Remove existing sphere
            self.remove_sphere()
            
            # Create new sphere
            self.sphere_prim = UsdGeom.Sphere.Define(stage, Sdf.Path(self.sphere_path))
            if not self.sphere_prim:
                print("Failed to create sphere")
                return False
            
            # Set sphere properties
            self.sphere_prim.CreateRadiusAttr().Set(self.sphere_radius)
            self.sphere_prim.CreateDisplayColorAttr().Set([Gf.Vec3f(*self.sphere_color)])
            
            # Set initial position
            prim = self.sphere_prim.GetPrim()
            xformable = UsdGeom.Xformable(prim)
            xformable.ClearXformOpOrder()
            translate_op = xformable.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(*position))
            
            self.last_position = None
            print(f"Sphere created at position: {position}")
            return True
            
        except Exception as e:
            print(f"Error creating sphere: {e}")
            return False
    
    def get_position(self):
        """Get current sphere position"""
        return get_position(self.sphere_path)
    
    def move_to(self, x, y, z):
        """Manually move the sphere to a specific position"""
        prim = get_valid_prim(self.sphere_path)
        if not prim:
            print("No sphere found")
            return False
        
        try:
            xformable = UsdGeom.Xformable(prim)
            translate_ops = xformable.GetOrderedXformOps()
            
            for op in translate_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(x, y, z))
                    print(f"Moved sphere to: ({x:.3f}, {y:.3f}, {z:.3f})")
                    return True
                    
        except Exception as e:
            print(f"Error moving sphere: {e}")
        
        return False
    
    def remove_sphere(self):
        """Remove the tracking sphere"""
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return False
            
            prim = stage.GetPrimAtPath(self.sphere_path)
            if prim and prim.IsValid():
                stage.RemovePrim(self.sphere_path)
            
            self.last_position = None
            return True
            
        except Exception as e:
            print(f"Error removing sphere: {e}")
            return False
    
    def has_moved_enough(self, current_pos, threshold=0.005):
        """Check if sphere moved beyond threshold"""
        if self.last_position is None:
            return True
        
        dx = current_pos[0] - self.last_position[0]
        dy = current_pos[1] - self.last_position[1]
        dz = current_pos[2] - self.last_position[2]
        distance = (dx*dx + dy*dy + dz*dz)**0.5
        
        return distance > threshold
    
    def update_last_position(self, position):
        """Update the last known position"""
        self.last_position = position