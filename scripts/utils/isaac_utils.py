import omni.usd
from pxr import UsdGeom

def get_stage():
    """Get the current USD stage"""
    return omni.usd.get_context().get_stage()

def check_stage_available():
    """Check if USD stage is available"""
    stage = get_stage()
    return stage is not None

def get_valid_prim(path):
    """Helper function to get valid prim with common error checking"""
    try:
        stage = get_stage()
        if not stage:
            return None
        
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return None
        
        return prim
    except Exception:
        return None

def get_prim_at_path(path):
    """Get prim at specified path"""
    return get_valid_prim(path)

def is_prim_valid(prim):
    """Check if prim is valid"""
    return prim and prim.IsValid()

def get_world_transform(prim_path):
    """Get world transform for prim at path"""
    prim = get_valid_prim(prim_path)
    if not prim:
        return None
    
    try:
        xformable = UsdGeom.Xformable(prim)
        return xformable.ComputeLocalToWorldTransform(0)
    except Exception:
        return None

def get_position(path):
    """Get world position for any prim (unified function)"""
    prim = get_valid_prim(path)
    if not prim:
        return None
    
    try:
        xformable = UsdGeom.Xformable(prim)
        world_transform = xformable.ComputeLocalToWorldTransform(0)
        translation = world_transform.ExtractTranslation()
        return (float(translation[0]), float(translation[1]), float(translation[2]))
    except Exception:
        return None

def create_xform_if_needed(path):
    """Create Xform at path if it doesn't exist"""
    try:
        stage = get_stage()
        if not stage:
            return False
        
        prim = stage.GetPrimAtPath(path)
        if not is_prim_valid(prim):
            UsdGeom.Xform.Define(stage, path)
            return True
        return True
        
    except Exception:
        return False

def remove_prim_if_exists(path):
    """Remove prim at path if it exists"""
    try:
        stage = get_stage()
        if not stage:
            return False
        
        prim = stage.GetPrimAtPath(path)
        if is_prim_valid(prim):
            stage.RemovePrim(path)
        return True
        
    except Exception:
        return False

def set_prim_translation(prim_path, translation):
    """Set translation for prim at path"""
    prim = get_valid_prim(prim_path)
    if not prim:
        return False
    
    try:
        xformable = UsdGeom.Xformable(prim)
        translate_ops = xformable.GetOrderedXformOps()
        
        for op in translate_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                from pxr import Gf
                op.Set(Gf.Vec3d(*translation))
                return True
        
        return False
    except Exception:
        return False

def print_prim_info(prim_path):
    """Print debug information about a prim"""
    prim = get_valid_prim(prim_path)
    if not prim:
        print(f"Prim at {prim_path} is not valid or not found")
        return
    
    print(f"Prim: {prim_path}")
    print(f"  Type: {prim.GetTypeName()}")
    print(f"  Valid: {prim.IsValid()}")
    
    # Try to get transform info
    try:
        xformable = UsdGeom.Xformable(prim)
        transform = xformable.ComputeLocalToWorldTransform(0)
        translation = transform.ExtractTranslation()
        print(f"  Translation: ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})")
    except Exception as e:
        print(f"  Transform error: {e}")

def list_children(prim_path):
    """List children of prim at path"""
    prim = get_valid_prim(prim_path)
    if not prim:
        return []
    
    children = []
    for child in prim.GetChildren():
        children.append(str(child.GetPath()))
    
    return children

# CONSOLIDATED FUNCTIONS FROM connection_points.py and sphere_manager.py

def get_stage_and_robot(robot_path):
    """Get stage and robot prim with error checking"""
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("No USD stage available")
        return None, None
    
    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim or not robot_prim.IsValid():
        print(f"Robot not found at path: {robot_path}")
        return None, None
    
    return stage, robot_prim

def search_for_link(prim, target_name):
    """Recursively search for a link by name"""
    if prim.GetName() == target_name:
        return str(prim.GetPath())
    
    for child in prim.GetChildren():
        result = search_for_link(child, target_name)
        if result:
            return result
    return None

# Alias functions for backward compatibility (these just call get_position now)
def get_link_world_position(link_path):
    """Get world position of a link (calls unified get_position)"""
    return get_position(link_path)

def get_sphere_position(sphere_path):
    """Get current sphere position (calls unified get_position)"""
    return get_position(sphere_path)

def get_world_translation(prim_path):
    """Get world translation for prim at path (calls unified get_position)"""
    return get_position(prim_path)