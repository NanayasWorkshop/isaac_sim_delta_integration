import omni.usd
from pxr import UsdGeom

def get_stage():
    """Get the current USD stage"""
    return omni.usd.get_context().get_stage()

def check_stage_available():
    """Check if USD stage is available"""
    stage = get_stage()
    return stage is not None

def get_prim_at_path(path):
    """Get prim at specified path"""
    stage = get_stage()
    if not stage:
        return None
    return stage.GetPrimAtPath(path)

def is_prim_valid(prim):
    """Check if prim is valid"""
    return prim and prim.IsValid()

def get_world_transform(prim_path):
    """Get world transform for prim at path"""
    try:
        stage = get_stage()
        if not stage:
            return None
        
        prim = stage.GetPrimAtPath(prim_path)
        if not is_prim_valid(prim):
            return None
            
        xformable = UsdGeom.Xformable(prim)
        return xformable.ComputeLocalToWorldTransform(0)
        
    except Exception:
        return None

def get_world_translation(prim_path):
    """Get world translation for prim at path"""
    transform = get_world_transform(prim_path)
    if transform is None:
        return None
    
    translation = transform.ExtractTranslation()
    return (float(translation[0]), float(translation[1]), float(translation[2]))

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
    try:
        stage = get_stage()
        if not stage:
            return False
        
        prim = stage.GetPrimAtPath(prim_path)
        if not is_prim_valid(prim):
            return False
        
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
    stage = get_stage()
    if not stage:
        print("No stage available")
        return
    
    prim = stage.GetPrimAtPath(prim_path)
    if not is_prim_valid(prim):
        print(f"Prim at {prim_path} is not valid")
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
    stage = get_stage()
    if not stage:
        return []
    
    prim = stage.GetPrimAtPath(prim_path)
    if not is_prim_valid(prim):
        return []
    
    children = []
    for child in prim.GetChildren():
        children.append(str(child.GetPath()))
    
    return children
