import math

def distance_3d(pos1, pos2):
    """Calculate 3D distance between two positions"""
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def meters_to_mm(meters):
    """Convert meters to millimeters"""
    return meters * 1000.0

def mm_to_meters(millimeters):
    """Convert millimeters to meters"""
    return millimeters / 1000.0

def position_m_to_mm(position_m):
    """Convert position tuple from meters to millimeters"""
    return (position_m[0] * 1000.0, position_m[1] * 1000.0, position_m[2] * 1000.0)

def position_mm_to_m(position_mm):
    """Convert position tuple from millimeters to meters"""
    return (position_mm[0] / 1000.0, position_mm[1] / 1000.0, position_mm[2] / 1000.0)

def degrees_to_radians(degrees):
    """Convert degrees to radians"""
    return math.radians(degrees)

def radians_to_degrees(radians):
    """Convert radians to degrees"""
    return math.degrees(radians)

def clamp(value, min_val, max_val):
    """Clamp value between min and max"""
    return max(min_val, min(value, max_val))

def interpolate_linear(start, end, t):
    """Linear interpolation between start and end positions"""
    t = clamp(t, 0.0, 1.0)
    return (
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
        start[2] + (end[2] - start[2]) * t
    )

def normalize_vector(vector):
    """Normalize a 3D vector"""
    magnitude = math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
    if magnitude == 0:
        return (0, 0, 0)
    return (vector[0]/magnitude, vector[1]/magnitude, vector[2]/magnitude)

def vector_magnitude(vector):
    """Calculate magnitude of a 3D vector"""
    return math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)

def dot_product(vec1, vec2):
    """Calculate dot product of two 3D vectors"""
    return vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2]

def cross_product(vec1, vec2):
    """Calculate cross product of two 3D vectors"""
    return (
        vec1[1]*vec2[2] - vec1[2]*vec2[1],
        vec1[2]*vec2[0] - vec1[0]*vec2[2],
        vec1[0]*vec2[1] - vec1[1]*vec2[0]
    )