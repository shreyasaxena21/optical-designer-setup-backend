import math
import json
from flask import Flask, request, jsonify
from flask_cors import CORS

# --- Utility Functions ---

def normalize(x, y): # normalizing the vectors -> v1/|v1|
    """Returns the unit vector direction (x, y)"""
    mag = math.hypot(x, y) # hypotenuse
    if mag == 0:
        return (0, 0)
    return (x / mag, y / mag)

def intersect_line_segment(p1, d, s1, s2, epsilon=1e-3):
    """
    Finds the intersection of a ray (p1, d) with a line segment (s1, s2).
    Returns (ix, iy, t_val) or None.
    - t_val is the distance along the ray (must be > epsilon).
    """
    p1x, p1y = p1
    dx, dy = d
    s1x, s1y = s1
    s2x, s2y = s2
    
    s_dx = s2x - s1x
    s_dy = s2y - s1y
    
    # Calculate denominator (det)
    det = dx * s_dy - dy * s_dx
    
    if abs(det) < 1e-10:
        # Parallel or collinear
        return None 

    # Calculate t (distance along the ray)
    t = ((s1x - p1x) * s_dy - (s1y - p1y) * s_dx) / det
    
    # Calculate u (position along the segment, 0 <= u <= 1)
    u = ((s1x - p1x) * dy - (s1y - p1y) * dx) / det
    
    # Intersection checks - CRITICAL: Use epsilon to avoid self-intersection
    if t > epsilon and 0 <= u <= 1:
        ix = p1x + t * dx
        iy = p1y + t * dy
        return (ix, iy, t)
    
    return None

def reflect_vector(incident, normal):
    """Reflect incident vector across normal vector"""
    ix, iy = incident
    nx, ny = normal
    dot = ix * nx + iy * ny
    rx = ix - 2 * dot * nx
    ry = iy - 2 * dot * ny
    return normalize(rx, ry)

def get_perpendicular_normal(seg_start, seg_end, incoming_dir):
    """
    Get the perpendicular normal to a segment, oriented to face the incoming ray.
    """
    s1x, s1y = seg_start
    s2x, s2y = seg_end
    
    # Segment direction
    seg_dx = s2x - s1x
    seg_dy = s2y - s1y
    
    # Perpendicular (rotate 90 degrees counterclockwise)
    nx, ny = normalize(-seg_dy, seg_dx)
    
    # Ensure normal faces the incoming ray (dot product should be negative)
    if incoming_dir[0] * nx + incoming_dir[1] * ny > 0:
        nx, ny = -nx, -ny
    
    return (nx, ny)

# --- Flask App Setup ---
app = Flask(__name__)
CORS(app, resources={r"/api/*": {"origins": "*"}})

# --- Simulation Route ---
@app.route('/api/simulate', methods=['POST'])
def simulate():
    if not request.is_json:
        return jsonify({"msg": "Missing JSON in request"}), 400
    
    comp_list = request.get_json()

    # 1. Component Pre-processing
    comps = []
    for c in comp_list:
        size = c.get('properties', {}).get('diameter', 50)
        angle_rad = math.radians(c.get('angle', 0))
        half = size / 2
        
        # Segment direction
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        
        # Segment endpoints
        cx, cy = c['x'], c['y']
        p1 = (cx - dx * half, cy - dy * half) 
        p2 = (cx + dx * half, cy + dy * half) 
        
        # Port tracking
        port = c.get('port_number', 0)
        
        comps.append({**c, 'seg': (p1, p2), 'port_number': port})

    rays_out = []
    detectors = {c['id']: 0.0 for c in comps if c['type'] == 'Photo Detector'}
    
    # Statistics tracking
    component_interactions = {c['id']: 0 for c in comps}
    total_reflections = 0
    total_refractions = 0
    total_splits = 0

    # 2. Ray Tracing Loop
    for c in comps:
        if c['type'] != 'Source':
            continue
        
        src_angle = math.radians(c.get('angle', 0))
        intensity_total = c.get('properties', {}).get('intensity', 1.0)
        
        # Ray direction
        d = normalize(math.cos(src_angle), math.sin(src_angle))
        p0 = (c['x'], c['y'])
        
        # Number of rays to trace (for spread if needed)
        spread_deg = c.get('properties', {}).get('spread_deg', 0)
        num_rays = max(1, int(spread_deg / 2)) if spread_deg > 0 else 1
        intensity_per_ray = intensity_total / num_rays

        for ray_idx in range(num_rays):
            # Calculate spread angle
            if num_rays > 1:
                spread_fraction = (ray_idx - (num_rays - 1) / 2) / (num_rays - 1)
                spread_angle = spread_fraction * math.radians(spread_deg)
            else:
                spread_angle = 0
            
            angle_with_spread = src_angle + spread_angle
            cur_d = normalize(math.cos(angle_with_spread), math.sin(angle_with_spread))
            
            # Initialize ray trace
            segments_left = 20  # Increased for more complex setups
            cur_p = p0
            cur_int = intensity_per_ray
            
            # CRITICAL FIX: Track the component ID we just left
            last_comp_id = c['id']
            start_port = 0

            while segments_left > 0 and cur_int > 1e-4:
                segments_left -= 1
                closest = None
                closest_comp = None
                
                # Find closest component intersection
                for comp in comps:
                    # CRITICAL: Skip the component we just left (fixes 0-degree bug)
                    if comp['id'] == last_comp_id:
                        continue
                    
                    inter = intersect_line_segment(cur_p, cur_d, comp['seg'][0], comp['seg'][1])
                    if not inter:
                        continue
                    
                    ix, iy, tval = inter
                    if tval <= 0:
                        continue
                    
                    if closest is None or tval < closest[2]:
                        closest = (ix, iy, tval)
                        closest_comp = comp
                
                if closest is None:
                    # No hit: extend ray to boundary
                    x2 = cur_p[0] + cur_d[0] * 3000
                    y2 = cur_p[1] + cur_d[1] * 3000
                    
                    rays_out.append({
                        'x1': cur_p[0], 'y1': cur_p[1], 
                        'x2': x2, 'y2': y2, 
                        'int': cur_int,
                        'color': 'rgba(255, 255, 100, 0.6)',
                        'start_port': start_port,
                        'end_port': 0
                    })
                    break

                ix, iy, _ = closest
                end_port = closest_comp['port_number']
                
                # Add segment to intersection point
                rays_out.append({
                    'x1': cur_p[0], 'y1': cur_p[1], 
                    'x2': ix, 'y2': iy, 
                    'int': cur_int,
                    'hit': closest_comp['id'],
                    'color': 'rgba(255, 255, 100, 0.8)',
                    'start_port': start_port,
                    'end_port': end_port
                })

                # Track interaction
                component_interactions[closest_comp['id']] += 1
                start_port = end_port

                # --- MIRROR (Reflection) ---
                if closest_comp['type'] == 'Mirror':
                    total_reflections += 1
                    
                    # Get normal facing the incoming ray
                    normal = get_perpendicular_normal(
                        closest_comp['seg'][0], 
                        closest_comp['seg'][1],
                        cur_d
                    )
                    
                    # Reflect
                    cur_d = reflect_vector(cur_d, normal)
                    
                    # Move slightly along reflected direction to avoid self-intersection
                    cur_p = (ix + cur_d[0] * 1e-2, iy + cur_d[1] * 1e-2)
                    
                    # Apply reflectivity
                    R = closest_comp.get('properties', {}).get('reflectivity', 0.95)
                    cur_int *= R
                    
                    last_comp_id = closest_comp['id']
                    continue

                # --- LENS (Refraction) ---
                if closest_comp['type'] == 'Lens':
                    total_refractions += 1
                    
                    props = closest_comp.get('properties', {})
                    f = props.get('focalLength', 100.0)
                    
                    # Lens center
                    s1, s2 = closest_comp['seg']
                    lx = (s1[0] + s2[0]) / 2
                    ly = (s1[1] + s2[1]) / 2
                    
                    # Normal to lens
                    normal = get_perpendicular_normal(s1, s2, cur_d)
                    
                    # Focal point (on the opposite side)
                    focal_point = (lx + normal[0] * f, ly + normal[1] * f)
                    
                    # New direction toward focal point
                    new_dx = focal_point[0] - ix
                    new_dy = focal_point[1] - iy
                    cur_d = normalize(new_dx, new_dy)
                    
                    # Move slightly to avoid self-intersection
                    cur_p = (ix + cur_d[0] * 1e-2, iy + cur_d[1] * 1e-2)
                    
                    # Apply transmission loss
                    cur_int *= props.get('transmission', 0.98)
                    
                    last_comp_id = closest_comp['id']
                    continue

                # --- BEAM SPLITTER ---
                if closest_comp['type'] == 'Beam Splitter':
                    total_splits += 1
                    
                    s = closest_comp.get('properties', {}).get('splitRatio', 0.5)
                    
                    # Get normal
                    normal = get_perpendicular_normal(
                        closest_comp['seg'][0],
                        closest_comp['seg'][1],
                        cur_d
                    )
                    
                    # Reflected ray
                    refl_d = reflect_vector(cur_d, normal)
                    refl_int = (1 - s) * cur_int
                    
                    # Add reflected ray segment (visualization only)
                    rays_out.append({
                        'x1': ix, 'y1': iy,
                        'x2': ix + refl_d[0] * 200,
                        'y2': iy + refl_d[1] * 200,
                        'int': refl_int,
                        'color': 'rgba(100, 200, 255, 0.7)',
                        'start_port': end_port,
                        'end_port': 0
                    })
                    
                    # Continue transmitted ray
                    cur_p = (ix + cur_d[0] * 1e-2, iy + cur_d[1] * 1e-2)
                    cur_int *= s
                    
                    last_comp_id = closest_comp['id']
                    continue

                # --- PHOTO DETECTOR ---
                if closest_comp['type'] == 'Photo Detector':
                    sensitivity = closest_comp.get('properties', {}).get('sensitivity', 1.0)
                    detectors[closest_comp['id']] += cur_int * sensitivity
                    last_comp_id = closest_comp['id']
                    break

                # Unknown component type - stop ray
                last_comp_id = closest_comp['id']
                break

    # 3. Calculate additional statistics
    total_path_length = sum(
        math.hypot(s['x2'] - s['x1'], s['y2'] - s['y1']) 
        for s in rays_out
    )
    
    # Count rays per component
    interaction_summary = {
        comp['id']: {
            'type': comp['type'],
            'interactions': component_interactions[comp['id']]
        }
        for comp in comps if component_interactions[comp['id']] > 0
    }

    # 4. Return comprehensive results
    return jsonify({
        "status": "success",
        "results": {
            "rays_out": rays_out,
            "detectors": detectors,
            "statistics": {
                "total_path_length_mm": round(total_path_length, 2),
                "total_ray_segments": len(rays_out),
                "total_reflections": total_reflections,
                "total_refractions": total_refractions,
                "total_splits": total_splits,
                "component_interactions": interaction_summary
            }
        }
    })

@app.route('/api', methods=['GET'])
def index():
    return jsonify({"msg": "Optical Simulator Backend is running."})

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({"status": "healthy", "service": "optical-simulator"})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)