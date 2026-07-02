def project_pixel_to_3d(u, v, depth_m, fx, fy, cx, cy):
    z = float(depth_m)
    x = (float(u) - float(cx)) * z / float(fx)
    y = (float(v) - float(cy)) * z / float(fy)
    return [x, y, z]
