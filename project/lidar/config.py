is_real_mode = True
if is_real_mode:
    resolution = 0.2  # 1 pixel is how many meters
    side_range = (-20, 20)
    fwd_range = (2.5, 8)
    height_range = (-1.3, 1.5)
else:
    resolution = 0.2  # 1 pixel is how many meters
    side_range = (-20, 20)
    fwd_range = (2.0, 20)
    height_range = (-1.0, 0.5)
