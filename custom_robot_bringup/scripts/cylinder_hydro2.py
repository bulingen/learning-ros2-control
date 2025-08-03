import numpy as np

# Cylinder parameters (meters)
radius = 0.06
length = 1.0  # Use 'length' for clarity (was 'height' for vertical)

# Orientation toggle: True = horizontal (AUV/submarine), False = vertical
is_horizontal = True

# Fluid density (kg/m^3)
rho = 1000

# Volume of cylinder
volume = np.pi * radius**2 * length

# Added mass coefficients (potential flow theory)
# For a horizontal cylinder (AUV):
#   - Surge (along axis): small, ~0.1 × mass of displaced fluid
#   - Sway/Heave (perpendicular): large, ~1.0 × mass of displaced fluid
# For a vertical cylinder:
#   - Surge/Sway (perpendicular): large, ~1.0 × mass of displaced fluid
#   - Heave (along axis): small, ~0.1 × mass of displaced fluid

if is_horizontal:
    added_mass_surge = rho * volume * 0.1  # Surge (along axis)
    added_mass_sway = rho * volume * 1.0  # Sway (perpendicular)
    added_mass_heave = rho * volume * 1.0  # Heave (perpendicular)
    # Projected areas for drag
    area_surge = np.pi * radius**2  # End area (along axis)
    area_sway = 2 * radius * length  # Side area (perpendicular)
    area_heave = 2 * radius * length  # Side area (perpendicular)
    # Quadratic drag coefficients (Cd)
    Cd_surge = 0.82  # Along axis (streamlined)
    Cd_sway = 1.0  # Perpendicular (bluff body)
    Cd_heave = 1.0  # Perpendicular (bluff body)
else:
    added_mass_surge = rho * volume * 1.0  # Surge (perpendicular)
    added_mass_sway = rho * volume * 1.0  # Sway (perpendicular)
    added_mass_heave = rho * volume * 0.1  # Heave (along axis)
    # Projected areas for drag
    area_surge = 2 * radius * length  # Side area (perpendicular)
    area_sway = 2 * radius * length  # Side area (perpendicular)
    area_heave = np.pi * radius**2  # End area (along axis)
    # Quadratic drag coefficients (Cd)
    Cd_surge = 1.0  # Perpendicular (bluff body)
    Cd_sway = 1.0  # Perpendicular (bluff body)
    Cd_heave = 0.82  # Along axis (streamlined)

# Quadratic drag (negative for damping)
quad_drag_x = -0.5 * rho * Cd_surge * area_surge
quad_drag_y = -0.5 * rho * Cd_sway * area_sway
quad_drag_z = -0.5 * rho * Cd_heave * area_heave

# Linear drag (often negligible, set to zero)
lin_drag_x = 0
lin_drag_y = 0
lin_drag_z = 0

# Angular added mass and drag (set to zero for simplicity)
kDotP = 0
mDotQ = 0
nDotR = 0
kP = 0
mQ = 0
nR = 0
kPabsP = 0
mQabsQ = 0
nRabsR = 0

print(
    f"""
<!-- Added mass: -->
<xDotU>{-added_mass_surge:.6f}</xDotU>
<yDotV>{-added_mass_sway:.6f}</yDotV>
<zDotW>{-added_mass_heave:.6f}</zDotW>
<kDotP>{kDotP}</kDotP>
<mDotQ>{mDotQ}</mDotQ>
<nDotR>{nDotR}</nDotR>
<!-- First order stability derivative: -->
<xU>{lin_drag_x}</xU>
<yV>{lin_drag_y}</yV>
<zW>{lin_drag_z}</zW>
<kP>{kP}</kP>
<mQ>{mQ}</mQ>
<nR>{nR}</nR>
<!-- Second order stability derivative: -->
<xUabsU>{quad_drag_x:.6f}</xUabsU>
<yVabsV>{quad_drag_y:.6f}</yVabsV>
<zWabsW>{quad_drag_z:.6f}</zWabsW>
<kPabsP>{kPabsP}</kPabsP>
<mQabsQ>{mQabsQ}</mQabsQ>
<nRabsR>{nRabsR}</nRabsR>
"""
)

# Notes:
# - Set is_horizontal = True for AUV/submarine (horizontal), False for vertical.
# - Added mass and drag coefficients are chosen based on orientation and direction.
# - All angular terms are set to zero for simplicity
