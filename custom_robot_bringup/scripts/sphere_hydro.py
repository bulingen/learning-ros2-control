import numpy as np


# Notes:
# - Added mass for a sphere: 0.5 * mass of displaced fluid in all directions.
# - Quadratic drag uses Cd=0.47 for a sphere.
# - All angular terms are set to zero for a perfect


# Sphere parameters (meters)
radius = 0.2

# Fluid density (kg/m^3)
rho = 1000

# Volume of sphere
volume = (4 / 3) * np.pi * radius**3

# Added mass coefficients for a sphere (potential flow theory)
# Surge/Sway: 0.5 * mass of displaced fluid
# Heave: 0.5 * mass of displaced fluid (same for all directions for a sphere)
added_mass_coeff = 0.5
added_mass = rho * volume * added_mass_coeff

# Quadratic drag coefficient (Cd) for a sphere in water is typically ~0.47
# Drag force: 0.5 * rho * Cd * A * v^2
# Here, we use a lumped value for the quadratic drag term
# Negative sign for damping convention
Cd = 0.47
area = np.pi * radius**2
quad_drag = -0.5 * rho * Cd * area

# Linear drag (often negligible for a sphere, set to zero)
lin_drag = 0

# Angular added mass and drag (very small for a sphere, set to zero)
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
<xDotU>{-added_mass:.6f}</xDotU>
<yDotV>{-added_mass:.6f}</yDotV>
<zDotW>{-added_mass:.6f}</zDotW>
<kDotP>{kDotP}</kDotP>
<mDotQ>{mDotQ}</mDotQ>
<nDotR>{nDotR}</nDotR>
<!-- First order stability derivative: -->
<xU>{lin_drag}</xU>
<yV>{lin_drag}</yV>
<zW>{lin_drag}</zW>
<kP>{kP}</kP>
<mQ>{mQ}</mQ>
<nR>{nR}</nR>
<!-- Second order stability derivative: -->
<xUabsU>{quad_drag:.6f}</xUabsU>
<yVabsV>{quad_drag:.6f}</yVabsV>
<zWabsW>{quad_drag:.6f}</zWabsW>
<kPabsP>{kPabsP}</kPabsP>
<mQabsQ>{mQabsQ}</mQabsQ>
<nRabsR>{nRabsR}</nRabsR>
"""
)
