import capytaine as cpt
import numpy as np

mesh = cpt.load_mesh("tethys.stl")

body = cpt.FloatingBody(mesh=mesh)
body.add_all_rigid_body_dofs()

# Pick a frequency for radiation problem (rad/s)
omega = 2.0

# We'll loop over all DOFs and sum results for damping and added mass
dofs = ["Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"]

added_mass = {}
radiation_damping = {}

solver = cpt.BEMSolver()

for dof in dofs:
    problem = cpt.RadiationProblem(body=body, omega=omega, radiating_dof=dof)
    result = solver.solve(problem)
    added_mass[dof] = result.added_mass[dof]
    radiation_damping[dof] = result.radiation_damping[dof]

# For linear terms, put 0 for xU, yV, zW; for angular terms, 0 too
lin_drag = {"xU": 0, "yV": 0, "zW": 0, "kP": 0, "mQ": 0, "nR": 0}

# Quadratic drag - you can guess, or use radiation damping values scaled
quad_drag = {
    "xUabsU": -radiation_damping["Surge"],
    "yVabsV": -radiation_damping["Sway"],
    "zWabsW": -radiation_damping["Heave"],
    "kPabsP": -radiation_damping["Roll"],
    "mQabsQ": -radiation_damping["Pitch"],
    "nRabsR": -radiation_damping["Yaw"],
}

print(
    f"""
<xDotU>{-added_mass['Surge']:.6f}</xDotU>
<yDotV>{-added_mass['Sway']:.6f}</yDotV>
<zDotW>{-added_mass['Heave']:.6f}</zDotW>
<kDotP>{-added_mass['Roll']:.6f}</kDotP>
<mDotQ>{-added_mass['Pitch']:.6f}</mDotQ>
<nDotR>{-added_mass['Yaw']:.6f}</nDotR>
<xUabsU>{quad_drag['xUabsU']:.6f}</xUabsU>
<xU>{lin_drag['xU']}</xU>
<yVabsV>{quad_drag['yVabsV']:.6f}</yVabsV>
<yV>{lin_drag['yV']}</yV>
<zWabsW>{quad_drag['zWabsW']:.6f}</zWabsW>
<zW>{lin_drag['zW']}</zW>
<kPabsP>{quad_drag['kPabsP']:.6f}</kPabsP>
<kP>{lin_drag['kP']}</kP>
<mQabsQ>{quad_drag['mQabsQ']:.6f}</mQabsQ>
<mQ>{lin_drag['mQ']}</mQ>
<nRabsR>{quad_drag['nRabsR']:.6f}</nRabsR>
<nR>{lin_drag['nR']}</nR>
"""
)
