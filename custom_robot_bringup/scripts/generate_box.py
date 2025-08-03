def generate_box_tags(x, y, z, mass, collision_name="body_collision"):
    # Inertia for a solid box
    ixx = (1 / 12) * mass * (y**2 + z**2)
    iyy = (1 / 12) * mass * (x**2 + z**2)
    izz = (1 / 12) * mass * (x**2 + y**2)
    # Inertial tag
    inertial = f"""<inertial>
  <mass>{mass}</mass>
  <inertia>
    <ixx>{ixx}</ixx>
    <iyy>{iyy}</iyy>
    <izz>{izz}</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>
</inertial>"""
    # Collision tag
    collision = f"""<collision name="{collision_name}">
  <geometry>
    <box>
      <size>{x} {y} {z}</size>
    </box>
  </geometry>
</collision>"""
    return inertial, collision


# Example usage:
x, y, z = 1.0, 0.12, 0.12  # meters
mass = 12  # kg
inertial_tag, collision_tag = generate_box_tags(x, y, z, mass)
print(inertial_tag)
print(collision_tag)
