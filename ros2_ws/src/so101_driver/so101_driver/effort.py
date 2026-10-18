"""Normalized joint effort from the MuJoCo state (docs/project/03 §5, ADR-016). Pure, no ROS."""


def normalized_effort(model, data, names) -> list[float]:
    """Actuator force over the upper bound of its forcerange, per joint, clipped to [-1, 1].

    Actuators are looked up by name (the MJCF names them after their joint). Sign: MuJoCo's, positive
    towards larger angles, except the gripper, which closes towards smaller angles: its value is the
    closing force, so holding an object is positive (0.12-0.19 on the cube, 0 empty [MES-10]).
    """
    effort = []
    for name in names:
        actuator = model.actuator(name).id
        value = data.actuator_force[actuator] / model.actuator_forcerange[actuator][1]
        effort.append(min(max(float(-value if name == "gripper" else value), -1.0), 1.0))
    return effort
