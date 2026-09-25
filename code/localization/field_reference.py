"""Build D500 wall geometry from the validated competition-field profile."""

from __future__ import annotations

from components.radar_driver import DroneGlobalAlignment, RectangularWallReference
from config.v2_models import D500LocalizationConfig


def build_field_wall_reference(config: D500LocalizationConfig) -> RectangularWallReference:
    """Create a field-frame wall reference using only configured wall sides."""

    back_x_cm = config.back_wall_x_m * 100.0
    right_y_cm = config.right_wall_y_m * 100.0
    return RectangularWallReference(
        wall_to_global=DroneGlobalAlignment(0.0, 0.0, 0.0),
        back_wall_x_cm=back_x_cm,
        right_wall_y_cm=right_y_cm,
        front_wall_x_cm=(back_x_cm + config.field_width_m * 100.0) if config.use_front_wall else None,
        left_wall_y_cm=(right_y_cm + config.field_height_m * 100.0) if config.use_left_wall else None,
    )
