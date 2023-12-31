use bevy::prelude::*;
use parry2d::math::DEFAULT_EPSILON;

use crate::{Collider, Convert, Location, StageCollision};

pub fn draw_collision(
    mut gizmos: Gizmos,
    colliders: Query<(&Collider, &Location)>,
    stage: Res<StageCollision>,
) {
    for solid in stage.solid_ground.iter() {
        let width = solid.shape.left.a.x - solid.shape.right.a.x;
        let height = solid.shape.top.a.y - solid.shape.bottom.a.y;
        let pos = solid.position.translation.vector.convert();

        gizmos.rect_2d(pos, 0.0, Vec2::new(width, height), Color::ORANGE);
    }

    for plat in stage.platforms.iter() {
        let pos = plat.position.translation.vector.convert();
        let a = plat.shape.a.convert() + pos;
        let b = plat.shape.b.convert() + pos;

        gizmos.line_2d(a, b, Color::WHITE);
    }

    for (collider, location) in colliders.iter() {
        if collider.shape.border_radius <= DEFAULT_EPSILON {
            gizmos.rect_2d(
                location.position.translation.vector.convert(),
                0.0,
                collider.shape.inner_shape.half_extents.convert() * 2.0,
                Color::GREEN,
            );
        } else {
            let position = location.position.translation.vector.convert();
            let points = collider.shape.to_polyline(8);
            if points.is_empty() {
                continue;
            }
            let first = points[0].convert() + position;
            gizmos.linestrip_2d(
                points
                    .into_iter()
                    .map(|p| p.convert() + position)
                    .chain([first]),
                Color::GREEN,
            );
        }
    }

    for (collider, location) in colliders.iter() {
        if let Some(waveland_shape) = collider.waveland_shape.as_ref() {
            gizmos.rect_2d(
                location.position.translation.vector.convert()
                    + waveland_shape.position.translation.vector.convert(),
                0.0,
                waveland_shape.shape.half_extents.convert() * 2.0,
                Color::SEA_GREEN,
            );
        }
    }
}
