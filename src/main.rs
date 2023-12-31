use bevy::prelude::*;
use debug_draw::draw_collision;
use input::{MappedInput, PlayerHandle};
use parry::{
    na::{Isometry2, Point2, Vector2},
    shape::{Cuboid, RoundCuboid, Segment},
};
use parry2d as parry;
use physics::{
    AirParameters, Facing, GroundParameters, PhysicsParameters, PushingWall, Velocities,
};
use smallvec::SmallVec;
use states::{Counter, State};

mod debug_draw;
mod input;
mod kinetic;
mod physics;
mod states;

pub trait Convert<T> {
    fn convert(self) -> T;
}

impl Convert<Vec2> for Vector2<f32> {
    fn convert(self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }
}

impl Convert<Vec2> for Point2<f32> {
    fn convert(self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }
}

impl Convert<Vector2<f32>> for Vec2 {
    fn convert(self) -> Vector2<f32> {
        Vector2::new(self.x, self.y)
    }
}

impl Convert<Point2<f32>> for Vec2 {
    fn convert(self) -> Point2<f32> {
        Point2::new(self.x, self.y)
    }
}

pub struct Locatable<T> {
    pub shape: T,
    pub position: Isometry2<f32>,
}

pub struct SolidSegments {
    pub top: Segment,
    pub bottom: Segment,
    pub right: Segment,
    pub left: Segment,
}

impl SolidSegments {
    fn from_half_extents(he: Vector2<f32>) -> Self {
        let top_right = he;
        let bottom_left = -he;
        let top_left = Vector2::new(-he.x, he.y);
        let bottom_right = Vector2::new(he.x, -he.y);

        Self {
            top: Segment {
                a: top_left.into(),
                b: top_right.into(),
            },
            bottom: Segment {
                a: bottom_left.into(),
                b: bottom_right.into(),
            },
            right: Segment {
                a: top_right.into(),
                b: bottom_right.into(),
            },
            left: Segment {
                a: top_left.into(),
                b: bottom_left.into(),
            },
        }
    }
}

/// Represents a set of stage collision
#[derive(Resource)]
pub struct StageCollision {
    pub solid_ground: SmallVec<[Locatable<SolidSegments>; 4]>,
    pub platforms: SmallVec<[Locatable<Segment>; 8]>,
}

#[derive(Component)]
pub struct Collider {
    /// The shape of the Collider
    ///
    /// Collider shapes are rounded rectangles, if the rounding for the rectangle is
    /// less than [`parry::math::DEFAULT_EPSILON`] then just a normal rectangle will be used.
    pub shape: RoundCuboid,

    /// The shape of the waveland collider
    ///
    /// Waveland colliders are special, since they only perform proximity detection
    /// on the ground collision to determine when snapping to ground is appropriate
    ///
    /// Waveland snaps also only apply to platform ground, not solid ground
    pub waveland_shape: Option<Locatable<Cuboid>>,
}

#[derive(Component)]
pub struct Location {
    pub position: Isometry2<f32>,
}

fn startup(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        transform: Transform::from_scale(Vec3::splat(2.0)),
        ..default()
    });
    commands.insert_resource(StageCollision {
        solid_ground: SmallVec::from_iter([Locatable {
            shape: SolidSegments::from_half_extents(Vector2::new(500.0, 400.0)),
            position: Isometry2::new(Vector2::new(0.0, -400.0), 0.0),
        }]),
        platforms: SmallVec::from_iter([Locatable {
            shape: Segment {
                a: Point2::new(-50.0, 0.0),
                b: Point2::new(50.0, 0.0),
            },
            position: Isometry2::new(Vector2::new(300.0, 75.0), 0.0),
        }]),
    });

    commands.spawn((
        Collider {
            shape: RoundCuboid {
                inner_shape: Cuboid::new(Vector2::new(27.0, 37.0)),
                border_radius: 3.0,
            },
            waveland_shape: Some(Locatable {
                shape: Cuboid::new(Vector2::new(30.0, 35.0)),
                position: Isometry2::new(Vector2::new(0.0, -35.0), 0.0),
            }),
        },
        Location {
            position: Isometry2::new(Vector2::new(30.0, 70.0), 0.0),
        },
        PhysicsParameters {
            air: AirParameters {
                gravity: 0.1,
                max_air_speed: 8.0,
                air_brake: 0.025,
                air_accel: 0.5,
                jump_height: 75.0,
                fall_speed: 4.0,
                ground_to_air: 0.9,
                airdodge_speed: 8.0,
            },
            ground: GroundParameters {
                initial_speed: 4.0,
                run_accel: 0.05,
                run_speed: 10.0,
                friction: 0.30,
                run_accel_mul: 0.3,
            },
        },
        Velocities::default(),
        State::Idle,
        PlayerHandle::Keyboard,
        MappedInput::default(),
        Facing::Right,
        Counter::default(),
        PushingWall::Neither,
    ));
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(PreUpdate, input::map_inputs)
        .add_systems(
            Update,
            (
                draw_collision,
                bevy::window::close_on_esc,
                (
                    physics::perform_state_update,
                    physics::perform_object_movements,
                )
                    .chain(),
            ),
        )
        .add_systems(Startup, startup)
        .run();
}
