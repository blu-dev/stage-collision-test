use bevy::{ecs::query::WorldQuery, prelude::*};
use parry2d::na::{Isometry2, Vector2};
use parry2d::shape::{Cuboid, RoundCuboid, Segment};

use crate::{
    kinetic::{AirdodgeEnergy, ControllerEnergy, GravityEnergy},
    states::{State, StateQuery},
    Collider, Location, StageCollision,
};

#[derive(Component)]
pub struct CollisionDetectionStateChange(State);

pub struct AirParameters {
    pub gravity: f32,
    pub max_air_speed: f32,
    pub air_brake: f32,
    pub air_accel: f32,
    pub jump_height: f32,
    pub fall_speed: f32,
    pub ground_to_air: f32,
    pub airdodge_speed: f32,
}

pub struct GroundParameters {
    pub initial_speed: f32,
    pub run_accel: f32,
    pub run_accel_mul: f32,
    pub run_speed: f32,
    pub friction: f32,
}

#[derive(Component)]
pub struct PhysicsParameters {
    pub air: AirParameters,
    pub ground: GroundParameters,
}

#[derive(Default)]
pub struct KineticSet {
    pub velocity: Vector2<f32>,
    pub speed_max: Vector2<f32>,
    pub speed_limit: Vector2<f32>,
    pub accel: Vector2<f32>,
}

#[derive(Component, Default)]
pub struct Velocities {
    pub airdodge: AirdodgeEnergy,
    pub controller: ControllerEnergy,
    pub gravity: GravityEnergy,
    pub sum_speed: Vector2<f32>,
}

#[derive(Component, PartialEq, Eq, Copy, Clone)]
pub enum Facing {
    Right,
    Left,
}

impl Facing {
    pub fn sign(&self) -> f32 {
        match self {
            Self::Right => 1.0,
            Self::Left => -1.0,
        }
    }
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct StateUpdateQuery {
    pub entity: Entity,
    pub state: &'static mut State,
    pub state_update: StateQuery,
    pub incoming: Option<&'static CollisionDetectionStateChange>,
}

pub fn perform_state_update(mut commands: Commands, mut physics_objects: Query<StateUpdateQuery>) {
    for mut object in physics_objects.iter_mut() {
        let mut taken_incoming = false;
        loop {
            let state_change = if let Some(incoming) = object.incoming.filter(|_| !taken_incoming) {
                taken_incoming = true;
                commands
                    .entity(object.entity)
                    .remove::<CollisionDetectionStateChange>();
                Some(incoming.0)
            } else {
                let set = object.state.set();
                (set.on_update)(&mut object.state_update)
            };

            if let Some(change) = state_change {
                let set = object.state.set();
                (set.on_exit)(&mut object.state_update);
                *object.state = change;
                let set = object.state.set();
                (set.on_enter)(&mut object.state_update);
            } else {
                break;
            }
        }

        object
            .state_update
            .velocities
            .controller
            .update(object.state_update.inputs);
        object.state_update.velocities.gravity.update();
        object.state_update.velocities.airdodge.update();

        object.state_update.velocities.sum_speed =
            object.state_update.velocities.controller.velocity()
                + object.state_update.velocities.gravity.velocity()
                + object.state_update.velocities.airdodge.velocity();
    }
}

#[derive(Component)]
pub enum PushingWall {
    Neither,
    IntoRight,
    IntoLeft,
    Both,
}

impl PushingWall {
    fn set_right(&mut self) {
        match self {
            Self::Neither => *self = Self::IntoRight,
            Self::IntoLeft => *self = Self::Both,
            _ => {}
        }
    }

    fn set_left(&mut self) {
        match self {
            Self::Neither => *self = Self::IntoLeft,
            Self::IntoRight => *self = Self::Both,
            _ => {}
        }
    }
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct ColliderQuery {
    pub entity: Entity,
    pub collider: &'static Collider,
    pub location: &'static mut Location,
    pub state: &'static State,
    pub pushing_wall: Option<&'static mut PushingWall>,
    pub velocities: Option<&'static Velocities>,
}

fn perform_stay_grounded_check(
    collider_pos: &Isometry2<f32>,
    collider_shape: &RoundCuboid,
    stage_pos: &Isometry2<f32>,
    stage_line: &Segment,
) -> Option<f32> {
    match parry2d::query::distance(collider_pos, collider_shape, stage_pos, stage_line) {
        Ok(real) => (real < 2.0).then(|| {
            stage_line.a.y
                + stage_pos.translation.y
                + collider_shape.inner_shape.half_extents.y
                + collider_shape.border_radius
        }),
        _ => None,
    }
}

fn iter_tops(
    stage_collision: &StageCollision,
) -> impl Iterator<Item = (&Isometry2<f32>, &Segment)> {
    stage_collision
        .solid_ground
        .iter()
        .map(|solid| (&solid.position, &solid.shape.top))
        .chain(
            stage_collision
                .platforms
                .iter()
                .map(|platform| (&platform.position, &platform.shape)),
        )
}

fn iter_walls(
    stage_collision: &StageCollision,
) -> impl Iterator<Item = (&Isometry2<f32>, &Segment, f32)> {
    stage_collision.solid_ground.iter().flat_map(|solid| {
        [
            (&solid.position, &solid.shape.left, -1.0),
            (&solid.position, &solid.shape.right, 1.0),
        ]
        .into_iter()
    })
}

/// Returns the new Y value of the ground we hit
fn perform_hit_ground_check(
    collider_pos: &Isometry2<f32>,
    collider_vel: &Vector2<f32>,
    collider_shape: &RoundCuboid,
    stage_pos: &Isometry2<f32>,
    stage_line: &Segment,
) -> Option<f32> {
    match parry2d::query::time_of_impact(
        collider_pos,
        collider_vel,
        collider_shape,
        stage_pos,
        &Vector2::zeros(),
        stage_line,
        1.1,
        true,
    ) {
        Ok(Some(toi)) => {
            let collider_half_height =
                collider_shape.inner_shape.half_extents.y + collider_shape.border_radius;

            if !((-collider_half_height - 0.2)..=(-collider_half_height + 0.2))
                .contains(&toi.witness1.y)
            {
                None
            } else {
                Some(
                    stage_pos.translation.y
                        + stage_line.a.y
                        + collider_shape.inner_shape.half_extents.y
                        + collider_shape.border_radius,
                )
            }
        }
        _ => None,
    }
}

fn perform_waveland_detection(
    collider_shape: &RoundCuboid,
    waveland_pos: &Isometry2<f32>,
    waveland_shape: &Cuboid,
    stage_pos: &Isometry2<f32>,
    stage_line: &Segment,
) -> Option<f32> {
    match parry2d::query::intersection_test(waveland_pos, waveland_shape, stage_pos, stage_line) {
        Ok(true) => {
            let collider_half_height =
                collider_shape.inner_shape.half_extents.y + collider_shape.border_radius;

            Some(stage_pos.translation.y + stage_line.a.y + collider_half_height)
        }
        _ => None,
    }
}

/// STEP 1: This performs ground detection
/// - For grounded states, we are going to check if we are still colliding with the ground
///      This is done after moving the objects in the X direction, so the state change
///      should be interpreted as "my object moved off of the stage last frame", which will
///      prevent there being a frame where gravity just don't work LOL
/// - For aerial states, we are going to check if we are colliding with the ground
///      This is done by checking if the object will collide with the ground in the next 1.1
///      frames. If it does, we snap the object to the ground and move them the correct X
///      delta. This state change should be interpreted as "my object connected with the
///      ground last frame"
///  - If wavelanding is enabled, we are going to check if the waveland collider is
///      intersecting the ground AT ALL. If it is, we are going to snap the object
///      to the ground there.
fn perform_ground_collision(
    commands: &mut Commands,
    collider: &mut ColliderQueryItem,
    stage_collision: &StageCollision,
) {
    match collider.state {
        // These are grounded states, so the collision logic here is to check if we are still on
        // a grounded surface
        //
        // In a real engine this should be handled via a check on a "situation", i.e. whether you
        // are grounded or airborne
        State::Idle | State::Dash | State::Landing => {
            if let Some(vel) = collider.velocities {
                collider.location.position.translation.x += vel.sum_speed.x;
            }

            // If we are able to get a position to snap to, then we are good and are still
            // on the surface, if we aren't then we need to communicate that we have
            // left the grounded situation and let some other code handle that
            if let Some(pos) = iter_tops(&stage_collision).find_map(|(pos, seg)| {
                perform_stay_grounded_check(
                    &collider.location.position,
                    &collider.collider.shape,
                    pos,
                    seg,
                )
            }) {
                collider.location.position.translation.y = pos;
            } else {
                // In this example, we directly trigger a "CollisionDetectionStateChange",
                // however in a real engine this would likely signal that the situation
                // has changed and let user code handle it differently depending on states
                commands
                    .entity(collider.entity)
                    .insert(CollisionDetectionStateChange(State::Fall));
            }
        }
        // These are aerial states, so the collision logic here is to check if we are colliding
        // with a grounded surface
        //
        // In a real engine this should also be handled via a situation
        State::WallJump | State::Jump | State::Fall | State::Airdodge => 'hit_detection: {
            // In this example, we don't have any moving ground collision objects
            // so there would be no way to collide with the ground unless the collider
            // is moving
            let Some(vel) = collider.velocities else {
                break 'hit_detection;
            };

            // If we are moving vertically, there is no way to collide with the ground here
            //
            // This is true both because our wall detection code should guarantee that we
            // are never inside of a solid collision object, and we don't want to collide
            // with platforms when we are moving vertically
            if vel.sum_speed.y >= 0.0 {
                collider.location.position.translation.vector += vel.sum_speed;
                break 'hit_detection;
            }

            // If we can find some ground that we collide with, we need to signal that
            // to the user code
            //
            // This logic just checks if at any point during the next 1.1 frames we will
            // collide with the ground using our current velocity. If we would then we
            // need to represent that here
            if let Some(pos) = iter_tops(&stage_collision).find_map(|(pos, seg)| {
                perform_hit_ground_check(
                    &collider.location.position,
                    &vel.sum_speed,
                    &collider.collider.shape,
                    pos,
                    seg,
                )
            }) {
                // Snap our y position (don't use a delta force to move us around as that could
                // lead to decomposition of our position state)
                collider.location.position.translation.y = pos;

                // Move our x position the proper amount, we will double check this during wall
                // collision phase
                collider.location.position.translation.x += vel.sum_speed.x;

                // In this example, we directly signal to change to a landing state,
                // but in a real engine we would likely signal that our situation has changed
                // and the user code should handle that
                commands
                    .entity(collider.entity)
                    .insert(CollisionDetectionStateChange(State::Landing));
            } else {
                // If we didn't detect that we would collide with anything then we are safe to go
                // ahead and move forward with full movement (again we will check horizontal)
                // during our wall collision pase
                collider.location.position.translation.vector += vel.sum_speed;
            }
        }
    }

    // In this example, the waveland collider is always set, but in a real engine you should
    // conditionally set this collider so that you aren't performing state checks.
    let Some(waveland) = collider
        .collider
        .waveland_shape
        .as_ref()
        .filter(|_| *collider.state == State::Airdodge)
    else {
        return;
    };

    // The waveland collider's position is relative to the base collider, so we need to calculate
    // the position for it
    let position = Isometry2::new(
        collider.location.position.translation.vector + waveland.position.translation.vector,
        0.0,
    );

    // Look for all grounded surfaces, if we can find one that we we intersect then we are going to
    // snap to it
    if let Some(pos) = iter_tops(&stage_collision).find_map(|(pos, seg)| {
        perform_waveland_detection(
            &collider.collider.shape,
            &position,
            &waveland.shape,
            pos,
            seg,
        )
    }) {
        collider.location.position.translation.y = pos;

        // In this example, we manually set the state change, however in a real engine this should
        // probably have the same situation change signal as a regular landing collision
        commands
            .entity(collider.entity)
            .insert(CollisionDetectionStateChange(State::Landing));
    }
}

/// STEP 2 - Performing Wall Collision
///
/// Performing wall collision is import for detecting when an object is "pushing" into a wall.
/// Collision objects do not maintain state about which collision object they are currently
/// touching on any side (yet, although that could be done without much of a lift).
///
/// When an object is "pushing" into a wall, that means that things like wall jumps or wall bounces.
/// We also need to detect when an object is moving into a wall to prevent it from continuing to
/// move into the wall
///
/// The pre-condition for calling this function is to ensure that the collision object has been
/// "moved", meaning it's [`Location`] component has been updated to reflect it's velocity this
/// frame. This is important because this function resimulates the movement
fn perform_wall_collision(collider: &mut ColliderQueryItem, stage_collision: &StageCollision) {
    // In this example, we don't have any moving ground collision so there would be no way to collide
    // with a wall unless the collider was also moving
    let Some(vel) = collider.velocities else {
        return;
    };

    let mut pushing = PushingWall::Neither;

    for (position, segment, normal) in iter_walls(stage_collision) {
        // Ensure that we are moving towards the wall in question
        if normal.is_sign_positive() == vel.sum_speed.x.is_sign_positive() {
            continue;
        }

        // Our ground collision has already corrected our vertical movement, but now
        // we need to correct our horizontal movement. To do this we revert the velocity
        // only in the horizontal direction and perform another time of impact query
        let x_vel = Vector2::new(vel.sum_speed.x, 0.0);
        let reverted_pos =
            Isometry2::new(collider.location.position.translation.vector - x_vel, 0.0);

        // Check if we would collide with the wall or not (1.1 frames worth for leniency)
        match parry2d::query::time_of_impact(
            &reverted_pos,
            &x_vel,
            &collider.collider.shape,
            position,
            &Vector2::zeros(),
            segment,
            1.1,
            false,
        ) {
            Ok(Some(toi)) => {
                let collider_half_width = collider.collider.shape.inner_shape.half_extents.x
                    + collider.collider.shape.border_radius;

                // In some situations (usually consecutive touching frames), the witness location
                // will be a very small, close to 0, number
                //
                // If that's the case, it will be the same sign as the normal, so we can convert it
                // into a different spacial point that will work nicer with our range check
                let witness = if toi.witness1.x.is_sign_positive() == normal.is_sign_positive() {
                    -toi.witness1.x + -normal * collider_half_width
                } else {
                    toi.witness1.x
                };

                // The time of impact query should give a VERY GOOD estimation of the edge of our
                // collider colliding with the wall. These +-0.2 leniency here is definitely
                // overkill, but for an engine that has cross-architecture play/netocde this
                // should provide peace of mind with floating point precision
                let range_a = -normal * (collider_half_width - 0.2);
                let range_b = -normal * (collider_half_width + 0.2);
                let range = range_a.min(range_b)..=range_a.max(range_b);

                if range.contains(&witness) {
                    collider.location.position.translation.x =
                        position.translation.x + segment.a.x + collider_half_width * normal;

                    if x_vel.x.is_sign_positive() {
                        pushing.set_left()
                    } else {
                        pushing.set_right();
                    }
                }
            }
            _ => {}
        }
    }

    // This is only optional because we don't want to conditionally disable wall collision
    // (or grounded collision) for an object that doesn't have it. This state is for
    // the user to consume so it's ok if they don't want or need it.
    if let Some(pushing_into) = collider.pushing_wall.as_mut() {
        **pushing_into = pushing;
    }
}

pub fn perform_object_movements(
    mut commands: Commands,
    mut colliders: Query<ColliderQuery>,
    stage_collision: Res<StageCollision>,
) {
    for mut collider in colliders.iter_mut() {
        perform_ground_collision(&mut commands, &mut collider, &stage_collision);

        // We perform wall collision after ground collision to ensure that ground has priority
        // I.e. if we collide with a corner, we should consider grounded connection more important
        // than walled connection for gamefeel reasons
        perform_wall_collision(&mut collider, &stage_collision);
    }
}
