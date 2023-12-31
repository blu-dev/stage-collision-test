use bevy::{ecs::query::WorldQuery, prelude::*};

use crate::{
    input::{ButtonBitflags, MappedInput},
    kinetic::{ControllerEnergyState, GravityEnergyState},
    physics::{Facing, PhysicsParameters, PushingWall, Velocities},
};

#[derive(Component, Copy, Clone, PartialEq, Eq)]
pub enum State {
    Idle,
    Dash,
    Jump,
    Fall,
    Airdodge,
    Landing,
    WallJump,
}

impl State {
    pub fn set(&self) -> &'static StateSet {
        match self {
            Self::Idle => &IDLE_STATE,
            Self::Dash => &DASH_STATE,
            Self::Jump => &JUMP_STATE,
            Self::Fall => &FALL_STATE,
            Self::Airdodge => &AIRDODGE_STATE,
            Self::Landing => &LANDING_STATE,
            Self::WallJump => &WALL_JUMP_STATE,
        }
    }
}

#[derive(Component, Deref, DerefMut, Default)]
pub struct Counter(usize);

#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct StateQuery {
    pub entity: Entity,
    pub inputs: &'static MappedInput,
    pub facing: &'static mut Facing,
    pub params: &'static PhysicsParameters,
    pub velocities: &'static mut Velocities,
    pub counter: &'static mut Counter,
    pub pushing: &'static PushingWall,
}

pub struct StateSet {
    pub on_enter: fn(&mut StateQueryItem),
    pub on_update: fn(&mut StateQueryItem) -> Option<State>,
    pub on_exit: fn(&mut StateQueryItem),
}

pub const IDLE_STATE: StateSet = StateSet {
    on_enter: |state| {
        let sum_speed_x = state.velocities.sum_speed.x;
        state.velocities.airdodge.enabled = false;
        state.velocities.controller.enabled = true;
        state.velocities.controller.initialize(
            sum_speed_x,
            *state.facing,
            state.params,
            ControllerEnergyState::Stand,
        );
        state.velocities.gravity.enabled = false;
    },
    on_update: |state| {
        if state.inputs.stick_l.x.abs() > 0.5
            && (state.inputs.stick_l.x - state.inputs.stick_l_last.x).abs() > 0.3
        {
            if state.inputs.stick_l.x.is_sign_positive() {
                *state.facing = Facing::Right;
            } else {
                *state.facing = Facing::Left;
            }
            return Some(State::Dash);
        }

        if state.inputs.press(ButtonBitflags::JUMP) {
            return Some(State::Jump);
        }

        None
    },
    on_exit: |_| {},
};

pub const DASH_STATE: StateSet = StateSet {
    on_enter: |state| {
        let sum_speed_x = state.velocities.sum_speed.x;
        state.velocities.airdodge.enabled = false;
        state.velocities.controller.enabled = true;
        state.velocities.controller.initialize(
            sum_speed_x,
            *state.facing,
            state.params,
            ControllerEnergyState::Dash,
        );
        state.velocities.gravity.enabled = false;
    },
    on_update: |state| {
        if state.inputs.stick_l.x.abs() < 0.3 {
            return Some(State::Idle);
        }

        if state.inputs.press(ButtonBitflags::JUMP) {
            return Some(State::Jump);
        }

        None
    },
    on_exit: |_| {},
};

pub const JUMP_STATE: StateSet = StateSet {
    on_enter: |state| {
        let sum_speed_x = state.velocities.sum_speed.x;
        state.velocities.airdodge.enabled = false;
        state.velocities.controller.enabled = true;
        state.velocities.controller.initialize(
            sum_speed_x,
            *state.facing,
            state.params,
            ControllerEnergyState::FallAdjust,
        );
        state.velocities.gravity.enabled = true;
        state
            .velocities
            .gravity
            .initialize(0.0, state.params, GravityEnergyState::Jump);
    },
    on_update: |state| {
        if state.inputs.press(ButtonBitflags::SHIELD) {
            return Some(State::Airdodge);
        }

        match state.pushing {
            PushingWall::Both => {
                if state.inputs.stick_l.x.abs() > 0.4 && state.inputs.press(ButtonBitflags::JUMP) {
                    return Some(State::WallJump);
                }
            }
            PushingWall::IntoLeft => {
                if state.inputs.stick_l.x > 0.4 && state.inputs.press(ButtonBitflags::JUMP) {
                    return Some(State::WallJump);
                }
            }
            PushingWall::IntoRight => {
                if state.inputs.stick_l.x < -0.4 && state.inputs.press(ButtonBitflags::JUMP) {
                    return Some(State::WallJump);
                }
            }
            PushingWall::Neither => {}
        }

        None
    },
    on_exit: |_| {},
};

pub const FALL_STATE: StateSet = StateSet {
    on_enter: |state| {
        let sum_speed = state.velocities.sum_speed;
        state.velocities.airdodge.enabled = false;
        state.velocities.controller.enabled = true;
        state.velocities.controller.initialize(
            sum_speed.x,
            *state.facing,
            state.params,
            ControllerEnergyState::FallAdjust,
        );
        state.velocities.gravity.enabled = true;
        state
            .velocities
            .gravity
            .initialize(sum_speed.y, state.params, GravityEnergyState::Fall);
    },
    on_update: |state| {
        if state.inputs.press(ButtonBitflags::SHIELD) {
            return Some(State::Airdodge);
        }

        match state.pushing {
            PushingWall::Both => {
                if state.inputs.stick_l.x.abs() > 0.4 && state.inputs.press(ButtonBitflags::JUMP) {
                    return Some(State::WallJump);
                }
            }
            PushingWall::IntoLeft => {
                if state.inputs.stick_l.x > 0.4 && state.inputs.press(ButtonBitflags::JUMP) {
                    return Some(State::WallJump);
                }
            }
            PushingWall::IntoRight => {
                if state.inputs.stick_l.x < -0.4 && state.inputs.press(ButtonBitflags::JUMP) {
                    return Some(State::WallJump);
                }
            }
            PushingWall::Neither => {}
        }

        None
    },
    on_exit: |_| {},
};

pub const AIRDODGE_STATE: StateSet = StateSet {
    on_enter: |state| {
        state.velocities.controller.enabled = false;
        state.velocities.gravity.enabled = false;
        state.velocities.airdodge.enabled = true;
        state
            .velocities
            .airdodge
            .initialize(state.inputs, state.params);
        **state.counter = 0;
    },
    on_update: |state| {
        if **state.counter >= 30 {
            return Some(State::Fall);
        }
        **state.counter += 1;
        None
    },
    on_exit: |_| {},
};

pub const LANDING_STATE: StateSet = StateSet {
    on_enter: |state| {
        state.velocities.gravity.enabled = false;
        state.velocities.airdodge.enabled = false;
        state.velocities.controller.enabled = true;
        let sum_speed_x = state.velocities.sum_speed.x;
        state.velocities.controller.initialize(
            sum_speed_x,
            *state.facing,
            state.params,
            ControllerEnergyState::Stand,
        );
        **state.counter = 0;
    },
    on_update: |state| {
        if **state.counter >= 5 {
            return Some(State::Idle);
        }

        **state.counter += 1;

        None
    },
    on_exit: |_| {},
};

pub const WALL_JUMP_STATE: StateSet = StateSet {
    on_enter: |state| {
        state.velocities.airdodge.enabled = false;
        state.velocities.controller.enabled = true;
        state.velocities.controller.initialize(
            -1.0 * state.inputs.stick_l.x.signum() * state.params.air.max_air_speed,
            *state.facing,
            state.params,
            ControllerEnergyState::FallAdjust,
        );
        state.velocities.gravity.enabled = true;
        state
            .velocities
            .gravity
            .initialize(0.0, state.params, GravityEnergyState::Jump);
    },
    on_update: |state| {
        if state.inputs.press(ButtonBitflags::SHIELD) {
            return Some(State::Airdodge);
        }

        None
    },
    on_exit: |_| {},
};
