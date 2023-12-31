use bevy::math::Vec2;
use parry2d::na::Vector2;

use crate::{
    input::MappedInput,
    physics::{Facing, PhysicsParameters},
    Convert,
};

pub enum ControllerEnergyState {
    FallAdjust,
    Dash,
    Stand,
}

#[derive(Default)]
pub struct AirdodgeEnergy {
    velocity: Vec2,
    accel: Vec2,
    counter: usize,
    pub enabled: bool,
}

impl AirdodgeEnergy {
    pub fn initialize(&mut self, inputs: &MappedInput, params: &PhysicsParameters) {
        self.velocity = inputs.stick_l.normalize_or_zero() * params.air.airdodge_speed;
        self.accel = -self.velocity / 30.0;
        self.counter = 0;
    }

    pub fn update(&mut self) {
        if self.counter >= 30 {
            return;
        }

        self.velocity += self.accel;
    }

    pub fn velocity(&self) -> Vector2<f32> {
        if self.enabled {
            self.velocity.convert()
        } else {
            Vector2::zeros()
        }
    }
}

#[derive(Default)]
pub struct ControllerEnergy {
    velocity: f32,
    brake: f32,
    accel_fixed: f32,
    accel_stick: f32,
    speed_max: f32,
    speed_cap: f32,
    lr: f32,
    pub enabled: bool,
}

impl ControllerEnergy {
    pub fn initialize(
        &mut self,
        transfer_x: f32,
        facing: Facing,
        params: &PhysicsParameters,
        kind: ControllerEnergyState,
    ) {
        match kind {
            ControllerEnergyState::FallAdjust => {
                self.brake = params.air.air_brake;
                self.accel_fixed = 0.0;
                self.accel_stick = params.air.air_accel;
                self.speed_max = params.air.max_air_speed;
                self.speed_cap = params.ground.run_speed * params.air.ground_to_air;
                self.lr = facing.sign();
                self.velocity = transfer_x;
            }
            ControllerEnergyState::Dash => {
                self.brake = params.ground.friction;
                self.accel_fixed = params.ground.run_accel;
                self.accel_stick = params.ground.run_accel_mul;
                self.speed_max = params.ground.run_speed;
                self.speed_cap = 20.0;
                self.lr = facing.sign();
                self.velocity = transfer_x + params.ground.initial_speed * self.lr;
            }
            ControllerEnergyState::Stand => {
                self.brake = params.ground.friction;
                self.accel_fixed = 0.0;
                self.accel_stick = 0.0;
                self.speed_max = 0.0;
                self.speed_cap = 20.0;
                self.lr = facing.sign();
                self.velocity = transfer_x;
            }
        }
    }

    pub fn update(&mut self, inputs: &MappedInput) {
        self.velocity = self.velocity.signum() * self.velocity.abs().min(self.speed_cap);
        if self.velocity.abs() > self.speed_max {
            self.velocity =
                self.velocity.signum() * (self.velocity.abs() - self.brake).max(self.speed_max);
        }

        let increase_max = self.velocity.abs().max(self.speed_max);

        let new_vel =
            self.velocity + self.accel_fixed * self.lr + self.accel_stick * inputs.stick_l.x;
        self.velocity = new_vel.abs().min(increase_max) * new_vel.signum();
    }

    pub fn velocity(&self) -> Vector2<f32> {
        if self.enabled {
            Vector2::new(self.velocity, 0.0)
        } else {
            Vector2::zeros()
        }
    }
}

pub enum GravityEnergyState {
    Jump,
    Fall,
}

#[derive(Default)]
pub struct GravityEnergy {
    velocity: f32,
    accel: f32,
    max_fall_speed: f32,
    pub enabled: bool,
}

impl GravityEnergy {
    pub fn initialize(
        &mut self,
        transfer_y: f32,
        params: &PhysicsParameters,
        kind: GravityEnergyState,
    ) {
        match kind {
            GravityEnergyState::Jump => {
                self.velocity = (params.air.jump_height * params.air.gravity * 2.0).sqrt();
                self.accel = -params.air.gravity;
                self.max_fall_speed = params.air.fall_speed;
            }
            GravityEnergyState::Fall => {
                self.velocity = transfer_y;
                self.accel = -params.air.gravity;
                self.max_fall_speed = params.air.fall_speed;
            }
        }
    }

    pub fn update(&mut self) {
        if self.velocity <= -self.max_fall_speed {
            return;
        }

        self.velocity += self.accel;
    }

    pub fn velocity(&self) -> Vector2<f32> {
        if self.enabled {
            Vector2::new(0.0, self.velocity)
        } else {
            Vector2::zeros()
        }
    }
}
