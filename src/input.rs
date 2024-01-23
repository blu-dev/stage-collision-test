use bevy::prelude::*;
use bevy::{ecs::component::Component, input::gamepad::Gamepad, math::Vec2};

bitflags::bitflags! {
    #[derive(Debug, Copy, Clone, Default)]
    pub struct ButtonBitflags: u8 {
        const JUMP = 1 << 0;
        const SHIELD = 1 << 1;
    }
}

#[derive(Component)]
pub enum PlayerHandle {
    Keyboard,
    #[allow(dead_code)]
    Gamepad(Gamepad),
}

#[derive(Component, Default)]
pub struct MappedInput {
    pub stick_l: Vec2,
    pub stick_l_last: Vec2,
    pub buttons: ButtonBitflags,
    pub buttons_last: ButtonBitflags,
}

impl MappedInput {
    pub fn press(&self, buttons: ButtonBitflags) -> bool {
        (self.buttons & !self.buttons_last).contains(buttons)
    }
}

pub fn map_inputs(
    kb_input: Res<Input<KeyCode>>,
    buttons: Res<Input<GamepadButton>>,
    axes: Res<Axis<GamepadAxis>>,
    mut inputs: Query<(&PlayerHandle, &mut MappedInput)>,
) {
    for (handle, mut input) in inputs.iter_mut() {
        input.buttons_last = input.buttons;
        input.stick_l_last = input.stick_l;

        match handle {
            PlayerHandle::Keyboard => {
                let mut x = 0.0f32;
                x += kb_input.pressed(KeyCode::D) as u8 as f32;
                x -= kb_input.pressed(KeyCode::A) as u8 as f32;

                let mut y = 0.0f32;
                y += kb_input.pressed(KeyCode::W) as u8 as f32;
                y -= kb_input.pressed(KeyCode::S) as u8 as f32;

                input.buttons = ButtonBitflags::empty();
                input
                    .buttons
                    .set(ButtonBitflags::JUMP, kb_input.pressed(KeyCode::J));
                input
                    .buttons
                    .set(ButtonBitflags::SHIELD, kb_input.pressed(KeyCode::K));

                input.stick_l = Vec2::new(x, y);
            }
            PlayerHandle::Gamepad(gp) => {
                let x = axes
                    .get(GamepadAxis::new(*gp, GamepadAxisType::LeftStickX))
                    .unwrap_or_default();
                let y = axes
                    .get(GamepadAxis::new(*gp, GamepadAxisType::LeftStickY))
                    .unwrap_or_default();

                input.buttons = ButtonBitflags::empty();
                input.buttons.set(
                    ButtonBitflags::JUMP,
                    buttons.pressed(GamepadButton::new(*gp, GamepadButtonType::North)),
                );
                input.buttons.set(
                    ButtonBitflags::SHIELD,
                    buttons.pressed(GamepadButton::new(*gp, GamepadButtonType::RightTrigger2)),
                );

                input.stick_l = Vec2::new(x, y);
            }
        }
    }
}
