use serde::Serialize;

#[derive(Serialize, Clone)]
pub struct AttitudeEvent
{
    pub roll_deg : f32,
    pub pitch_deg : f32,
    pub yaw_deg : f32,
}