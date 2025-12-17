use tauri::Emitter;
use rand::prelude::*;

use crate::events::backend_events::attitude::AttitudeEvent;

pub struct Backend
{
    app_handle : tauri::AppHandle
}

impl Backend
{
    pub fn new(app_handle : tauri::AppHandle) -> Backend
    {
        Backend 
        { 
            app_handle
        }
    }

    pub fn tick(&mut self)
    {
        let mut rng = rand::rng();
        self.app_handle.emit("attitude", AttitudeEvent 
        {
            roll_deg :  rng.random_range(0.0..15.0),
            pitch_deg : rng.random_range(0.0..15.0),
            yaw_deg :   rng.random_range(0.0..15.0)
        }).unwrap();
    }
}