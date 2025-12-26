use serde::{Deserialize, Serialize};
use std::time::Duration;

pub mod udp_sender;

pub const HMI_UDP_RCV_PORT : u16 = 8001;
pub const AUTOPLANE_UDP_RCV_PORT : u16 = 8002; 

#[derive(Serialize, Deserialize, Debug)]
pub enum Message
{
    PING(Duration),
    ROLL(f64),
    PITCH(f64)
}