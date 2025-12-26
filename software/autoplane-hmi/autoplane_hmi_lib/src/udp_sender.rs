use std::{net::UdpSocket, sync::{Arc, Mutex}};
use crate::Message;

#[derive(Clone)]
pub struct UdpSender
{
    udp_socket : Arc<Mutex<UdpSocket>>,
    destination : String,
}

impl UdpSender
{
    pub fn new(destination : String) -> UdpSender
    {
        UdpSender 
        { 
            udp_socket : Arc::new(Mutex::new(UdpSocket::bind("0.0.0.0:0").unwrap())),
            destination,
        }
    }

    pub fn send_message(&mut self, msg : Message)
    {

        if let Err(e) = self.udp_socket.lock().unwrap().send_to(serde_json::to_string_pretty(&msg).unwrap().as_bytes(), self.destination.clone())
        {
            println!("Failed sending udp data to {:?} : {:?}", self.destination, e);
        }
    }
}
