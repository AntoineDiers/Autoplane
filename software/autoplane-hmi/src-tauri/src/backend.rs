use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{net::UdpSocket, thread::spawn};
use anyhow::Result;

use autoplane_hmi_lib::{AUTOPLANE_UDP_RCV_PORT, Message};
use autoplane_hmi_lib::HMI_UDP_RCV_PORT;
use autoplane_hmi_lib::udp_sender::UdpSender;
use tauri::Emitter;

const TIMEOUT_MS : u128 = 3000;

pub struct Backend 
{
    app_handle : tauri::AppHandle,
    start_time : Instant,
    udp_sender : UdpSender,

    ping_ms : Option<u128>,
    last_ping : Option<Instant>
}

impl Backend
{
    pub fn new(app_handle : tauri::AppHandle) -> Result<Arc<Mutex<Backend>>>
    {
        let res = Arc::new(Mutex::new(Backend
        {
            app_handle,
            start_time : Instant::now(),
            udp_sender : UdpSender::new("192.168.1.10:".to_owned() + AUTOPLANE_UDP_RCV_PORT.to_string().as_str()),
            ping_ms : None,
            last_ping : None,
        }));

        // Start UDP Listen thread
        let res_clone = res.clone();
        spawn(move ||
        {
            let udp_socket = UdpSocket::bind("0.0.0.0:".to_string() + &HMI_UDP_RCV_PORT.to_string()).unwrap();
            let mut udp_buffer = vec![0;65535];
            loop
            {
                let (size,_) = udp_socket.recv_from(&mut udp_buffer).unwrap();

                match serde_json::from_slice::<Message>(&udp_buffer[..size]) 
                {
                    Ok(msg) => { res_clone.lock().unwrap().on_udp_message(msg); }
                    Err(err) => { println!("Failed to deserialise udp data {:?}", err); }
                }
            }
        });

        // Send pings and update connection state
        let res_clone = res.clone();
        spawn(move ||
        {
            loop 
            {
                res_clone.lock().unwrap().update_connection_state();
                sleep(Duration::from_secs(1));    
            }
        });

        // Return the result
        Ok(res)
    }

    fn on_udp_message(&mut self, message : Message)
    {
        match message
        {
            Message::PING(stamp) => 
            {
                let ping_ms = self.start_time.elapsed().as_millis() - stamp.as_millis();
                if ping_ms < TIMEOUT_MS 
                {
                    self.ping_ms = Some(ping_ms);
                    self.last_ping = Some(Instant::now());
                }
            }
            Message::ROLL(data) =>      { self.app_handle.emit("roll", data).unwrap(); },
            Message::PITCH(data) =>     { self.app_handle.emit("pitch", data).unwrap(); },
            Message::HEADING(data) =>   { self.app_handle.emit("heading", data).unwrap(); },
            Message::LATITUDE(data) =>  { self.app_handle.emit("latitude", data).unwrap(); },
            Message::LONGITUDE(data) => { self.app_handle.emit("longitude", data).unwrap(); },
            Message::ALTITUDE(data) =>  { self.app_handle.emit("altitude", data).unwrap(); },
        }
    }

    fn update_connection_state(&mut self)
    {
        self.udp_sender.send_message(Message::PING(self.start_time.elapsed()));

        match self.last_ping
        {
            Some(last_ping) if (Instant::now() - last_ping).as_millis() > TIMEOUT_MS => 
            {
                self.ping_ms = None;
            }
            _ => {}
        }

        self.app_handle.emit("ping", self.ping_ms).unwrap();
    }

}