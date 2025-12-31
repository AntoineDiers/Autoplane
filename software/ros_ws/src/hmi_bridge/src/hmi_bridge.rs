use std::{net::UdpSocket, thread::spawn, time::{Duration, Instant}};

use anyhow::Result;
use autoplane_hmi_lib::{AUTOPLANE_UDP_RCV_PORT, HMI_UDP_RCV_PORT, Message, udp_sender::UdpSender};
use futures::{StreamExt, executor::LocalPool, future, task::LocalSpawnExt};
use r2r::{Node, QosProfile, WrappedTypesupport};


pub struct HmiBridge
{
    udp_sender : UdpSender,
}

impl HmiBridge
{

    pub fn new(node : &mut Node, task_pool : &mut LocalPool) -> Result<HmiBridge>
    {
        let hmi_ip : String = node.get_parameter("hmi_ip").unwrap();
        let udp_sender= UdpSender::new(hmi_ip + ":" + &HMI_UDP_RCV_PORT.to_string());
        
        let mut hmi_bridge = HmiBridge 
        {
            udp_sender,
        };

        hmi_bridge.start_transmitting_message::<r2r::std_msgs::msg::Float64>("~/roll", node, task_pool, Some(Duration::from_millis(100)), 
        |msg| { return Message::ROLL(msg.data); })?;

        hmi_bridge.start_transmitting_message::<r2r::std_msgs::msg::Float64>("~/pitch", node, task_pool, Some(Duration::from_millis(100)), 
        |msg| { return Message::PITCH(msg.data); })?;

        hmi_bridge.start_transmitting_message::<r2r::std_msgs::msg::Float64>("~/heading", node, task_pool, Some(Duration::from_millis(100)), 
        |msg| { return Message::HEADING(msg.data); })?;

        hmi_bridge.start_transmitting_message::<r2r::std_msgs::msg::Float64>("~/latitude", node, task_pool, Some(Duration::from_millis(100)), 
        |msg| { return Message::LATITUDE(msg.data); })?;

        hmi_bridge.start_transmitting_message::<r2r::std_msgs::msg::Float64>("~/longitude", node, task_pool, Some(Duration::from_millis(100)), 
        |msg| { return Message::LONGITUDE(msg.data); })?;

        hmi_bridge.start_transmitting_message::<r2r::std_msgs::msg::Float64>("~/altitude", node, task_pool, Some(Duration::from_millis(100)), 
        |msg| { return Message::ALTITUDE(msg.data); })?;

        hmi_bridge.start_udp_listen_thread()?;

        Ok(hmi_bridge)
    }

    fn start_transmitting_message<T : WrappedTypesupport + 'static>(
        &mut self,
        topic : &str,
        node : &mut Node, 
        task_pool : &mut LocalPool,
        min_period : Option<Duration>,
        translator : fn(T) -> Message) -> Result<()>
    {
        let subscriber = node.subscribe::<T>(topic, QosProfile::default())?;
        let mut udp_sender = self.udp_sender.clone();
        let mut last_send = None;

        task_pool.spawner().spawn_local(async move 
        {
            subscriber.for_each(|msg| 
            {
                match (min_period, last_send)
                {
                    (Some(min_period), Some(last_send)) if last_send + min_period > Instant::now() => {}
                    _ => 
                    {
                        last_send = Some(Instant::now());
                        udp_sender.send_message(translator(msg));
                    }
                }
                future::ready(())
            }).await
        })?;

        Ok(())
    }

    fn start_udp_listen_thread(&mut self) -> Result<()>
    {
        let udp_socket = UdpSocket::bind("0.0.0.0:".to_string() + &AUTOPLANE_UDP_RCV_PORT.to_string())?;
        let mut udp_sender = self.udp_sender.clone();

        spawn(move ||
        {
            let mut udp_buffer = vec![0;65535];
            loop
            {
                match udp_socket.recv_from(&mut udp_buffer)
                {
                    Ok((size,_)) => 
                    {
                        match serde_json::from_slice::<Message>(&udp_buffer[..size]) 
                        {
                            Ok(Message::PING(data)) => { udp_sender.send_message(Message::PING(data)); } 
                            Ok(msg) => { println!("Received unexpected message from HMI : {:?}", msg); }
                            Err(err) => { println!("Failed to deserialise udp data {:?}", err); }
                        }
                    }
                    Err(e) => { println!("Error when receiving UDP data : {:?}", e); }
                    
                }            
            }
        });

        Ok(())
    }
}