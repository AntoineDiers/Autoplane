mod hmi_bridge;

use futures::executor::LocalPool;
use hmi_bridge::HmiBridge;

fn main()
{
    let mut node = r2r::Node::create(r2r::Context::create().unwrap(), "hmi_bridge", "").unwrap();
    let mut task_pool = LocalPool::new();

    let _hmi_bridge = HmiBridge::new(&mut node, &mut task_pool).unwrap();

    loop 
    {
        node.spin_once(std::time::Duration::from_millis(100));
        task_pool.run_until_stalled();
    }
}
