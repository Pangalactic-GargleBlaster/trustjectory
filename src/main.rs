use futures::{executor::block_on, StreamExt};
use gamepad::*;
use r2r::{Node, Publisher, QosProfile};
use std::{f64, sync::{Arc, Mutex}, time::Duration};
use r2r::trajectory_msgs::msg::JointTrajectoryPoint;

const QOS_PROFILE: QosProfile = QosProfile::sensor_data().reliable();
const GAMEPAD_SAMPLING_PERIOD: Duration = Duration::from_millis(10);
const ROS_SAMPLING_PERIOD: Duration = Duration::from_millis(10);
const JOYSTICK_SPEED_FACTOR: f64 = GAMEPAD_SAMPLING_PERIOD.as_secs_f64(); // targeting 1 rad/s
const DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR: f64 = f64::consts::PI/180.0;
const MAX_ANGLES : [f64; 4] = [
    170.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    85.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    75.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    160.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
];

async fn teach_pendant() {
    let mut gamepad_engine = GamepadEngine::new();
    let ctx = r2r::Context::create().expect("Couldn't initialize ros");
    let node: Arc<Mutex<Node>> = Arc::new(Mutex::new(
        r2r::Node::create(ctx, "teach_pendant", "trustjectory").expect("Couldn't create the ros node")
    ));
    let node_clone = node.clone();
    let mut middleware_heartbeat_counter: u32 = 0;
    std::thread::spawn(move || loop {
        node_clone.lock().unwrap().spin_once(Duration::ZERO);
        middleware_heartbeat_counter += 1;
        if middleware_heartbeat_counter % 1024 == 0 {
            println!("Still running ROS middleware");
        }
        std::thread::sleep(ROS_SAMPLING_PERIOD);
    });
    let mut publisher = initialize_publisher(node.clone()).await;
    let mut desired_robot_state = JointTrajectoryPoint{
        positions: vec![0.0; 5],
        velocities: vec![0.0; 4],
        accelerations: vec![0.0; 4],
        effort: vec![0.0; 4],
        time_from_start: r2r::builtin_interfaces::msg::Duration::default(),
    };
    let mut subscription = node.lock().unwrap().subscribe::<JointTrajectoryPoint>("/robot_state", QOS_PROFILE).expect("Unable to subscribe to robot state topic");
    println!("Registered a subscriber!");
    desired_robot_state.positions[0..5].copy_from_slice(&subscription.next().await.expect("No initial position received").positions[0..5]);
    println!("Initial positions: {:?}", desired_robot_state.positions);
    let mut loop_heartbeat_counter: u32 = 0;
    println!("Teach pendant ready!");
    loop {
        gamepad_engine.update().expect("There was an error in updating the gamepad engine");
        if let Some(gamepad) = gamepad_engine.gamepads().iter().next() {
            let left_joystick: (f32, f32) = gamepad.joystick(Joystick::Left);
            let right_joystick: (f32, f32) = gamepad.joystick(Joystick::Right);
            desired_robot_state.positions[0] += left_joystick.0 as f64 * JOYSTICK_SPEED_FACTOR;
            desired_robot_state.positions[1] += left_joystick.1 as f64 * JOYSTICK_SPEED_FACTOR;
            desired_robot_state.positions[2] += right_joystick.0 as f64 * JOYSTICK_SPEED_FACTOR;
            desired_robot_state.positions[3] += right_joystick.1 as f64 * JOYSTICK_SPEED_FACTOR;
            clip_desired_positions(&mut desired_robot_state.positions);
            if gamepad.is_just_pressed(Button::South){
                desired_robot_state.positions[4] = 1.0-desired_robot_state.positions[4];
            }
        }
        match publisher.publish(&desired_robot_state){
            Ok(_) => {
                loop_heartbeat_counter += 1;
                if loop_heartbeat_counter % 1024 == 0 {
                    println!("Still publishing successfully");
                }
            },
            Err(_) => publisher = initialize_publisher(node.clone()).await,
        }
        std::thread::sleep(GAMEPAD_SAMPLING_PERIOD);
    }
}

async fn initialize_publisher(node: Arc<Mutex<Node>>) -> Publisher<JointTrajectoryPoint> {
    let publisher = node.lock().unwrap()
        .create_publisher::<JointTrajectoryPoint>("/robot_commands", QOS_PROFILE)
        .expect("Couldn't create a publisher fot the robot commands topic");
    println!("Registered a publisher");
    while let Err(_) = publisher.wait_for_inter_process_subscribers().unwrap().await{
        println!("Failed to establish a connection to a subscriber. Trying again");
    }
    println!("The publisher got a subscriber");
    publisher
}
pub fn main() {
    block_on(teach_pendant());
}

fn clip_desired_positions(desired_positions: &mut Vec<f64>) {
    for index in 0..MAX_ANGLES.len() {
        let max_angle = MAX_ANGLES[index];
        if desired_positions[index] < -max_angle {
            desired_positions[index] = -max_angle;
        }else if desired_positions[index] > max_angle {
            desired_positions[index] = max_angle;
        }
    }
}