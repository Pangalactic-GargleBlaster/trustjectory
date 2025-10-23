use futures::{executor::{block_on, LocalPool}, stream::StreamExt, task::{LocalSpawnExt, SpawnExt}, Stream};
use gamepad::*;
use r2r::{Publisher, QosProfile};
use std::{f64, time::{Duration, Instant}};
use r2r::trajectory_msgs::msg::JointTrajectoryPoint;
mod trajectories;
use crate::trajectories::{equally_spaced_trajectory, high_jerk_trajectory, write_position_list_to_file, JointPosition, JointPositionExt, Trajectory, TrajectoryExt, ARM_DEGREES_OF_FREEDOM, HOME_POSITION, MAX_ANGLES, MIN_ANGLES};

const QOS_PROFILE: QosProfile = QosProfile::sensor_data().reliable().keep_last(1);
const ROS_SAMPLING_PERIOD: Duration = Duration::from_millis(16);
const STARTING_DISTANCE_THRESHOLD: f64 = 3.0;

pub fn main() {
    let ctx = r2r::Context::create().expect("Couldn't initialize ros");
    let mut node = r2r::Node::create(ctx, "teach_pendant", "trustjectory").expect("Couldn't create the ros node");
    let mut subscription = node.subscribe::<JointTrajectoryPoint>("/robot_state", QOS_PROFILE).expect("Failed to subscribe to robot state topic");
    let publisher: Publisher<JointTrajectoryPoint> = node.create_publisher::<JointTrajectoryPoint>("/robot_commands", QOS_PROFILE).expect("Failed to create a publisher for the robot commands topic");
    println!("Created publisher and subscriber");

    if std::env::args().collect::<Vec<String>>().contains(&"publish_test".to_string()) {
        let mut count = 0;
        loop {
            publisher.send_command_to_qarm([count as f64, 0.0,0.0,0.0,0.0]);
            count += 1;
            std::thread::sleep(ROS_SAMPLING_PERIOD);
        }
    }

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    spawner.spawn_local(async move {
        publisher.wait_for_inter_process_subscribers().expect("error before awaiting").await.expect("error in waiting for subscribers");
        if std::env::args().collect::<Vec<String>>().contains(&"jerk_test".to_string()) {
            println!("Running high jerk test");
            execute_trajectory(&high_jerk_trajectory(Duration::from_secs(60)), &publisher, &mut subscription).await
        }
        println!("Teach pendant ready!");
        teach_pendant(publisher, subscription).await
    }).expect("Failed to spawn a task");
    
    let mut middleware_heartbeat_counter: u32 = 0;
    loop {
        node.spin_once(ROS_SAMPLING_PERIOD);
        pool.run_until_stalled();
        middleware_heartbeat_counter += 1;
        if middleware_heartbeat_counter % 1024 == 0 {
            println!("Still running ROS middleware");
        }
    }
}

async fn teach_pendant(publisher: Publisher<JointTrajectoryPoint>, mut subscription: impl Stream<Item = JointTrajectoryPoint> + Unpin) {
    let mut gamepad_engine = GamepadEngine::new();
    let initial_position: JointPosition = get_robot_state(&mut subscription).await;
    assert!(HOME_POSITION.euclidian_distance_to(initial_position) <= STARTING_DISTANCE_THRESHOLD, "Starting too far from the home position is unsafe");
    let mut loop_heartbeat_counter: u32 = 0;
    let mut trajectory_counter: u32 = 1;
    let mut current_position_list: Vec<JointPosition> = vec![HOME_POSITION.into()];
    let mut last_iteration = Instant::now();
    let mut robot_command: JointPosition = HOME_POSITION;
    loop {
        let robot_state: JointPosition = get_robot_state(&mut subscription).await;
        gamepad_engine.update().expect("There was an error in updating the gamepad engine");
        if let Some(gamepad) = gamepad_engine.gamepads().iter().next() {
            let left_joystick: (f32, f32) = gamepad.joystick(Joystick::Left);
            let right_joystick: (f32, f32) = gamepad.joystick(Joystick::Right);
            let left_back_button = gamepad.is_pressed(Button::LeftShoulder);
            let right_back_button = gamepad.is_pressed(Button::RightShoulder);
            let speed_factor = last_iteration.elapsed().as_secs_f64()/2.0; // target 0.5 rad/s
            if gamepad_is_idle(left_joystick, right_joystick, left_back_button, right_back_button) {
                robot_command = robot_state;
            } else {
                robot_command[0] = (robot_command[0] + left_joystick.0 as f64 * speed_factor).clamp(MIN_ANGLES[0], MAX_ANGLES[0]);
                robot_command[1] = (robot_command[1] - left_joystick.1 as f64 * speed_factor).clamp(MIN_ANGLES[1], MAX_ANGLES[1]);
                robot_command[2] = (robot_command[2] - right_joystick.1 as f64 * speed_factor).clamp(MIN_ANGLES[2], MAX_ANGLES[2]);
                robot_command[3] = (robot_command[3] - right_joystick.0 as f64 * speed_factor).clamp(MIN_ANGLES[3], MAX_ANGLES[3]);
                robot_command[4] = (robot_command[4] + (right_back_button as u8 as f64 - left_back_button as u8 as f64) * speed_factor).clamp(MIN_ANGLES[4], MAX_ANGLES[4]);
            }
            publisher.send_command_to_qarm(robot_command);
            if gamepad.is_just_pressed(Button::West) {
                println!("Adding point {robot_state:?} to the trajectory");
                current_position_list.push(robot_state);
            }
            if gamepad.is_just_pressed(Button::East) {
                write_position_list_to_file(&current_position_list, &("trajectory".to_string()+&trajectory_counter.to_string()));
                trajectory_counter += 1;
                execute_trajectory(&equally_spaced_trajectory(&current_position_list).invert(), &publisher, &mut subscription).await;
                current_position_list = vec![HOME_POSITION];
            }

            loop_heartbeat_counter += 1;
            if loop_heartbeat_counter % 1024 == 0 {
                println!("Still publishing successfully");
            }
        }
        last_iteration = Instant::now();
    }
}

fn gamepad_is_idle(left_joystick: (f32, f32), right_joystick: (f32, f32), left_back_button: bool, right_back_button: bool) -> bool {
    return left_joystick == (0.0,0.0) && right_joystick == (0.0,0.0) && left_back_button == false && right_back_button == false;
}

async fn execute_trajectory(trajectory: &Trajectory, publisher: &Publisher<JointTrajectoryPoint>, subscription: &mut (impl Stream<Item = JointTrajectoryPoint> + Unpin)) {
    println!("Executing a saved trajectory with {} points", trajectory.len());
    let trajectory_start_time = Instant::now();
    publisher.send_command_to_qarm(trajectory[0].joint_position);
    let trajectory_duration = trajectory.last().unwrap().time_from_start;
    loop {
        subscription.next().await; // Use the /robot_state topic to time new commands
        let current_time = trajectory_start_time.elapsed();
        if current_time > trajectory_duration {break;}
        publisher.send_command_to_qarm(trajectory[trajectory.segment_index(current_time)].joint_position);
    }
    publisher.send_command_to_qarm(trajectory.last().unwrap().joint_position);
}

trait MoveQarm {
    fn send_command_to_qarm(&self, joint_position: JointPosition);
}

impl MoveQarm for Publisher<JointTrajectoryPoint> {
    fn send_command_to_qarm(&self, joint_position: JointPosition) {
        self.publish(&JointTrajectoryPoint{
            positions: joint_position.to_vec(),
            velocities: vec![],
            accelerations: vec![],
            effort: vec![],
            time_from_start: r2r::builtin_interfaces::msg::Duration::default(),
        }).expect("Couldn't publish robot command");
    }
}

async fn get_robot_state(subscription: &mut (impl Stream<Item = JointTrajectoryPoint> + Unpin)) -> JointPosition {
    let mut robot_state: JointPosition = [0.0;ARM_DEGREES_OF_FREEDOM];
    robot_state.copy_from_slice(&subscription.next().await.expect("The subscription was closed").positions);
    return robot_state;
}