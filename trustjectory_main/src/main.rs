use futures::{executor::{block_on, LocalPool}, task::LocalSpawnExt};
use gamepad::*;
use r2r::{std_msgs::msg::Header, trajectory_msgs::msg::JointTrajectory, Publisher, QosProfile};
use std::{cell::Cell, f64, fs::{self, File}, path::{Path, PathBuf}, process::exit, rc::Rc, time::{Duration, Instant}, vec};
use r2r::trajectory_msgs::msg::JointTrajectoryPoint;
mod trajectories;
use crate::trajectories::{high_jerk_trajectory, JointPosition, ParametricTrajectory, PointTrajectory, PointTrajectoryExt, ParametricTrajectoryExt, TrajectoryPoint, HOME_POSITION, MAX_ANGLES, MIN_ANGLES};

const QOS_PROFILE: QosProfile = QosProfile::sensor_data().reliable().keep_last(1);
const ROS_SAMPLING_PERIOD: Duration = Duration::from_millis(16);

pub fn main() {
    let ctx = r2r::Context::create().expect("Couldn't initialize ros");
    let mut node = r2r::Node::create(ctx, "teach_pendant", "trustjectory").expect("Couldn't create the ros node");
    let publisher: Publisher<JointTrajectory> = node.create_publisher::<JointTrajectory>("/robot_commands", QOS_PROFILE).expect("Failed to create a publisher for the robot commands topic");
    println!("Created publisher");

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    spawner.spawn_local(async move {
        publisher.wait_for_inter_process_subscribers().expect("error before awaiting").await.expect("error in waiting for subscribers");
        println!("connected to subscriber");
        if std::env::args().collect::<Vec<String>>().contains(&"hardcoded_trajectory".to_string()) {
            let point_list = vec![
                JointPosition(HOME_POSITION.0),
                JointPosition([1.0,0.0,0.0,0.0,0.5]),
                JointPosition([0.0,0.0,-1.0,0.0,0.5]),
                JointPosition([-1.0,0.0,0.0,0.0,0.5]),
                JointPosition([0.0,0.0,1.0,0.0,0.5]),
                JointPosition([1.0,0.0,0.0,0.0,0.5]),
                JointPosition([0.0,0.0,-1.0,0.0,0.5]),
                JointPosition([-1.0,0.0,0.0,0.0,0.5]),
                JointPosition([0.0,0.0,1.0,0.0,0.5]),
            ];
            let point_trajectory: PointTrajectory = PointTrajectory::from_parametric_trajectory(&ParametricTrajectory::from_position_list(&point_list));
            save_trajectory(&point_trajectory, &PathBuf::from("trajectories").join("hardcoded_trajectory"));
            publisher.send_trajectory_to_qarm(&point_trajectory);
            std::thread::sleep(Duration::from_secs(1));
            exit(0);
        }
        if std::env::args().collect::<Vec<String>>().contains(&"jerk_test".to_string()) {
            println!("Running high jerk test");
            publisher.send_trajectory_to_qarm(&high_jerk_trajectory(Duration::from_secs(60)));
        }
        println!("Teach pendant ready!");
        teach_pendant(publisher);
    }).expect("Failed to spawn a task");
    
    loop {
        pool.run_until_stalled();
        node.spin_once(ROS_SAMPLING_PERIOD);
    }
}

fn teach_pendant(publisher: Publisher<JointTrajectory>) {
    let mut gamepad_engine = GamepadEngine::new();
    let mut loop_heartbeat_counter: u32 = 0;
    let mut trajectory_counter: u32 = 1;
    let mut current_position_list: Vec<JointPosition> = vec![HOME_POSITION.into()];
    let mut last_iteration = Instant::now();
    let mut robot_command: JointPosition = HOME_POSITION;
    loop {
        std::thread::sleep(ROS_SAMPLING_PERIOD);
        gamepad_engine.update().expect("There was an error in updating the gamepad engine");
        if let Some(gamepad) = gamepad_engine.gamepads().iter().next() {
            let left_joystick: (f32, f32) = gamepad.joystick(Joystick::Left);
            let right_joystick: (f32, f32) = gamepad.joystick(Joystick::Right);
            let left_back_button = gamepad.is_pressed(Button::LeftShoulder);
            let right_back_button = gamepad.is_pressed(Button::RightShoulder);
            let speed_factor = last_iteration.elapsed().as_secs_f64()/2.0; // target 0.5 rad/s
            robot_command[0] = (robot_command[0] + left_joystick.0 as f64 * speed_factor).clamp(MIN_ANGLES[0], MAX_ANGLES[0]);
            robot_command[1] = (robot_command[1] - left_joystick.1 as f64 * speed_factor).clamp(MIN_ANGLES[1], MAX_ANGLES[1]);
            robot_command[2] = (robot_command[2] - right_joystick.1 as f64 * speed_factor).clamp(MIN_ANGLES[2], MAX_ANGLES[2]);
            robot_command[3] = (robot_command[3] - right_joystick.0 as f64 * speed_factor).clamp(MIN_ANGLES[3], MAX_ANGLES[3]);
            robot_command[4] = (robot_command[4] + (right_back_button as u8 as f64 - left_back_button as u8 as f64) * speed_factor).clamp(MIN_ANGLES[4], MAX_ANGLES[4]);
            publisher.send_position_to_qarm(&robot_command);
            if gamepad.is_just_pressed(Button::West) {
                println!("Adding point {robot_command:?} to the trajectory");
                current_position_list.push(robot_command);
            }
            if gamepad.is_just_pressed(Button::East) {
                let folder_path: PathBuf = PathBuf::from("trajectories").join(format!("trajectory{trajectory_counter}"));
                trajectory_counter += 1;
                let point_trajectory: PointTrajectory = PointTrajectory::from_parametric_trajectory(&ParametricTrajectory::from_position_list(&current_position_list));
                save_trajectory(&point_trajectory, &folder_path);
                let return_home_trajectory: PointTrajectory = point_trajectory.inverted();
                publisher.send_trajectory_to_qarm(&return_home_trajectory);
                let duration = return_home_trajectory.last().unwrap().time_from_start;
                println!("Returning home in {} seconds", duration.as_secs_f64());
                std::thread::sleep(duration);
                current_position_list = vec![HOME_POSITION];
                robot_command = HOME_POSITION;
            }

            loop_heartbeat_counter += 1;
            if loop_heartbeat_counter % 1024 == 0 {
                println!("Still publishing successfully");
            }
        }
        last_iteration = Instant::now();
    }
}

fn save_trajectory(trajectory: &PointTrajectory, folder_path: &Path) {
    fs::create_dir_all(folder_path);
    trajectory.save_to_file(&folder_path.join("trajectory.json"));
    trajectory.plot_positions_velocities_and_accelerations(&folder_path);
}

trait MoveQarm {
    fn send_trajectory_to_qarm(&self, trajectory: &PointTrajectory);
    fn send_position_to_qarm(&self, position: &JointPosition);
}

impl MoveQarm for Publisher<JointTrajectory> {
    fn send_trajectory_to_qarm(&self, trajectory: &PointTrajectory) {
        let mut ros_trajectory: Vec<JointTrajectoryPoint> = Vec::with_capacity(trajectory.len());
        for trajectory_point in trajectory {
            ros_trajectory.push(JointTrajectoryPoint {
                positions: trajectory_point.joint_position.0.to_vec(),
                velocities: vec![],
                accelerations: vec![],
                effort: vec![],
                time_from_start: std_to_ros_duration(trajectory_point.time_from_start)
            });
        }
        self.publish(&JointTrajectory{
            points: ros_trajectory,
            header: Header::default(),
            joint_names: vec![]
        }).expect("Couldn't publish robot command");
    }
    
    fn send_position_to_qarm(&self, position: &JointPosition) {
        let mut trajectory: PointTrajectory = Vec::with_capacity(2);
        for _ in 0..2{
            trajectory.push(TrajectoryPoint{joint_position: position.clone(), time_from_start: Duration::ZERO});
        }
        self.send_trajectory_to_qarm(&trajectory);
    }
}

fn std_to_ros_duration(value: Duration) -> r2r::builtin_interfaces::msg::Duration {
    return r2r::builtin_interfaces::msg::Duration{
        sec: value.as_secs() as i32,
        nanosec: value.subsec_nanos(),
    };
}
