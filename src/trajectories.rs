use std::{f64, fs::File, path::Path, time::Duration};
use std::io::Write;


pub const ARM_DEGREES_OF_FREEDOM: usize = 5;
const DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR: f64 = f64::consts::PI/180.0;
pub const MAX_ANGLES: [f64; ARM_DEGREES_OF_FREEDOM] = [
    170.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    85.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    75.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    160.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    0.9
];
pub const MIN_ANGLES: [f64; ARM_DEGREES_OF_FREEDOM] = [
    -170.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    -85.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    -95.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    -160.0*DEGREES_TO_RADIANS_MULTIPLICATIVE_FACTOR,
    0.1
];
// const MAX_VELOCITIES: [f64; ARM_DEGREES_OF_FREEDOM] = [f64::consts::PI/2.0, f64::consts::PI/2.0, f64::consts::PI/2.0, f64::consts::PI/2.0, 1.0];
const MAX_ACCELERATIONS: [f64; ARM_DEGREES_OF_FREEDOM] = [f64::consts::PI/3.0, f64::consts::PI/3.0, f64::consts::PI/3.0, f64::consts::PI/3.0, 4.0];
const BASIC_TRAJECTORY_INTER_POINT_DELAY: Duration = Duration::from_secs(3);

pub type JointPosition = [f64;ARM_DEGREES_OF_FREEDOM];
pub const HOME_POSITION:JointPosition = [0.0, 0.0, 0.0, 0.0, 0.5];
pub struct TrajectoryPoint {
    pub joint_position: JointPosition,
    pub time_from_start: Duration,
}
pub type Trajectory = Vec<TrajectoryPoint>;
pub trait TrajectoryExt {
    fn invert(&self) -> Self;
    fn segment_index(&self, time_from_start: Duration) -> usize;
}
impl TrajectoryExt for Trajectory{
    fn invert(&self) -> Self {
        match self.len() {
            0 => return Trajectory::new(),
            length => {
                let total_duration = self[length-1].time_from_start;
                let mut inverted_trajectory = Trajectory::new();
                for trajectory_point in self {
                    inverted_trajectory.push(TrajectoryPoint{
                        joint_position: trajectory_point.joint_position,
                        time_from_start: total_duration - trajectory_point.time_from_start
                    });
                }
                return inverted_trajectory;
            }
        }
    }

    fn segment_index(&self, time_from_start: Duration) -> usize {
        let mut starting_point_index: usize = 0;
        let mut ending_point_index: usize = self.len()-1;
        while ending_point_index - starting_point_index > 1 {
            let midpoint_index = (starting_point_index + ending_point_index)/2;
            if self[midpoint_index].time_from_start < time_from_start {
                ending_point_index = midpoint_index;
            } else {
                starting_point_index = midpoint_index;
            }
        }
        return starting_point_index;
    }
}

pub trait JointPositionExt {
    fn euclidian_distance_to(&self, other_position: Self) -> f64;
}

impl JointPositionExt for JointPosition {
    fn euclidian_distance_to(&self, other_position: Self) -> f64 {
        let mut distance: f64 = 0.0;
        for index in 0..ARM_DEGREES_OF_FREEDOM {
            distance += (self[index] - other_position[index]).powi(2);
        }
        return distance;
    }
}

pub fn equally_spaced_trajectory(points: &Vec<JointPosition>) -> Trajectory {
    let mut current_point_time = Duration::ZERO;
    let mut trajectory: Trajectory = vec![];
    for point in points {
        trajectory.push(TrajectoryPoint{
            joint_position: *point,
            time_from_start: current_point_time
        });
        current_point_time += BASIC_TRAJECTORY_INTER_POINT_DELAY;
    }
    return trajectory;
}

const HIGH_JERK_TRAJECTORY_SAMPLING_PERIOD: Duration = Duration::from_millis(1);
pub fn high_jerk_trajectory(duration: Duration) -> Trajectory {
    let mut current_position: JointPosition = HOME_POSITION;
    let mut current_velocity: f64 = 0.0;
    let mut time_so_far = Duration::ZERO;
    let mut trajectory: Trajectory = vec![TrajectoryPoint{
        joint_position: current_position,
        time_from_start: time_so_far
    }];

    // initial acceleration to one side
    let inflection_point: f64 = MAX_ANGLES[0]/4.0;
    while current_position[0] < inflection_point {
        current_velocity += MAX_ACCELERATIONS[0] * HIGH_JERK_TRAJECTORY_SAMPLING_PERIOD.as_secs_f64();
        current_position[0] += current_velocity * HIGH_JERK_TRAJECTORY_SAMPLING_PERIOD.as_secs_f64();
        time_so_far += HIGH_JERK_TRAJECTORY_SAMPLING_PERIOD;
        trajectory.push(TrajectoryPoint{
            joint_position: current_position,
            time_from_start: time_so_far
        });
    }

    // high-jerk oscillation
    while time_so_far < duration {
        let acceleration_sign = if current_position[0] > 0.0 {-1.0} else {1.0};
        current_velocity += acceleration_sign * MAX_ACCELERATIONS[0] * HIGH_JERK_TRAJECTORY_SAMPLING_PERIOD.as_secs_f64();
        current_position[0] += current_velocity * HIGH_JERK_TRAJECTORY_SAMPLING_PERIOD.as_secs_f64();
        time_so_far += HIGH_JERK_TRAJECTORY_SAMPLING_PERIOD;
        trajectory.push(TrajectoryPoint{
            joint_position: current_position,
            time_from_start: time_so_far
        });
    }
    return trajectory;
}

pub fn write_position_list_to_file(trajectory: &Vec<JointPosition>, file_name: &str) {
    println!("Writing trajectory to file");
    let json_string = serde_json::to_string_pretty(trajectory).expect("Failed to serialize a trajectory");
    let mut file = File::create(Path::new(&("trajectories/".to_string()+file_name))).expect("Failed to create the trajectory file");
    _ = file.write_all(json_string.as_bytes());
}