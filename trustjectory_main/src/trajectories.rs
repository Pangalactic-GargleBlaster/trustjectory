use std::{f64, fs::File, path::Path, time::Duration};
use std::io::Write;
use nalgebra::{ArrayStorage, DMatrix, DVector, Matrix, SMatrix, SVector, VecStorage};
use vector_algebra_macro::VectorAlgebra;


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

#[derive(Copy, Clone, Debug, serde::Serialize, VectorAlgebra)]
pub struct JointPosition(pub [f64;ARM_DEGREES_OF_FREEDOM]);

#[derive(VectorAlgebra)]
pub struct JointVelocity(pub [f64;ARM_DEGREES_OF_FREEDOM]);

impl JointPosition {
    fn weighted_euclidian_distance_to(&self, other_position: Self, weights: [f64;ARM_DEGREES_OF_FREEDOM]) -> f64 {
        let mut distance: f64 = 0.0;
        for index in 0..ARM_DEGREES_OF_FREEDOM {
            distance += (self[index] - other_position[index]).powi(2) * weights[index];
        }
        return distance;
    }
}

impl From<JointPosition> for JointVelocity {
    fn from(value: JointPosition) -> Self {
        JointVelocity(value.0)
    }
}

pub const HOME_POSITION:JointPosition = JointPosition([0.0, 0.0, 0.0, 0.0, 0.5]);
#[derive(Debug)]
pub struct TrajectoryPoint {
    pub joint_position: JointPosition,
    pub time_from_start: Duration,
}
pub type Trajectory = Vec<TrajectoryPoint>;
pub trait TrajectoryExt {
    fn inverted(&self) -> Self;
}
impl TrajectoryExt for Trajectory{
    fn inverted(&self) -> Self {
        match self.len() {
            0 => return Trajectory::new(),
            length => {
                let total_duration = self[length-1].time_from_start;
                let mut inverted_trajectory = Trajectory::new();
                for trajectory_point in self.iter().rev() {
                    inverted_trajectory.push(TrajectoryPoint{
                        joint_position: trajectory_point.joint_position,
                        time_from_start: total_duration - trajectory_point.time_from_start
                    });
                }
                return inverted_trajectory;
            }
        }
    }
}

pub trait JointPositionExt {
    fn weighted_euclidian_distance_to(&self, other_position: Self, weights: [f64;ARM_DEGREES_OF_FREEDOM]) -> f64;
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

struct Cubic {
    coefficients: [f64;4],
}

impl Cubic {
    fn interpolate(&self, time: &Duration) -> f64 {
        let seconds = time.as_secs_f64();
        return self.coefficients[0] + seconds*self.coefficients[1] + seconds.powi(2)*self.coefficients[2] + seconds.powi(3)*self.coefficients[3];
    }
}

struct CubicTrajectorySegment {
    cubic: Cubic,
    end_time: Duration
}

type CubicTrajectory = Vec<CubicTrajectorySegment>;

trait CubicTrajectoryExt {
    fn segment_index(&self, time_from_start: Duration) -> usize;
    fn interpolate(&self, time_from_start: Duration) -> f64;
    fn from_position_list(points: &Vec<JointPosition>) -> Self;
}

impl CubicTrajectoryExt for CubicTrajectory {
    fn segment_index(&self, time_from_start: Duration) -> usize {
        let mut lower_bound_segment_index: usize = 0;
        let mut upper_bound_segment_index: usize = self.len()-1;
        while upper_bound_segment_index - lower_bound_segment_index > 1 {
            let midpoint_index = (lower_bound_segment_index + upper_bound_segment_index)/2;
            if self[midpoint_index].end_time < time_from_start {
                lower_bound_segment_index = midpoint_index;
            } else {
                upper_bound_segment_index = midpoint_index;
            }
        }
        return if time_from_start < self[lower_bound_segment_index].end_time {lower_bound_segment_index} else {upper_bound_segment_index};
    }

    fn interpolate(&self, time_from_start: Duration) -> f64 {
        let segment_index = self.segment_index(time_from_start);
        let local_time = if segment_index == 0 {time_from_start} else {time_from_start - self[segment_index-1].end_time};
        return self[segment_index].cubic.interpolate(&local_time);
    }
    
    fn from_position_list(points: &Vec<JointPosition>) -> Self {
        let times_from_start = relative_times_from_position_list(points);
        let mut velocities: Vec<JointVelocity> = Vec::with_capacity(points.len());
        velocities.push(JointVelocity([0.0;ARM_DEGREES_OF_FREEDOM]));
        for index in 1..points.len()-1 {
            velocities.push((
                (points[index+1]-points[index])
                * ((times_from_start[index] - times_from_start[index-1])
                / (times_from_start[index+1] - times_from_start[index])
                / (times_from_start[index+1] - times_from_start[index-1]))
                + (points[index]-points[index-1])
                * ((times_from_start[index+1] - times_from_start[index])
                / (times_from_start[index] - times_from_start[index-1])
                / (times_from_start[index+1] - times_from_start[index-1]))
            ).into());
        }
        velocities.push(JointVelocity([0.0;ARM_DEGREES_OF_FREEDOM]));
        vec![]
    }
}

fn relative_times_from_position_list(points: &Vec<JointPosition>) -> Vec<f64> {
    todo!()
}

#[cfg(test)]
mod tests {
    use super::*;
    const FLAT_CUBIC: Cubic = Cubic{coefficients: [0.0,0.0,0.0,0.0]};
    #[test]
    fn test_segment_index() {
        let cubic_trajectory: CubicTrajectory = vec![
            CubicTrajectorySegment{cubic: FLAT_CUBIC, end_time: Duration::from_secs(1)},
            CubicTrajectorySegment{cubic: FLAT_CUBIC, end_time: Duration::from_secs(2)},
            CubicTrajectorySegment{cubic: FLAT_CUBIC, end_time: Duration::from_secs(3)},
            CubicTrajectorySegment{cubic: FLAT_CUBIC, end_time: Duration::from_secs(4)},
        ];
        assert_eq!(cubic_trajectory.segment_index(Duration::from_secs_f64(0.5)), 0);
        assert_eq!(cubic_trajectory.segment_index(Duration::from_secs_f64(1.5)), 1);
        assert_eq!(cubic_trajectory.segment_index(Duration::from_secs_f64(3.5)), 3);
    }
}