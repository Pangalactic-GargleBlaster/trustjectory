use core::{f64};
use std::mem::{self};
use std::{fs::File, path::Path, time::Duration};
use std::io::{Read, Write};
use vector_algebra_macro::VectorAlgebra;
use plotters::prelude::*;

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
const JOINT_MAX_VELOCITY: f64 = 0.25;
const MAX_VELOCITIES: [f64; ARM_DEGREES_OF_FREEDOM] = [JOINT_MAX_VELOCITY, JOINT_MAX_VELOCITY, JOINT_MAX_VELOCITY, JOINT_MAX_VELOCITY, 0.25];
const COMBINED_VELOCITY: f64 = JOINT_MAX_VELOCITY;
const JOINT_DISTANCE_WEIGHTS: [f64; ARM_DEGREES_OF_FREEDOM] = [1.0, 1.0, 1.0, 1.0, 2.0/f64::consts::PI];
const JOINT_MAX_ACCELERATION: f64 = 0.25;
const MAX_ACCELERATIONS: [f64; ARM_DEGREES_OF_FREEDOM] = [JOINT_MAX_ACCELERATION, JOINT_MAX_ACCELERATION, JOINT_MAX_ACCELERATION, JOINT_MAX_ACCELERATION, 1.0];
const COMBINED_ACCELERATION: f64 = JOINT_MAX_ACCELERATION;
const BASIC_TRAJECTORY_INTER_POINT_DELAY: Duration = Duration::from_secs(3);
const TRAJECTORY_SAMPLING_PERIOD: Duration = Duration::from_millis(10);

#[derive(Copy, Clone, Debug, serde::Serialize, serde::Deserialize, VectorAlgebra)]
pub struct JointPosition(pub [f64;ARM_DEGREES_OF_FREEDOM]);

#[derive(Copy, Clone, VectorAlgebra)]
pub struct JointVelocity(pub [f64;ARM_DEGREES_OF_FREEDOM]);

impl JointPosition {
    fn weighted_distance_to(&self, other_position: Self) -> f64 {
        let mut distance: f64 = 0.0;
        for index in 0..ARM_DEGREES_OF_FREEDOM {
            distance += (self[index] - other_position[index]).powi(2) * JOINT_DISTANCE_WEIGHTS[index];
        }
        return distance;
    }
}

impl From<JointPosition> for JointVelocity {
    fn from(value: JointPosition) -> Self {
        JointVelocity(value.0)
    }
}

impl From<JointVelocity> for JointPosition {
    fn from(value: JointVelocity) -> Self {
        JointPosition(value.0)
    }
}

pub const HOME_POSITION:JointPosition = JointPosition([0.0, 0.0, 0.0, 0.0, 0.5]);
#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct TrajectoryPoint {
    pub joint_position: JointPosition,
    pub time_from_start: Duration,
}
pub type PointTrajectory = Vec<TrajectoryPoint>;
pub trait PointTrajectoryExt {
    fn inverted(&self) -> Self;
    fn derivative(&self) -> Self;
    fn from_parametric_trajectory(parametric_trajectory: &ParametricTrajectory) -> Self;
    fn save_to_file(&self, file_path: &Path);
    fn load_from_file(file_path: &Path) -> Self;
    fn plot_positions_velocities_and_accelerations(&self, folder_path: &Path, source_points: &Vec<JointPosition>);
    fn plot_joint_trajectories(&self, file_name: &Path, limits: [f64; ARM_DEGREES_OF_FREEDOM], source_points: Vec<TrajectoryPoint>);
}

impl PointTrajectoryExt for PointTrajectory{
    fn inverted(&self) -> Self {
        match self.len() {
            0 => return PointTrajectory::new(),
            length => {
                let total_duration = self[length-1].time_from_start;
                let mut inverted_trajectory = PointTrajectory::new();
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

    fn derivative(&self) -> Self {
        let mut derivative: PointTrajectory = PointTrajectory::new();
        derivative.push(TrajectoryPoint{
            joint_position: (self[1].joint_position - self[0].joint_position)/self[1].time_from_start.as_secs_f64(),
            time_from_start: self[0].time_from_start
        });
        for index in 1..self.len()-1 {
            let i_to_i_plus_1_time = (self[index+1].time_from_start-self[index].time_from_start).as_secs_f64();
            let i_minus_1_to_i_time = (self[index].time_from_start-self[index-1].time_from_start).as_secs_f64();
            let i_to_i_plus_1_derivative = (self[index+1].joint_position-self[index].joint_position)/i_to_i_plus_1_time;
            let i_minus_1_to_i_derivative = (self[index].joint_position-self[index-1].joint_position)/i_minus_1_to_i_time;
            derivative.push(TrajectoryPoint{
                joint_position: (i_to_i_plus_1_derivative*i_to_i_plus_1_time + i_minus_1_to_i_derivative*i_minus_1_to_i_time)/(i_to_i_plus_1_time+i_minus_1_to_i_time),
                time_from_start: self[index].time_from_start
            });
        }
        derivative.push(TrajectoryPoint{
            joint_position: (self.last().unwrap().joint_position - self[self.len()-2].joint_position)/(self.last().unwrap().time_from_start - self[self.len()-2].time_from_start).as_secs_f64(),
            time_from_start: self.last().unwrap().time_from_start
        });
        return derivative;
    }

    fn from_parametric_trajectory(parametric_trajectory: &ParametricTrajectory) -> Self {
        let mut point_trajectory: PointTrajectory = Vec::new();
        let mut time_so_far: Duration = Duration::ZERO;
        let mut segment_local_time: Duration = Duration::ZERO;
        let mut segment_index = 0;
        while segment_index < parametric_trajectory.len() {
            point_trajectory.push(TrajectoryPoint { joint_position: parametric_trajectory[segment_index].interpolate(segment_local_time), time_from_start: time_so_far });
            time_so_far += TRAJECTORY_SAMPLING_PERIOD;
            segment_local_time += TRAJECTORY_SAMPLING_PERIOD;
            while segment_index < parametric_trajectory.len() && segment_local_time > parametric_trajectory[segment_index].duration {
                segment_local_time -= parametric_trajectory[segment_index].duration;
                segment_index += 1;
            }
        }
        return point_trajectory;
    }

    fn save_to_file(&self, file_path: &Path) {
        File::create(&file_path).expect("The file doesn't exist").write_all(serde_json::to_string_pretty(self).expect("Couldn't serialize trajectory").as_bytes()).expect("Couldn't write to file");
    }

    fn load_from_file(file_path: &Path) -> Self {
        let mut file_content: String = String::new();
        File::open(&file_path).expect("The file doesn't exist").read_to_string(&mut file_content).expect("Corrupted file contents");
        return serde_json::from_str(&file_content).expect("Unable to parse trajectory");
    }

    fn plot_positions_velocities_and_accelerations(&self, folder_path: &Path, source_points: &Vec<JointPosition>) {
        let point_times = relative_times_from_position_list(source_points);
        let source_trajectory_points: Vec<TrajectoryPoint> = point_times.iter().enumerate().map(|(index, time)| TrajectoryPoint{time_from_start: Duration::from_secs_f64(*time), joint_position: source_points[index]}).collect();
        let velocities = velocities_at_points(source_points, &point_times);
        let source_velocity_points: Vec<TrajectoryPoint> = point_times.iter().enumerate().map(|(index, time)| TrajectoryPoint{time_from_start: Duration::from_secs_f64(*time), joint_position: velocities[index].into()}).collect();
        self.plot_joint_trajectories(&folder_path.join("positions.png"), MAX_ANGLES, source_trajectory_points);
        let velocity_trajectory = self.derivative();
        velocity_trajectory.plot_joint_trajectories(&folder_path.join("velocities.png"), MAX_VELOCITIES, source_velocity_points);
        let acceleration_trajectory = velocity_trajectory.derivative();
        acceleration_trajectory.plot_joint_trajectories(&folder_path.join("accelerations.png"), MAX_ACCELERATIONS, Vec::new());
    }

    fn plot_joint_trajectories(&self, file_path: &Path, limits: [f64; ARM_DEGREES_OF_FREEDOM], source_points: Vec<TrajectoryPoint>) {
        let root = BitMapBackend::new(file_path, (1024, 1024)).into_drawing_area();
        root.fill(&WHITE).expect("Couldn't create the canvas");

        // Split into vertically stacked subplots
        let areas = root.split_evenly((ARM_DEGREES_OF_FREEDOM, 1));

        // Compute x range (time)
        let t_min = self[0].time_from_start.as_secs_f64();
        let t_max = self.last().unwrap().time_from_start.as_secs_f64();

        for (joint_index, area) in areas.iter().enumerate() {
            let y_min = -limits[joint_index];
            let y_max = limits[joint_index];

            let mut chart = ChartBuilder::on(area)
                .margin(10)
                .x_label_area_size(40)
                .y_label_area_size(60)
                .caption(format!("Joint {}", joint_index), ("sans-serif", 16))
                .build_cartesian_2d(t_min..t_max, 1.5*y_min..1.5*y_max).expect("Couldn't build joint chart");

            chart.configure_mesh()
                .x_labels(5)
                .y_labels(5)
                .x_desc("Time (s)")
                .y_desc("Position (rad)")
                .draw().expect("Couldn't configure the labels");

            // Draw trajectory curve
            chart.draw_series(LineSeries::new(
                self.iter().map(|p| {
                    (p.time_from_start.as_secs_f64(), p.joint_position[joint_index])
                }),
                &BLUE,
            )).expect("Couldn't draw the curve");

            // Draw source points
            chart.draw_series(PointSeries::of_element(
                source_points.iter().map(|trajectory_point| (trajectory_point.time_from_start.as_secs_f64(), trajectory_point.joint_position[joint_index])),
                5,
                &RED,
                &|c, s, st| {
                    // Custom symbol for each point (e.g., a circle)
                    return Circle::new(c, s, st);
                }
            )).expect("Couldn't draw the source points");

            // Draw horizontal red lines for limits
            chart.draw_series(std::iter::once(PathElement::new(
                vec![(t_min, y_min), (t_max, y_min)],
                &RED,
            ))).expect("Couldn't draw lower bound");
            chart.draw_series(std::iter::once(PathElement::new(
                vec![(t_min, y_max), (t_max, y_max)],
                &RED,
            ))).expect("Couldn't draw upper bound");
        }

        root.present().expect("Couldn't render the image");
        println!("Saved to {file_path:?}");
    }

}

pub fn equally_spaced_trajectory(points: &Vec<JointPosition>) -> PointTrajectory {
    let mut current_point_time = Duration::ZERO;
    let mut trajectory: PointTrajectory = vec![];
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
pub fn high_jerk_trajectory(duration: Duration) -> PointTrajectory {
    let mut current_position: JointPosition = HOME_POSITION;
    let mut current_velocity: f64 = 0.0;
    let mut time_so_far = Duration::ZERO;
    let mut trajectory: PointTrajectory = vec![TrajectoryPoint{
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

#[derive(Copy, Clone, Default)]
struct Parabola {
    coefficients: [f64;3]
}


impl Parabola {
    fn interpolate(&self, time_from_start: Duration) -> f64 {
        let time_from_start = time_from_start.as_secs_f64();
        return self.coefficients[0] + self.coefficients[1]*time_from_start + self.coefficients[2]*time_from_start.powi(2);
    }

    fn derivative(&self) -> Self {
        return Parabola { coefficients: [self.coefficients[1], 2.0*self.coefficients[2], 0.0] }
    }
}
#[derive(Default)]
struct TwoParabolasSegment {
    first_parabola: Parabola,
    second_parabola: Parabola,
    inflection_time: Duration
}

impl TwoParabolasSegment {
    fn interpolate(&self, mut time_from_start: Duration) -> f64 {
        let parabola;
        if time_from_start <= self.inflection_time {
            parabola = self.first_parabola;
        } else {
            parabola = self.second_parabola;
            time_from_start -= self.inflection_time;
        }
        return parabola.interpolate(time_from_start);
    }

    fn from_positions_and_velocities(start_position: f64, end_position: f64, start_velocity: f64, end_velocity: f64, duration: Duration) -> Self {
        // solve for the knot time
        let duration = duration.as_secs_f64();
        let knot;
        let acceleration;
        if end_velocity == start_velocity {
            knot = duration/2.0;
            acceleration = 4.0*(end_position - start_position - start_velocity*duration)/duration.powi(2);
        } else {
            let a = start_velocity-end_velocity;
            let b = 2.0*(start_position - end_position + end_velocity*duration);
            let c = (end_position-start_position)*duration - duration.powi(2)/2.0*(start_velocity+end_velocity);
            let plus_or_minus_term = (b.powi(2) - 4.0*a*c).sqrt();
            let knot_minus = (-b - plus_or_minus_term)/(2.0*a);
            let knot_plus = (-b + plus_or_minus_term)/(2.0*a);
            let knot_minus_valid = knot_minus >= 0.0 && knot_minus <= duration;
            let knot_plus_valid = knot_plus >= 0.0 && knot_plus <= duration;
            assert_ne!(knot_minus_valid, knot_plus_valid);
            knot = if knot_minus_valid {knot_minus} else {knot_plus};
            acceleration = (end_velocity - start_velocity)/(2.0*knot - duration);
        }
        let knot_velocity = start_velocity + knot*acceleration;
        let knot_position = start_position + start_velocity*knot + acceleration/2.0*knot*knot;
        let first_parabola = Parabola{coefficients:[start_position, start_velocity, acceleration/2.0]};
        let second_parabola = Parabola{coefficients:[knot_position, knot_velocity, -acceleration/2.0]};
        return Self{
            first_parabola: first_parabola,
            second_parabola: second_parabola,
            inflection_time: Duration::from_secs_f64(knot)
        };
    }

}

pub struct TrajectorySegment {
    joint_trajectories: [TwoParabolasSegment; ARM_DEGREES_OF_FREEDOM],
    duration: Duration
}

impl TrajectorySegment {
    fn interpolate(&self, time_from_start: Duration) -> JointPosition {
        assert!(time_from_start >= Duration::ZERO && time_from_start <= self.duration);
        let mut joint_positions = [0.0;ARM_DEGREES_OF_FREEDOM];
        for index in 0..ARM_DEGREES_OF_FREEDOM {
            joint_positions[index] = self.joint_trajectories[index].interpolate(time_from_start);
        }
        return JointPosition(joint_positions);
    }

    fn from_positions_and_velocities(
        start_position: JointPosition,
        end_position: JointPosition,
        start_velocity: JointVelocity,
        end_velocity: JointVelocity,
        duration: Duration,
    ) -> Self {
        let joint_trajectories =
            std::array::from_fn(|joint_index: usize| {
                TwoParabolasSegment::from_positions_and_velocities(
                    start_position[joint_index],
                    end_position[joint_index],
                    start_velocity[joint_index],
                    end_velocity[joint_index],
                    duration,
                )
            });

        TrajectorySegment { joint_trajectories, duration }
    }
}

pub type ParametricTrajectory = Vec<TrajectorySegment>;

pub trait ParametricTrajectoryExt {
    fn from_position_list(points: &Vec<JointPosition>) -> Self;
}

impl ParametricTrajectoryExt for ParametricTrajectory {
    fn from_position_list(points: &Vec<JointPosition>) -> Self {
        let times_from_start = relative_times_from_position_list(points);
        let velocities = velocities_at_points(points, &times_from_start);
        let mut parametric_trajectory: ParametricTrajectory = Vec::with_capacity(points.len()-1);
        for index in 0..points.len()-1 {
            parametric_trajectory.push(TrajectorySegment::from_positions_and_velocities(points[index], points[index+1], velocities[index], velocities[index+1], Duration::from_secs_f64(times_from_start[index+1] - times_from_start[index])));
        }
        return parametric_trajectory;
    }
}

const MIN_SPEED_LIMITED_DISTANCE: f64 = COMBINED_VELOCITY * COMBINED_VELOCITY / COMBINED_ACCELERATION;

fn relative_times_from_position_list(points: &Vec<JointPosition>) -> Vec<f64> {
    let mut cumulative_distances: Vec<f64> = Vec::with_capacity(points.len());
    let mut total_so_far = 0.0;
    cumulative_distances.push(total_so_far);
    for index in 1..points.len() {
        total_so_far += points[index].weighted_distance_to(points[index-1]);
        cumulative_distances.push(total_so_far);
    }
    let total_distance = total_so_far;
    println!("The cumulative distances of the points are {cumulative_distances:?}");
    let mut times: Vec<f64> = Vec::with_capacity(points.len());
    if total_distance <= MIN_SPEED_LIMITED_DISTANCE {
        println!("Creating acceleration limited trajectory");
        let inflection_time = (total_distance/COMBINED_ACCELERATION).sqrt();
        let total_time = 2.0 * inflection_time;
        let mut index = 0;
        while cumulative_distances[index] <= total_distance/2.0 { // acceleration phase
            times.push((cumulative_distances[index]*2.0/COMBINED_ACCELERATION).sqrt());
            index += 1;
        }
        while index < points.len() { // deceleration phase
            let time_from_end = ((total_distance-cumulative_distances[index])*2.0/COMBINED_ACCELERATION).sqrt();
            times.push(total_time - time_from_end);
            index += 1;
        }
    } else {
        println!("Creating speed limited trajectory");
        let inflection_time = (MIN_SPEED_LIMITED_DISTANCE/COMBINED_ACCELERATION).sqrt();
        let linear_time = (total_distance - MIN_SPEED_LIMITED_DISTANCE) / COMBINED_VELOCITY;
        let total_time = 2.0*inflection_time + linear_time;
        let mut index = 0;
        while cumulative_distances[index] <= MIN_SPEED_LIMITED_DISTANCE/2.0 { // acceleration phase
            times.push((cumulative_distances[index]*2.0/COMBINED_ACCELERATION).sqrt());
            index += 1;
        }
        while cumulative_distances[index] <= total_distance - MIN_SPEED_LIMITED_DISTANCE/2.0 { // constant velocity phase
            let time_since_inflection = (cumulative_distances[index] - MIN_SPEED_LIMITED_DISTANCE/2.0)/COMBINED_VELOCITY;
            times.push(inflection_time + time_since_inflection);
            index += 1;
        }
        while index < points.len() { // deceleration phase
            let time_from_end = ((total_distance-cumulative_distances[index])*2.0/COMBINED_ACCELERATION).sqrt();
            times.push(total_time - time_from_end);
            index += 1;
        }
    }
    return times;
}

fn velocities_at_points(points: &Vec<JointPosition>, times_from_start: &Vec<f64>) -> Vec<JointVelocity> {
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
    return velocities;
}


#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_two_parabolas() {
        // 0,0 1,0 1
        let two_parabolas_segment = TwoParabolasSegment::from_positions_and_velocities(0.0, 1.0, 0.0, 0.0, Duration::from_secs_f64(1.0));
        assert_eq!(two_parabolas_segment.inflection_time, Duration::from_secs_f64(0.5));
        assert_eq!(two_parabolas_segment.first_parabola.coefficients, [0.0, 0.0, 2.0]);
        assert_eq!(two_parabolas_segment.second_parabola.coefficients, [0.5, 2.0, -2.0]);

        // 0,0 1,1 1
        let one_second = Duration::from_secs(1);
        let two_parabolas_segment: TwoParabolasSegment = TwoParabolasSegment::from_positions_and_velocities(0.0, 1.0, 0.0, 1.0, one_second);
        assert_abs_diff_eq!(two_parabolas_segment.interpolate(Duration::ZERO), 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(two_parabolas_segment.first_parabola.derivative().interpolate(Duration::ZERO), 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(two_parabolas_segment.interpolate(one_second), 1.0, epsilon = 1e-5);
        assert_abs_diff_eq!(two_parabolas_segment.second_parabola.derivative().interpolate(one_second-two_parabolas_segment.inflection_time), 1.0, epsilon = 1e-5);
        assert_abs_diff_eq!(two_parabolas_segment.first_parabola.interpolate(two_parabolas_segment.inflection_time), two_parabolas_segment.second_parabola.interpolate(Duration::ZERO), epsilon = 1e-5);
        assert_abs_diff_eq!(two_parabolas_segment.first_parabola.derivative().interpolate(two_parabolas_segment.inflection_time), two_parabolas_segment.second_parabola.derivative().interpolate(Duration::ZERO), epsilon = 1e-5);
    }

    #[test]
    fn test_times_from_distances() {
        let joint_position_zero = JointPosition([0.0;ARM_DEGREES_OF_FREEDOM]);
        let joint_position_half = JointPosition([0.5,0.0,0.0,0.0,0.0]);
        let joint_position_one = JointPosition([1.0,0.0,0.0,0.0,0.0]);
        let times = relative_times_from_position_list(&vec![joint_position_zero, joint_position_half, joint_position_one]);
        assert_eq!(times[0], 0.0);
        assert_eq!(times[1], times[2]/2.0);

        let positions: Vec<JointPosition> = (0..10).map(|index| JointPosition([index as f64, 0.0, 0.0, 0.0, 0.0])).collect();
        let times = relative_times_from_position_list(&positions);
        assert_eq!(times[0], 0.0);
        for index in 0..9 {
            assert!(times[index] < times[index+1], "times are out of order at indices {index} and {}", index+1);
        }
        assert_abs_diff_eq!(times[6]-times[5], 1.0/COMBINED_VELOCITY, epsilon=1e-5);
    }
}