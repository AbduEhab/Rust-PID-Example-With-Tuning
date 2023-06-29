use ndarray::Array1;
use plotters::prelude::*;

fn main() {
    // Define the parameters for the spring-mass-damper system
    let m: f64 = 1.0; // mass
    let c: f64 = 0.2; // damping coefficient
    let k: f64 = 1.0; // spring constant

    // Set initial conditions
    let x0: f64 = 0.0; // initial position
    let v0: f64 = 1.0; // initial velocity

    // Set the desired position
    let x_desired: f64 = 1.0;

    // PID controller gains
    let kp: f64 = 1.0; // Proportional gain
    let ki: f64 = 0.5; // Integral gain
    let kd: f64 = 0.9; // Derivative gain

    // Define the range of values for t (time)
    let t_start: f64 = 0.0;
    let t_end: f64 = 100.0;
    let h: f64 = 0.01; // Step size

    // Calculate the number of steps
    let num_steps = ((t_end - t_start) / h).ceil() as usize;

    // Create arrays to store t, x, and v values
    let mut t_values = Array1::zeros(num_steps + 1);
    let mut x_values = Array1::zeros(num_steps + 1);
    let mut v_values = Array1::zeros(num_steps + 1);

    // Initialize the arrays with initial values
    t_values[0] = t_start;
    x_values[0] = x0;
    v_values[0] = v0;

    // Initialize the integral and derivative terms of the PID controller
    let mut integral_term = 0.0;
    let mut prev_error = 0.0;

    // Perform numerical integration using Euler's method
    for i in 0..num_steps {
        let t = t_values[i];
        let x = x_values[i];
        let v = v_values[i];

        // Calculate the error between the desired position and the current position
        let error = x_desired - x;

        // Calculate the control input using PID controller
        let control_input = kp * error + ki * integral_term + kd * (error - prev_error) / h;

        // Update the integral term
        integral_term += error * h;

        let dx_dt = v;
        let dv_dt = (-c / m) * v - (k / m) * x + control_input;

        let dt = h;

        let x_next = x + dx_dt * dt;
        let v_next = v + dv_dt * dt;
        let t_next = t + dt;

        t_values[i + 1] = t_next;
        x_values[i + 1] = x_next;
        v_values[i + 1] = v_next;

        prev_error = error;
    }

    // Plot the solution trajectory
    let root = BitMapBackend::new("spring_mass_damper_pid.png", (800, 600)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let t_min = *t_values
        .iter()
        .min_by(|t1, t2| t1.partial_cmp(t2).unwrap())
        .unwrap();
    let t_max = *t_values
        .iter()
        .max_by(|t1, t2| t1.partial_cmp(t2).unwrap())
        .unwrap();
    let x_min = *x_values
        .iter()
        .min_by(|x1, x2| x1.partial_cmp(x2).unwrap())
        .unwrap();
    let x_max = *x_values
        .iter()
        .max_by(|x1, x2| x1.partial_cmp(x2).unwrap())
        .unwrap();

    let mut chart = ChartBuilder::on(&root)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .margin(5)
        .caption(
            "Spring-Mass-Damper System with PID Controller",
            ("Arial", 20),
        )
        .build_cartesian_2d(t_min..t_max, x_min..x_max)
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    chart
        .draw_series(LineSeries::new(
            t_values.iter().zip(x_values.iter()).map(|(&t, &x)| (t, x)),
            &BLUE,
        ))
        .unwrap();

    // Save the plot as an image
    root.present().expect("Failed to save the image");
}
