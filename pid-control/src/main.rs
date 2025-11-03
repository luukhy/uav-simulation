use plotters::prelude::*;



struct PIDController {
    p_val: f32,
    d_val: f32,
    i_val: f32,
    prev_error: f32,
    integral: f32,
}

impl PIDController {
    fn compute_pid(&mut self, setpoint: f32, measured_val: f32) -> f32 {
        let curr_err: f32 = setpoint - measured_val;
        self.integral += curr_err;
        let output: f32 = self.p_val * curr_err + self.i_val * self.integral + self.d_val * (curr_err - self.prev_error);
        println!("{output}");
        self.prev_error = curr_err;
        output
    }
}



fn main() {
    let mut pid_controller = PIDController {
        p_val: 0.3,
        i_val: 0.1,
        d_val: 0.1,
        prev_error: 0.0,
        integral: 0.0,
    };

    let setpoint: f32 = 10.0;
    let mut measured: f32 = 0.0;
    let mut output_vec: Vec<f32> = Vec::new();
    for _i in 1..100{
        measured = pid_controller.compute_pid(setpoint, measured);         
        output_vec.push(measured);
        println!("{measured}")
    }
    for val in output_vec.iter() {
        println!("{val}");
    }
    plot_results(output_vec).expect("REASON");
}

fn plot_results(output_vec: Vec<f32>) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("output_plot.png", (640, 480)).into_drawing_area();
    root.fill(&WHITE);
    let root = root.margin(10, 10, 10, 10);
    let last_value = match output_vec.last() {
        Some(last_element) => *last_element,
        None => {println!("Output vector is empty!"); 
                return Ok(())}, 
    };
    let output_length = output_vec.len();
    let mut points: Vec<(f32, f32)> = Vec::new();
    for x in 1..output_length {
        points.push((x as f32, output_vec[x]));
    }

    let output_length_f32 = output_length as f32;

    let mut chart = ChartBuilder::on(&root)
        .caption("PID output", ("sans-serif", 40).into_font())
        .x_label_area_size(20)
        .y_label_area_size(40)
        .build_cartesian_2d(0f32..output_length_f32, 0f32..14f32)?;
    
    chart
        .configure_mesh()
        // We can customize the maximum number of labels allowed for each axis
        .x_labels(5)
        .y_labels(5)
        // We can also change the format of the label text
        .y_label_formatter(&|x| format!("{:.3}", x))
        .draw()?;

    chart.draw_series(LineSeries::new(
        points,
        &RED,
    ))?;
    Ok(())
}