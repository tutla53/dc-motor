
use super::*;

pub fn plot_csv(dir_path: &str, file_path: &str, chart_title: &str, y_label:&str) -> Result<(), Box<dyn std::error::Error>> {

    // Plotter Config
    let canvas_width = 2048;
    let canvas_height = (canvas_width * 9)/16;

    let title_font_size = 80;
    let axis_desc_font_size = 60;
    let axis_label_font_size = 45;
    let legend_font_size = 45;
    
    let line_stroke_width = 1;
    let circle_size = 7;
    let x_area_padding = 150;
    let y_area_padding = 200;   

    let margin_top = 30;    
    let margin_bottom = 30;
    let margin_left = 50;
    let margin_right = 100;  

    let y_offset_factor = 0.1; 

    // Process CSV
    let mut reader = csv::Reader::from_path(file_path)?;
    
    let headers = reader.headers()?;
    let n_col = headers.len();
    let header_names: Vec<String> = headers.iter().map(|h| h.to_string()).collect();

    let mut data: Vec<Vec<f32>> = Vec::new();
    
    for result in reader.records() {
        let record = result?;
        let mut row_values = Vec::with_capacity(n_col);
        
        for (idx, field) in record.iter().enumerate() {
            let mut val: f32 = field.parse().unwrap_or(0.0f32);
            
            if idx == 0 { val /= 1000.0; }
            
            row_values.push(val);
        }
        data.push(row_values);
    }
    
    let x_col = 0;

    let min_x = data.iter().map(|row| row[x_col]).fold(f32::INFINITY, f32::min);
    let max_x = data.iter().map(|row| row[x_col]).fold(f32::NEG_INFINITY, f32::max);

    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;
    for row in &data {
        for col_idx in 0..n_col {
            if col_idx != x_col {
                min_y = min_y.min(row[col_idx]);
                max_y = max_y.max(row[col_idx]);
            }
        }
    }

    let y_range = if max_y == min_y { 1.0 } else { max_y - min_y };
    let y_offset = y_range * y_offset_factor; 
    let padded_min_y = min_y - y_offset;
    let padded_max_y = max_y + y_offset;

    let output_file = format!("{}/plot.png", dir_path); 
    let root = BitMapBackend::new(&output_file, (canvas_width, canvas_height)).into_drawing_area();
    let bg_color = RGBColor(245, 247, 250);
    root.fill(&bg_color)?;

    let mut chart = ChartBuilder::on(&root)
        .caption(chart_title, ("sans-serif", title_font_size).into_font())
        .margin_top(margin_top)
        .margin_bottom(margin_bottom)
        .margin_left(margin_left)
        .margin_right(margin_right)
        .x_label_area_size(x_area_padding)
        .y_label_area_size(y_area_padding)
        .build_cartesian_2d(min_x..max_x, padded_min_y..padded_max_y)?;

    chart.configure_mesh()
        .x_desc("Time (ms)")
        .y_desc(y_label)
        .axis_desc_style(("sans-serif", axis_desc_font_size).into_font())
        .label_style(("sans-serif", axis_label_font_size).into_font())
        .draw()?;

    let colors = [&RED, &BLUE, &GREEN, &CYAN, &MAGENTA, &BLACK];

    for col_idx in 0..n_col {
        if col_idx == x_col {
            continue;
        }

        let series_data: Vec<(f32, f32)> = data.iter()
            .map(|row| (row[x_col], row[col_idx]))
            .collect();

        let color = colors[(col_idx - 1) % colors.len()];
        let line_style = color.filled().stroke_width(line_stroke_width);

        let label_name = header_names.get(col_idx)
            .filter(|name| !name.trim().is_empty())
            .cloned()
            .unwrap_or_else(|| format!("Column {}", col_idx));

        chart.draw_series(LineSeries::new(series_data.iter().copied(), line_style))?
            .label(label_name)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], line_style));

        chart.draw_series(
            series_data.iter().map(|&(x, y)| Circle::new((x, y), circle_size, color.filled()))
        )?;
    }

    chart.configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .label_font(("sans-serif", legend_font_size).into_font())
        .draw()?;

    root.present()?;
    println!("  [INFO] - Successfully saved chart map to {}", output_file);

    Ok(())
}