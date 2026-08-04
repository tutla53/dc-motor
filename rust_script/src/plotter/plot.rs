use super::*;

pub fn plot_csv(
    dir_path: &str,
    file_path: &str,
    chart_title: &str,
    y_label: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    /* Plotter Config */
    let font_style = "sans-serif";
    let canvas = Canvas::new(2048, 16, 9);
    let font_size = FontSize::new(80, 60, 45, 45);
    let margin = Margin::new(30, 30, 50, 100, 150, 200);
    let chart_config = ChartConfig::new(7, 1, chart_title, "Time (s)", y_label);
    let color_config = ColorConfig::new(WHITE.mix(0.92), WHITE.mix(0.5), BLACK, SERIES_COLORS);

    /* Processing CSV */
    let csv_log = CsvProcessing::extract_information(
        file_path,
        TIMESTAMP_INDEX,
        motor_config::DT_S as f32,
        Y_AXIS_OFFSET,
    )?;

    /*  Plotting Process */
    let output_file = format!("{}/plot_result.png", dir_path);

    let root = BitMapBackend::new(&output_file, canvas.get_size()).into_drawing_area();
    root.fill(&color_config.bg_color)?;

    let mut chart = ChartBuilder::on(&root)
        .caption(
            chart_config.title,
            (font_style, font_size.title).into_font(),
        )
        .margin_top(margin.top)
        .margin_bottom(margin.bottom)
        .margin_left(margin.left)
        .margin_right(margin.right)
        .x_label_area_size(margin.x_pad)
        .y_label_area_size(margin.y_pad)
        .build_cartesian_2d(csv_log.min_x..csv_log.max_x, csv_log.min_y..csv_log.max_y)?;

    chart
        .configure_mesh()
        .x_desc(chart_config.x_label)
        .y_desc(chart_config.y_label)
        .axis_desc_style((font_style, font_size.axis_desc).into_font())
        .label_style((font_style, font_size.axis_label).into_font())
        .draw()?;

    for col_idx in (0..csv_log.n_col).rev() {
        if col_idx == TIMESTAMP_INDEX {
            continue;
        }

        let series_data: Vec<(f32, f32)> = csv_log
            .data
            .iter()
            .map(|row| (row[TIMESTAMP_INDEX], row[col_idx]))
            .collect();

        let color = color_config.pallete[(col_idx - 1) % color_config.pallete.len()];
        let line_style = color.filled().stroke_width(chart_config.line_stroke_width);
        let legend_line_style = color
            .filled()
            .stroke_width(chart_config.line_stroke_width * 5);

        let label_name = csv_log
            .header_names
            .get(col_idx)
            .filter(|name| !name.trim().is_empty())
            .cloned()
            .unwrap_or_else(|| format!("Column {}", col_idx));

        chart
            .draw_series(LineSeries::new(series_data.iter().copied(), line_style))?
            .label(label_name)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], legend_line_style));

        chart.draw_series(
            series_data
                .iter()
                .map(|&(x, y)| Circle::new((x, y), chart_config.circle_size, color.filled())),
        )?;
    }

    chart
        .configure_series_labels()
        .background_style(color_config.label_bg)
        .border_style(color_config.label_border)
        .label_font((font_style, font_size.legend).into_font())
        .draw()?;

    root.present()?;
    println!("  [INFO] - Successfully saved chart map to {}", output_file);

    Ok(())
}
