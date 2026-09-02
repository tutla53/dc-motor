#![allow(unused)]

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

    for col_idx in 0..csv_log.n_col {
        if col_idx == TIMESTAMP_INDEX {
            continue;
        }

        let series_data: Vec<(f32, f32)> = csv_log
            .data
            .iter()
            .map(|row| (row[TIMESTAMP_INDEX], row[col_idx]))
            .collect();

        let color = color_config.pallete[(col_idx - 1) % color_config.pallete.len()];

        let label_name = csv_log
            .header_names
            .get(col_idx)
            .filter(|name| !name.trim().is_empty())
            .cloned()
            .unwrap_or_else(|| format!("Column {}", col_idx));

        if csv_log.header_names[col_idx].contains("Commanded") {
            let line_style = BLACK
                .filled()
                .stroke_width(chart_config.line_stroke_width * 6);
            let legend_line_style = BLACK
                .filled()
                .stroke_width(chart_config.line_stroke_width * 5);

            chart
                .draw_series(DashedLineSeries::new(
                    series_data.iter().copied(),
                    30,
                    30,
                    line_style,
                ))?
                .label(&label_name)
                .legend(move |(x, y)| {
                    DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, legend_line_style)
                });
        } else {
            let line_style = color.filled().stroke_width(chart_config.line_stroke_width);
            let legend_line_style = color
                .filled()
                .stroke_width(chart_config.line_stroke_width * 5);

            chart
                .draw_series(LineSeries::new(series_data.iter().copied(), line_style))?
                .label(&label_name)
                .legend(move |(x, y)| {
                    PathElement::new(vec![(x, y), (x + 20, y)], legend_line_style)
                });

            chart.draw_series(
                series_data
                    .iter()
                    .map(|&(x, y)| Circle::new((x, y), chart_config.circle_size, color.filled())),
            )?;
        }
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

pub fn plot_log(
    dir_path: &str,
    csv_log: &CsvProcessing,
    chart_title: &str,
    y_label: &str,
    overlays: &[OverlaySeries],
) -> Result<(), Box<dyn std::error::Error>> {
    if csv_log.data.is_empty() {
        return Err("Cannot plot an empty log".into());
    }

    /* ---------- Plot Configuration ---------- */
    let font_style = "sans-serif";
    let canvas = Canvas::new(2048, 16, 9);
    let font_size = FontSize::new(80, 60, 45, 45);
    let margin = Margin::new(30, 30, 50, 100, 150, 200);
    let chart_config = ChartConfig::new(7, 1, chart_title, "Time (s)", y_label);
    let color_config = ColorConfig::new(WHITE.mix(0.92), WHITE.mix(0.5), BLACK, SERIES_COLORS);

    /* ---------- Determine Bounds ---------- */
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;

    for row in &csv_log.data {
        min_x = min_x.min(row[TIMESTAMP_INDEX]);
        max_x = max_x.max(row[TIMESTAMP_INDEX]);

        for (column, &value) in row.iter().enumerate() {
            if column != TIMESTAMP_INDEX {
                min_y = min_y.min(value);
                max_y = max_y.max(value);
            }
        }
    }

    for overlay in overlays {
        for &(time_s, value) in &overlay.points {
            min_x = min_x.min(time_s);
            max_x = max_x.max(time_s);
            min_y = min_y.min(value);
            max_y = max_y.max(value);
        }
    }

    let y_range = (max_y - min_y).max(1.0);
    min_y -= y_range * Y_AXIS_OFFSET;
    max_y += y_range * Y_AXIS_OFFSET;

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
        .build_cartesian_2d(min_x..max_x, min_y..max_y)?;

    chart
        .configure_mesh()
        .x_desc(chart_config.x_label)
        .y_desc(chart_config.y_label)
        .axis_desc_style((font_style, font_size.axis_desc).into_font())
        .label_style((font_style, font_size.axis_label).into_font())
        .draw()?;

    /* ---------- Firmware CSV Series ---------- */
    for col_idx in 0..csv_log.n_col {
        if col_idx == TIMESTAMP_INDEX {
            continue;
        }

        let series_data: Vec<(f32, f32)> = csv_log
            .data
            .iter()
            .map(|row| (row[TIMESTAMP_INDEX], row[col_idx]))
            .collect();

        let color = color_config.pallete[(col_idx - 1) % color_config.pallete.len()];

        let label_name = csv_log
            .header_names
            .get(col_idx)
            .filter(|name| !name.trim().is_empty())
            .cloned()
            .unwrap_or_else(|| format!("Column {}", col_idx));

        if csv_log.header_names[col_idx].contains("Commanded") {
            let line_style = BLACK
                .filled()
                .stroke_width(chart_config.line_stroke_width * 6);
            let legend_style = BLACK
                .filled()
                .stroke_width(chart_config.line_stroke_width * 5);

            chart
                .draw_series(DashedLineSeries::new(
                    series_data.iter().copied(),
                    30,
                    30,
                    line_style,
                ))?
                .label(label_name)
                .legend(move |(x, y)| {
                    DashedPathElement::new(vec![(x, y), (x + 20, y)], 5, 5, legend_style)
                });
        } else {
            let line_style = color.filled().stroke_width(chart_config.line_stroke_width);
            let legend_style = color
                .filled()
                .stroke_width(chart_config.line_stroke_width * 5);

            chart
                .draw_series(LineSeries::new(series_data.iter().copied(), line_style))?
                .label(label_name)
                .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], legend_style));

            chart.draw_series(
                series_data
                    .iter()
                    .map(|&(x, y)| Circle::new((x, y), chart_config.circle_size, color.filled())),
            )?;
        }
    }

    /* ---------- Simulation Series ---------- */
    for overlay in overlays {
        if overlay.points.is_empty() {
            continue;
        }

        let color = overlay.color;
        let line_style = color.filled().stroke_width(chart_config.line_stroke_width);
        let legend_style = color
            .filled()
            .stroke_width(chart_config.line_stroke_width * 5);

        chart
            .draw_series(LineSeries::new(overlay.points.iter().copied(), line_style))?
            .label(overlay.label.clone())
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], legend_style));

        chart.draw_series(
            overlay
                .points
                .iter()
                .map(|&(x, y)| Circle::new((x, y), chart_config.circle_size / 2, color.filled())),
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
