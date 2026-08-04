pub struct CsvProcessing {
    pub n_col: usize,
    pub header_names: Vec<String>,
    pub data: Vec<Vec<f32>>,
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
}

impl CsvProcessing {
    pub fn extract_information(
        file_path: &str,
        timestamp_index: usize,
        dt_s: f32,
        y_axis_offset: f32,
    ) -> Result<Self, Box<dyn std::error::Error>> {
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

                if idx == 0 {
                    val *= dt_s;
                }

                row_values.push(val);
            }
            data.push(row_values);
        }

        let min_x = data
            .iter()
            .map(|row| row[timestamp_index])
            .fold(f32::INFINITY, f32::min);
        let max_x = data
            .iter()
            .map(|row| row[timestamp_index])
            .fold(f32::NEG_INFINITY, f32::max);

        let mut min_y = f32::INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        for row in &data {
            for (col_idx, &current_row) in row.iter().enumerate().take(n_col) {
                if col_idx != timestamp_index {
                    min_y = min_y.min(current_row);
                    max_y = max_y.max(current_row);
                }
            }
        }

        let y_range = if max_y == min_y { 1.0 } else { max_y - min_y };
        let y_offset = y_range * y_axis_offset;
        let padded_min_y = min_y - y_offset;
        let padded_max_y = max_y + y_offset;

        let result = Self {
            n_col,
            header_names,
            data,
            min_x,
            max_x,
            min_y: padded_min_y,
            max_y: padded_max_y,
        };

        Ok(result)
    }
}
