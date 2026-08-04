/* Plotter Config */

use super::*;

pub const TIMESTAMP_INDEX: usize = 0;
pub const Y_AXIS_OFFSET: f32 = 0.1;
pub const SERIES_COLORS: &[RGBColor] = &[RED, BLUE, GREEN, CYAN, MAGENTA, BLACK];

pub struct Canvas {
    width: u32,
    height: u32,
}

impl Canvas {
    pub fn new(width: u32, width_ratio: u32, height_ratio: u32) -> Self {
        let height = (width * height_ratio) / width_ratio;
        Self { width, height }
    }
    pub fn get_size(&self) -> (u32, u32) {
        (self.width, self.height)
    }
}

pub struct FontSize {
    pub title: i32,
    pub axis_desc: i32,
    pub axis_label: i32,
    pub legend: i32,
}

impl FontSize {
    pub fn new(title: i32, axis_desc: i32, axis_label: i32, legend: i32) -> Self {
        Self {
            title,
            axis_desc,
            axis_label,
            legend,
        }
    }
}

pub struct Margin {
    pub top: i32,
    pub bottom: i32,
    pub left: i32,
    pub right: i32,
    pub x_pad: i32,
    pub y_pad: i32,
}

impl Margin {
    pub fn new(top: i32, bottom: i32, left: i32, right: i32, x_pad: i32, y_pad: i32) -> Self {
        Self {
            top,
            bottom,
            left,
            right,
            x_pad,
            y_pad,
        }
    }
}

pub struct ChartConfig<'a> {
    pub circle_size: i32,
    pub line_stroke_width: u32,
    pub title: &'a str,
    pub x_label: &'a str,
    pub y_label: &'a str,
}

impl<'a> ChartConfig<'a> {
    pub fn new(
        circle_size: i32,
        line_stroke_width: u32,
        title: &'a str,
        x_label: &'a str,
        y_label: &'a str,
    ) -> Self {
        Self {
            circle_size,
            line_stroke_width,
            title,
            x_label,
            y_label,
        }
    }
}

pub struct ColorConfig<'a> {
    pub bg_color: RGBAColor,
    pub label_bg: RGBAColor,
    pub label_border: RGBColor,
    pub pallete: &'a [RGBColor],
}

impl<'a> ColorConfig<'a> {
    pub fn new(
        bg_color: RGBAColor,
        label_bg: RGBAColor,
        label_border: RGBColor,
        pallete: &'a [RGBColor],
    ) -> Self {
        Self {
            bg_color,
            label_bg,
            label_border,
            pallete,
        }
    }
}
