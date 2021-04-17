use image::{GenericImageView, Rgb, GenericImage, ImageBuffer, RgbImage, RgbaImage};
use std::thread;
use imageproc::drawing::draw_cross_mut;
use std::time::Duration;
use std::sync::{Mutex, Arc, MutexGuard};
use piston_window::{PistonWindow, WindowSettings};


pub fn load_map() -> RgbaImage {
    use imageproc::window::display_image;
    use std::env;

    let mut image = image::open("maps/map_large.png")
        .expect("No image found at provided path")
        .to_rgba8();

    return image
}