use image::{GenericImageView, Rgb, GenericImage, ImageBuffer};
use std::thread;
use imageproc::drawing::draw_cross_mut;
use std::time::Duration;
use std::sync::{Mutex, Arc, MutexGuard};
use piston_window::{PistonWindow, WindowSettings};


fn load_map() {
    use imageproc::window::display_image;
    use std::env;

    let mut image = image::open("maps/map_large.png")
        .expect("No image found at provided path")
        .to_rgb8();
    display_image("gt map", &image, 500, 500);
}


fn main() {
    load_map();
}