mod game;

use fastslam::occupancy_grid::{OccupancyGrid};
use fastslam::load_map::load_map;
use piston_window::*;
use std::path::Path;



fn main() {
    let opengl = OpenGL::V4_5;

    let map_image = load_map();
    let width = map_image.width();
    let height = map_image.height();

    let mut window: PistonWindow = WindowSettings::new("Vacuum Robot Simulator", [width, height])
        .graphics_api(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    let map: G2dTexture = Texture::from_image(
        &mut window.create_texture_context(),
        &map_image,
        &TextureSettings::new()
    ).unwrap();


    window.set_lazy(true);
    while let Some(e) = window.next() {
        window.draw_2d(&e, |c, g, _| {
            clear([1.0; 4], g);
            image(&map, c.transform, g);
        });
    }
}