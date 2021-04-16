use fastslam::occupancy_grid::{OccupancyGrid};
use piston_window::*;
use std::path::Path;



fn main() {
    let opengl = OpenGL::V4_5;

    let mut window: PistonWindow = WindowSettings::new("Vacuum Robot Simulator", [800, 400])
        .graphics_api(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    let map: G2dTexture = Texture::from_path(
        &mut window.create_texture_context(),
        Path::new("maps/map_large.png"),
        Flip::None,
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