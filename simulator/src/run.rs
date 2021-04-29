mod game;
use opengl_graphics::{GlGraphics, OpenGL};
use piston_window::{PistonWindow, WindowSettings, Events, EventSettings, RenderEvent, MouseScrollEvent, UpdateEvent, Button, PressEvent};
use fastslam::geometry::Point;
use fastslam::render::RenderConfig;
use crate::game::Game;
use fastslam::simulator::{Robot};
use fastslam::sensor::laserscanner::Scan;
use fastslam::particlefilter::ParticleFilter;
use std::{env, fs, thread};
use std::io::Read;
use svg2polylines::Polyline;
use fastslam::geometry;

fn main() {
    let opengl = OpenGL::V4_5;

    let mut window: PistonWindow = WindowSettings::new("FastSLAM Robot Simulator", [800, 400])
        .graphics_api(opengl)
        .fullscreen(true)
        .exit_on_esc(true)
        .build()
        .unwrap();



    // TODO: Try setting the fps
    // window.set_up(60)
    // window.set_max_fps(60)

    // little helper to construct vectors
    let point = |x: f64, y: f64| Point::new(x,y);

    let mut game = Game::new(
        GlGraphics::new(opengl),
        RenderConfig { scale: 20.0 },
        Robot::default(),
        Scan::empty(),
        ParticleFilter::default(),
        vec![]
    );

    // read static world from SVG file
    let args: Vec<_> = env::args().collect();
    match args.len() {
        2 => {}
        _ => {
            println!("Usage: {} <map.svg>", args[0]);
            std::process::exit(1);
        }
    };

    let mut file = fs::File::open(&args[1]).unwrap();
    let mut s = String::new();
    file.read_to_string(&mut s).unwrap();

    // parse data
    let polylines: Vec<Polyline> = svg2polylines::parse(&s).unwrap_or_else(|e| {
        println!("Error: {}", e);
        std::process::exit(1);
    });

    let m_per_px = 0.02;
    for polyline in &polylines {
        for pair in polyline.windows(2) {
            game.objects.push(geometry::Line::new(
                point(-pair[0].y * m_per_px, -pair[0].x * m_per_px),
                point(-pair[1].y * m_per_px, -pair[1].x * m_per_px)
            ))
        }
    }

    let mut events = Events::new(EventSettings::new());
    while let Some(e) = events.next(&mut window) {

        if let Some(Button::Keyboard(key)) = e.press_args() {
            game.key_pressed(key);
        }

        if let Some(a) = e.render_args() {
            game.render(&a);
        }

        if let Some(a) = e.update_args() {
            game.update(&a);
        }

        if let Some(a) = e.mouse_scroll_args() {
            game.render_config.scale *= 1.0 + 0.2 * a[1];
            game.render_config.scale = f64::max(1.0, game.render_config.scale);
        }
    }


    let mapgl = OpenGL::V4_5;

    thread::spawn(move || {
        let mut map_window: PistonWindow = WindowSettings::new("Particle Filter Map", [400, 200])
            .graphics_api(mapgl)
            .fullscreen(false)
            .exit_on_esc(true)
            .build()
            .unwrap();
    });

}