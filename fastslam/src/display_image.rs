
fn show_img() {
    use imageproc::window::display_image;
    use std::env;

    let image = image::open("maps/map_large.png")
        .expect("No image found at provided path")
        .to_rgb8();

    display_image("gt map", &image, 500, 500);
}

use image::GenericImageView;

fn main() {
    // Use the open function to load an image from a Path.
    // `open` returns a `DynamicImage` on success.
    // let img = image::open("maps/map_large.png").unwrap();
    //
    // // The dimensions method returns the images width and height.
    // println!("dimensions {:?}", img.dimensions());
    //
    // // The color method returns the image's `ColorType`.
    // println!("{:?}", img.color());
    //
    // // Write the contents of this image to the Writer in PNG format.
    // img.save("test.png").unwrap();
    show_img();
}