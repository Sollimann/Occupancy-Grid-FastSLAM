use image::io::Reader as ImageReader;
use image::{RgbImage, ImageBuffer};

#[derive(Debug)]
pub struct OccupancyGrid {
    pub map: RgbImage
}

impl OccupancyGrid {

    pub fn new(width: u32, height: u32) -> Self {
        let map: RgbImage = ImageBuffer::new(width, height);

        let mut occ_grid = Self {
            map
        };

        return occ_grid
    }
}