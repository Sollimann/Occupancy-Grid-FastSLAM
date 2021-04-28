use crate::geometry::vector::Vector;
use crate::geometry::point::Point;
use crate::geometry::target::Target;
use crate::geometry::ray::Ray;

pub struct Line {
    pub start: Point,
    pub end: Point,
}

impl Line {
    pub fn new(start: Point, end: Point) -> Line {
        Line { start, end }
    }
}

impl Target for Line {

    /// see https://stackoverflow.com/a/565282/704831
    fn intersect(&self, ray: &Ray) -> Vec<Point> {

        // ray light
        let p = ray.origin;
        let r_vec = ray.direction;
        let p_r = p + r_vec;
        let p_to_r = p.to_point_vec(p_r);

        // obstacle line
        let q = self.start;
        let s_vec = q.to_point_vec(self.end);
        let q_s = q + s_vec;
        let q_to_s = q.to_point_vec(q_s);

        let det = p_to_r.cross(q_to_s);

        // if the cross product is zero, then the vectors are parallel or co-linear
        // which means that they will never cross however long they are
        if det == 0.0 {
            Vec::new()
        } else {
            // since the lines are not parallel or co-linear, then they may cross
            let q_minus_p = p.to_point_vec(q);
            let u = (q_minus_p).cross(p_to_r) / det;
            let point_of_intersect = q + (q_to_s * u);

            let t = (q_minus_p).cross(q_to_s) / det;

            if t >= 0.0 && u >= 0.0 && u <= 1.0 {
                vec![point_of_intersect]
            } else {
                Vec::new()
            }
        }
    }
}