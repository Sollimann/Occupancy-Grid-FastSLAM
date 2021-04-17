use fastslam::geometry::ray::Ray;
use fastslam::geometry::point::Point;
use fastslam::geometry::vector::Vector;

#[test]
fn test_point_vector() {
    let p0 = Point::new(1.0, 0.0);
    let dir = Vector::new(2.0, 2.0);
    let ray = Ray::new(p0, dir);

    // ray light
    let p = ray.origin;
    let r_vec = ray.direction;
    let p_r = p + r_vec;

    assert_eq!(p_r, Point::new(3.0, 2.0));

    let p_to_r = p.to_point_vec(p_r);
    assert_eq!(p_to_r, dir);
}