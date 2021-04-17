use fastslam::geometry::point::Point;
use fastslam::geometry::line::Line;
use fastslam::geometry::ray::Ray;
use fastslam::geometry::vector::Vector;
use fastslam::geometry::target::Target;

#[test]
fn test_line_and_raycast_intersection() {
    let p = Point::new(-1.0, 1.0);
    let r = Point::new(1.0, -1.0);
    let p_to_r = Line::new(p, r);

    let ray = Ray::new(Point::new(-1.0, -1.0), Vector::new(2.0, 2.0));

    let intersection_point = p_to_r.intersect(&ray);
    let len = intersection_point.len();
    assert_eq!(len, 1);
    if len == 1 {
        assert_eq!(intersection_point[0], Point::new(0.0,0.0));
    }
}

#[test]
fn test_line_and_raycast_parallel() {
    let p = Point::new(-1.0, 1.0);
    let r = Point::new(1.0, 1.0);
    let p_to_r = Line::new(p, r);

    let ray = Ray::new(Point::new(-1.0, -1.0), Vector::new(2.0, 0.0));

    let intersection_point = p_to_r.intersect(&ray);
    let len = intersection_point.len();
    assert_eq!(len, 0);
}

#[test]
fn test_line_and_raycast_colinear_and_opposite() {
    let p = Point::new(-1.0, 1.0);
    let r = Point::new(1.0, 1.0);
    let p_to_r = Line::new(p, r);

    let ray = Ray::new(r, Vector::new(-2.0, 0.0));

    let intersection_point = p_to_r.intersect(&ray);
    let len = intersection_point.len();
    assert_eq!(len, 0);
}

#[test]
fn test_line_and_raycast_intersection_neg_sign() {
    let p = Point::new(-1.0, 1.0);
    let r = Point::new(1.0, -1.0);
    let p_to_r = Line::new(r, p);

    let ray = Ray::new(Point::new(1.0, 1.0), Vector::new(-2.0, -2.0));

    let intersection_point = p_to_r.intersect(&ray);
    let len = intersection_point.len();
    assert_eq!(len, 1);
    if len == 1 {
        assert_eq!(intersection_point[0], Point::new(0.0,0.0));
    }
}

#[test]
fn test_line_and_raycast_can_cross_but_too_short() {
    let p = Point::new(-1.0, 1.0);
    let r = Point::new(1.0, -1.0);
    let p_to_r = Line::new(p, r);

    let ray = Ray::new(Point::new(-2.0, -2.0), Vector::new(1.0, 1.0));

    let intersection_point = p_to_r.intersect(&ray);
    let len = intersection_point.len();
    assert_eq!(len, 0);
}