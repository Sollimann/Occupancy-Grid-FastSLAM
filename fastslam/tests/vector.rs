use std::env;
use fastslam::geometry::vector::Vector;
use fastslam::math::scalar::PI;

#[test]
fn it_works() {
    assert_eq!(2 + 2, 4);
}

#[test]
fn create_vector_from_angle() {
    let u = Vector::from_angle(-PI/4.0);
    println!("from angle: {}", u)
}

#[test]
fn get_angle_from_vector() {
    let u = Vector::new(0.7, 0.7);
    println!("from angle: {}", u.angle())
}

#[test]
fn test_rotate_vector() {
    let u = Vector::new(1.0, 0.0);

    println!("rot pi/2: {}", u.rotate(0.0));
    println!("rot pi/2: {}", u.rotate(PI/2.0));
    println!("rot pi/2: {}", u.rotate(PI));
    println!("rot pi/2: {}", u.rotate(3.0*PI/2.0));
    println!("rot pi/2: {}", u.rotate(2.0*PI));
}