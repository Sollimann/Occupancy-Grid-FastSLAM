use fastslam::odometry::Pose;
use fastslam::geometry::Point;

#[test]
fn test_pose_add_assign() {
    let mut p1 = Pose::new(Point::new(1.0, 1.0), 2.0);
    let p2 = Pose::new(Point::new(1.0, 1.0), 2.0);

    p1 += p2.clone();

    assert_eq!(p1, p2.clone() * 2.0);
}

#[test]
fn test_pose_add_assign_negative() {
    let mut p1 = Pose::new(Point::new(1.0, 1.0), 2.0);
    let p2 = Pose::new(Point::new(-1.0, -1.0), -2.0);

    p1 += p2.clone();

    assert_eq!(p1, p2.clone() * 0.0);
}
