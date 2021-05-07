use fastslam::pointcloud::PointCloud;
use fastslam::geometry::point::Point;
use rayon::iter::ParallelIterator;
use fastslam::geometry::vector::Vector;

#[test]
fn test_empty_pointcloud() {
    let mut cloud = PointCloud::empty();

    assert_eq!(cloud.size(), 0);

    cloud.add(Point::new(0.0, 0.0));
    cloud.add(Point::new(1.0, 1.0));
    cloud.add(Point::new(2.0, 2.0));
    cloud.add(Point::new(3.0, 3.0));

    assert_eq!(cloud.size(), 4);
}

#[test]
fn test_new_pointcloud() {
    let p0 = Point::new(1.0, 1.0);
    let p1 = Point::new(10.0, 10.0);
    let mut cloud1 = PointCloud::new(vec![p0, p0, p0]);

    assert_eq!(cloud1.size(), 3);

    let cloud2: Vec<Vector> = cloud1
        .iter()
        .map(|m| m.to_point_vec(p1))
        .collect();

    for vec in cloud2.iter() {
        assert_eq!(vec.x, 9.0);
        assert_eq!(vec.y, 9.0);
    }
}

#[test]
fn test_mutate_pointcloud() {
    let p0 = Point::new(1.0, 1.0);
    let p1 = Point::new(2.0, 2.0);
    let p3 = Point::new(-1.0, -1.0);
    let mut cloud = PointCloud::new(vec![p0, p0, p0]);

    assert_eq!(cloud.size(), 3);

    cloud
        .iter_mut()
        .for_each(|p| *p = *p - p1);

    cloud
        .iter()
        .for_each(|p| assert_eq!(*p, p3))
}


#[test]
fn test_pointcloud_centroid() {
    let p0 = Point::new(-1.0, -1.0);
    let p1 = Point::new(-1.0, -1.0);
    let p2 = Point::new(-1.0, -1.0);
    let p3 = Point::new(1.0, 1.0);
    let mut cloud = PointCloud::new(vec![p0, p1, p2, p3]);

    assert_eq!(cloud.size(), 4);

    let centroid = cloud.centroid();
    assert_eq!(Point::new(-0.5, -0.5), centroid)
}