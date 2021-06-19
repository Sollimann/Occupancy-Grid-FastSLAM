use fastslam::odometry::Pose;
use fastslam::geometry::Point;
use fastslam::particlefilter::particle_filter::ParticleFilter;

#[test]
fn test_sample_distribution() {
    let init_pose = Pose::new(
        Point::new(0.049825244434982714, -0.0000000000000002220446049250313),
        0.00000000000000005551115123125786
    );

    let std_dev_sampling = Pose::new(Point::new(0.05, 0.05), 0.05);

    let pose_samples: Vec<Pose> = ParticleFilter::sample_distribution(&init_pose, std_dev_sampling, 10);
    println!("pose samples: {:?}", pose_samples);
}