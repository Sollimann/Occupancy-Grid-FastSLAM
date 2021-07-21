use nalgebra as na;
use na::{MatrixXx2};
use fastslam::geometry::Point;
use fastslam::scanmatching::icp::{to_na_homogeneous, icp};
use fastslam::pointcloud::PointCloud;
use fastslam::odometry::Pose;
use fastslam::math::utils::pose_relative_eq;

// type Matrix2xXf64 = DMatrix<f64, U2, Dynamic>;
type Matrix2xXf64 = MatrixXx2<f64>;

#[test]
fn test_homogeneous() {
    let p0 = na::Point2::new(2.0, 2.0);
    let p1 = na::Point2::new(3.0, 3.0);
    let p2 = na::Point2::new(4.0, 4.0);
    let p3 = na::Point2::new(5.0, 5.0);
    let v = vec![p0, p1, p2, p3];

    let dm1 = Matrix2xXf64::from_vec(vec![1.0, 2.0, 3.0, 4.0]);

    let p2 = na::Point2::new(2.3, 3.5);
    let _ = p2.to_homogeneous().data;
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_0() {
    let a = vec![
        Point::new(1.0, 2.0),
        Point::new(2.0, 2.0),
        Point::new(3.0, -3.0),
        Point::new(-1.0, -2.0)
    ];

    let b = vec![
        Point::new(3.0, 6.0),
        Point::new(-4.0, -40.0),
        Point::new(3.0, 3.0),
        Point::new(0.0, 0.0)
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&A, &B, 1, 0.000001);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_1() {
    let a = vec![
        Point::new(3.0, -2.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(40.0, -10.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
    ];

    let b = vec![
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),

    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&A, &B, 100, 0.000001);
    let pose_dif_expected = Pose::new(
        Point { x: 7.729003210378913, y: 2.9526877942756187 },
        0.011227837265104052 );

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}


#[test]
#[allow(non_snake_case)]
fn test_icp_case_2() {
    let a = vec![
        Point::new(3.0, -2.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(40.0, -10.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
    ];

    let b = vec![
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),

    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&B, &A, 100, 0.000001);
    let pose_dif_expected = Pose::new(
        Point { x: 2.8121879293507206, y: 16.15395993803767 },
        -0.04966336822821934);
    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_3() {
    let a = vec![
        Point::new(100.0, 100.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(400.0, -100.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
    ];

    let b = vec![
        Point::new(-200.0, -200.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(30.0, -30.0),

    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&B, &A, 100, 0.000001);
    let pose_dif_expected = Pose::new(
        Point { x: 20.129471360126942, y: 47.34280242686634 },
        -0.18743341847319145 );

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_4() {
    let a = vec![
        Point::new(100.0, 100.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(400.0, -100.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
    ];

    let b = vec![
        Point::new(-200.0, -200.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(30.0, -30.0),

    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&A, &B, 100, 0.000000000000001);

    let pose_dif_expected = Pose::new(
        Point { x: -1.8648786600632477, y: -1.8806433339899442 },
        0.1396554628187165);

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_5() {
    let a = vec![
        Point::new(100.0, 100.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(400.0, -100.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
        Point::new(100.0, 200.5),
        Point::new(300.0, 400.5),
        Point::new(500.0, 600.5),
        Point::new(700.0, 800.5),
        Point::new(900.0, 1000.5),
    ];

    let b = vec![
        Point::new(-200.0, -200.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(30.0, -30.0),
        Point::new(-100.0, -200.5),
        Point::new(-300.0, -400.5),
        Point::new(-500.0, -600.5),
        Point::new(-700.0, -800.5),
        Point::new(-900.0, -1000.5),
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&A, &B, 100, 0.00001);

    let pose_dif_expected = Pose::new(
        Point { x: -176.46623804632895, y: 24.5001641879306 },
        -0.9117716408207921);

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_real() {
    let A = PointCloud::new(vec![Point { x: 2.1081734290318326, y: -0.024841726273664107 }, Point { x: 2.109651267036626, y: 0.09888161003103749 }, Point { x: 2.111140850854742, y: 0.22358829580111772 }, Point { x: 2.1126543027127513, y: 0.3502931934977102 }, Point { x: 2.1142045235903404, y: 0.48007636212812405 }, Point { x: 2.1158056505242064, y: 0.6141213423375473 }, Point { x: 2.1174736047158964, y: 0.7537610435294234 }, Point { x: 2.1192267780970035, y: 0.9005352225176555 }, Point { x: 2.1210869252607303, y: 1.0562651552758773 }, Point { x: 2.123080358646588, y: 1.2231536967801429 }, Point { x: 2.1252395951081153, y: 1.4039231302528667 }, Point { x: 2.1276056850609137, y: 1.6020101614482711 }, Point { x: 2.130231596396501, y: 1.8218492170318237 }, Point { x: 2.133187272406088, y: 2.0692958896412277 }, Point { x: 2.1365674324227504, y: 2.3522800009910196 }, Point { x: 2.140504038092863, y: 2.68184926747529 }, Point { x: 2.14518705677693, y: 3.0739075943627823 }, Point { x: 1.3304927609694273, y: 2.101265028382214 }, Point { x: 1.162885104027928, y: 2.1032670513854774 }, Point { x: 1.0049162678253003, y: 2.1051539415768175 }, Point { x: 0.9461602618602064, y: 2.377618698021308 }, Point { x: 0.9947122677799471, y: 3.1636845218780256 }, Point { x: 0.7851894252455291, y: 3.1661872091544754 }, Point { x: 0.43430655621039127, y: 2.1119697027979005 }, Point { x: 0.3000883295878062, y: 2.113572899108015 }, Point { x: 0.16692845377118085, y: 2.1151634537363733 }, Point { x: 0.033768577954555484, y: 2.116754008364732 }, Point { x: -0.22149811751645562, y: 3.1782117887857715 }, Point { x: -0.4260513782248819, y: 3.1806551159034204 }, Point { x: -0.6355742207592984, y: 3.18315780317987 }, Point { x: -2.673622605939388, y: 9.028749908866804 }, Point { x: -3.312355948901916, y: 9.036379386355463 }, Point { x: -3.9837234841053495, y: 9.044398669445798 }, Point { x: -4.696056006310541, y: 9.052907266973207 }, Point { x: -5.459429710975569, y: 9.062025536058965 }, Point { x: -6.286296737773474, y: 9.071902213762698 }, Point { x: -6.789255825258231, y: 8.583410557441624 }, Point { x: -6.800556148045551, y: 7.570579738523252 }, Point { x: -6.810435264306135, y: 6.685129508408976 }, Point { x: -3.936112312402248, y: 3.454924837618806 }, Point { x: -4.190131584490467, y: 3.225615920777042 }, Point { x: -4.823073251392447, y: 3.2331762184387656 }, Point { x: -5.595322218792851, y: 3.242400499865965 }, Point { x: -6.567167720266135, y: 3.254008901803392 }, Point { x: -7.838993078231362, y: 3.2692004728165753 }, Point { x: -8.968040245782422, y: 3.0793335762570795 }, Point { x: -8.975486165206753, y: 2.455967557758114 }, Point { x: -8.982695349875565, y: 1.8524207709189244 }, Point { x: -8.98973354300628, y: 1.2631892496934018 }, Point { x: -8.996660739626913, y: 0.6832502615474552 }, Point { x: -9.00353331323314, y: 0.10788426554241867 }, Point { x: -0.008454838450606877, y: -0.008997120520776437 }, Point { x: -0.008568450892111251, y: -0.018508657145935636 }, Point { x: -0.00868388377920129, y: -0.028172599921627073 }, Point { x: -0.008802121085911996, y: -0.03807132631415831 }, Point { x: -0.008924241061909727, y: -0.048295106465218694 }, Point { x: -0.009051458037370036, y: -0.05894560306057828 }, Point { x: -0.009185174788173545, y: -0.07014025529957765 }, Point { x: -0.009327050564654327, y: -0.08201797420387384 }, Point { x: -0.009479092249847998, y: -0.09474677430817478 }, Point { x: -0.00964377994529414, y: -0.1085342875963967 }, Point { x: -0.009824244618102052, y: -0.12364263596240045 }, Point { x: -0.010024526196340089, y: -0.14041003873564498 }, Point { x: -0.010249959343277493, y: -0.15928310937146792 }, Point { x: -0.010507768422228225, y: -0.1808666653999249 }, Point { x: -0.010808018314837131, y: -0.20600333012110414 }, Point { x: -0.011165198071918708, y: -0.23590611450160073 }, Point { x: -0.011600991310073833, y: -0.2723903524144938 }, Point { x: -0.012149420270101213, y: -0.3183043568186493 }, Point { x: -0.012867132969648476, y: -0.3783906513977837 }, Point { x: -0.013856033937307638, y: -0.461180596301697 }, Point { x: -0.015319859951783366, y: -0.5837308607374146 }, Point { x: -0.017733905964221947, y: -0.7858327323114662 }, Point { x: -0.02252392704140163, y: -1.1868492082150344 }, Point { x: -0.036818448779133955, y: -2.3835743665451283 }, Point { x: 0.037362603195502664, y: -8.731978961323321 }, Point { x: 0.5866470830753059, y: -8.738539999071367 }, Point { x: 1.1402972599672014, y: -8.745153183755914 }, Point { x: 1.7028187365793042, y: -8.751872333444883 }, Point { x: 0.8815334504569597, y: -3.032887811853698 }, Point { x: 0.8891067092174567, y: -2.3988610528256533 }, Point { x: 0.9035598026811517, y: -1.9949694352821377 }, Point { x: 1.051655583822452, y: -1.9967383948030897 }, Point { x: 1.2087877592356822, y: -1.9986152913331803 }, Point { x: 1.3771790191151765, y: -2.000626674237631 }, Point { x: 1.5595761590014823, y: -2.0028053531622416 }, Point { x: 1.7594468298583568, y: -2.005192748153825 }, Point { x: 1.981265387362421, y: -2.0078423040439164 }, Point { x: 2.230940149800328, y: -2.0108245939157294 }, Point { x: 2.5164723415646955, y: -2.014235189985836 }, Point { x: 5.821107395499566, y: -4.232278272007286 }, Point { x: 6.386612401051843, y: -4.069963190301129 }, Point { x: 7.017310278839761, y: -3.8889360082957842 }, Point { x: 9.989441987895368, y: -4.779860323158052 }, Point { x: 9.998270999360281, y: -4.040703019604621 }, Point { x: 10.006670845513598, y: -3.3374750696102797 }, Point { x: 10.014734148440866, y: -2.662422231233198 }, Point { x: 10.022541087588403, y: -2.0088319496675897 }, Point { x: 10.03016285731749, y: -1.370743893757313 }, Point { x: 10.037664427184925, y: -0.7427188676640605 }]);
    let B = PointCloud::new(vec![Point { x: 2.2204649742643507, y: -0.06192828606028377 }, Point { x: 2.2238822557850604, y: 0.05231557828646829 }, Point { x: 2.227326697759543, y: 0.16746744953218964 }, Point { x: 2.2308263310414613, y: 0.2844644330993757 }, Point { x: 2.2344109872357265, y: 0.40430383570802664 }, Point { x: 2.2381133561460294, y: 0.5280785171286285 }, Point { x: 2.2419702531930437, y: 0.6570192615092781 }, Point { x: 2.246024206994762, y: 0.7925478521111491 }, Point { x: 2.2503255218261478, y: 0.9363460218351175 }, Point { x: 2.2549350413067417, y: 1.0904478466491971 }, Point { x: 2.2599279558446597, y: 1.2573670330574636 }, Point { x: 2.2653991884464797, y: 1.4402769722544282 }, Point { x: 2.2714712195168807, y: 1.64327233266366 }, Point { x: 2.278305782558299, y: 1.8717600613571652 }, Point { x: 2.2861219020007004, y: 2.1330624105737477 }, Point { x: 2.295224719608017, y: 2.437380639435288 }, Point { x: 2.3060535067675647, y: 2.7994001229920626 }, Point { x: 2.244229979133925, y: 3.1137807296134685 }, Point { x: 1.4634789320218475, y: 2.078328125668717 }, Point { x: 1.3055694550413446, y: 2.083051540275846 }, Point { x: 1.1553357263778654, y: 2.0875453566193714 }, Point { x: 1.103273717634832, y: 2.4108505465937635 }, Point { x: 1.104932244635362, y: 3.1478595934277287 }, Point { x: 0.9004558479637332, y: 3.153975925501776 }, Point { x: 0.6010063667524312, y: 2.1041265488599015 }, Point { x: 0.4678965278091331, y: 2.108108152558678 }, Point { x: 0.33478668886583496, y: 2.1120897562574537 }, Point { x: 0.20061889680818523, y: 2.1161030056262016 }, Point { x: -0.10585341645615826, y: 3.184076816022977 }, Point { x: -0.3152975275568995, y: 3.190341743199403 }, Point { x: -0.5316193512860523, y: 3.1968123976628413 }, Point { x: -2.8865345412074475, y: 9.090689490705833 }, Point { x: -3.557649799723861, y: 9.110764002228247 }, Point { x: -4.269714652017516, y: 9.132063404828147 }, Point { x: -5.03280150726092, y: 9.154888986088288 }, Point { x: -5.859357826043932, y: 9.179613076525472 }, Point { x: -6.515657621536941, y: 8.886131966901349 }, Point { x: -6.545774199132232, y: 7.8520527953120745 }, Point { x: -6.572103108229296, y: 6.948026551898668 }, Point { x: -3.756645712656191, y: 3.6498566576923523 }, Point { x: -3.868519211636429, y: 3.296626302719675 }, Point { x: -4.501223040962967, y: 3.3155518451669876 }, Point { x: -5.273181823925139, y: 3.3386428050084627 }, Point { x: -6.244662139489736, y: 3.367701886203714 }, Point { x: -7.516009589497024, y: 3.4057306431277974 }, Point { x: -8.796760078757085, y: 3.2893709519070016 }, Point { x: -8.815705000672617, y: 2.6560192434622816 }, Point { x: -8.834047589793165, y: 2.0428042506216717 }, Point { x: -8.851955118963053, y: 1.4441338393979553 }, Point { x: -8.869580235700491, y: 0.8549048181876058 }, Point { x: -8.88706637312809, y: 0.2703220438036777 }, Point { x: 0.10418041460461497, y: -0.017508479916949683 }, Point { x: 0.10361160733675129, y: -0.03652439650312045 }, Point { x: 0.10303368590263812, y: -0.05584501071701989 }, Point { x: 0.10244172397928569, y: -0.07563501550886398 }, Point { x: 0.10183032324729657, y: -0.0960748832410306 }, Point { x: 0.10119340409232286, y: -0.11736786279208854 }, Point { x: 0.10052394343568122, y: -0.1397487441971937 }, Point { x: 0.09981363414448474, y: -0.16349524498032025 }, Point { x: 0.09905242864259362, y: -0.18894326779349813 }, Point { x: 0.09822791015802368, y: -0.21650792037823863 }, Point { x: 0.09732440332262249, y: -0.24671324929668023 }, Point { x: 0.09632168200255814, y: -0.2802354387249746 }, Point { x: 0.09519303789723399, y: -0.31796737953319393 }, Point { x: 0.09390230181334447, y: -0.3611182517050403 }, Point { x: 0.09239908333806268, y: -0.41137266783461995 }, Point { x: 0.09061084219940196, y: -0.4711557371622066 }, Point { x: 0.08842901811511805, y: -0.544096761541637 }, Point { x: 0.08568327676637222, y: -0.6358902237648756 }, Point { x: 0.08209000657223686, y: -0.7560176028327955 }, Point { x: 0.07713901660886774, y: -0.9215351998845901 }, Point { x: 0.06981028689496749, y: -1.1665435195664557 }, Point { x: 0.05772422573255742, y: -1.5705951973569343 }, Point { x: 0.03374270781213201, y: -2.3723264166103513 }, Point { x: -0.03782364308567593, y: -4.764876294111998 }, Point { x: 0.14355691652541275, y: -8.734958281747383 }, Point { x: 0.6926349943056962, y: -8.751382396769701 }, Point { x: 1.24607712862021, y: -8.767937050178784 }, Point { x: 0.932249822478054, y: -3.306687624504094 }, Point { x: 0.9574710665498476, y: -2.4635108569674227 }, Point { x: 0.989754569486883, y: -2.0103775202595133 }, Point { x: 1.1305986874472744, y: -2.0145904730019537 }, Point { x: 1.2786388193188845, y: -2.019018674112455 }, Point { x: 1.4357119498981616, y: -2.0237170717865682 }, Point { x: 1.6040399341693818, y: -2.028752126521082 }, Point { x: 1.7863685355223808, y: -2.0342059700483737 }, Point { x: 1.9861641018976828, y: -2.0401822883460605 }, Point { x: 2.2078993076638804, y: -2.0468148687956815 }, Point { x: 2.4574802509660723, y: -2.0542803755898063 }, Point { x: 5.451920109049022, y: -4.443862770600255 }, Point { x: 5.9643107121382215, y: -4.306703117872003 }, Point { x: 6.52829118099556, y: -4.155733594255426 }, Point { x: 7.157288769575304, y: -3.9873599154713806 }, Point { x: 10.017181633788809, y: -4.887160034519094 }, Point { x: 10.03894839203253, y: -4.159470916430785 }, Point { x: 10.059657100049305, y: -3.467153700208569 }, Point { x: 10.079536105322005, y: -2.802574452651803 }, Point { x: 10.098783079654654, y: -2.159124765694625 }, Point { x: 10.11757354331433, y: -1.5309367840081693 }, Point { x: 10.136067670137216, y: -0.912655702054211 }]);

    let pose_dif = icp(&A, &B, 100, 0.00001);
    println!("pose diff: {:?}", pose_dif);
}