#include "gd/ground_detect.h"
#include "gd/map_builder.h"
#include "gd/lidarRoadDetect.h"
#include "chunkmap/chunk_map.h"
#include "method.h"

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

ChunkMap::Ptr chunk_map;
GroundDetect::Ptr ground_detector;
MapBuilder::Ptr map_builder;
ImageKeyPoints::Ptr image_kp;

float submap_radius;

void init()
{
    // float submap_radius;
    float robot_height;
    float resolution;
    float chunk_base;
    float hit_probability;
    float miss_probability;
    // resolution = 0.1f; // nh.param<float>("resolution", resolution, 0.1f);
    // resolution = 0.5f;
    resolution = 0.3f;
    chunk_base = 3.0f;        // nh.param<float>("chunk_base", chunk_base, 3.0f);
    hit_probability = 0.45f;  // nh.param<float>("hit_probability", hit_probability, 0.45f);
    miss_probability = 0.51f; // nh.param<float>("miss_probability", miss_probability, 0.51f);
    // miss_probability = 0.53f;
    // submap_radius = 20.0f;    // nh.param<float>("submap_radius", submap_radius, 20.0f);
    // submap_radius = 100.0f;
    submap_radius = 60.0f;
    robot_height = 0.5f; // nh.param<float>("robot_height", robot_height, 0.5f);
    // if(save_path != "") ROS_INFO_STREAM("will be saved to " << save_path);
    // init chunk map
    chunk_map = std::make_shared<ChunkMap>(ChunkMap::Config{
        .resolution = resolution,
        .chunk_base = chunk_base,
    });
    chunk_map->desc_type = ChunkMap::DescType::compressed;
    chunk_map->obstacle_segments = obstacle_segments;
    chunk_map->obstacle_bits = obstacle_bits;
#ifdef BUILD_MAP
    chunk_map->desc_type = ChunkMap::DescType::compressed;
#endif
    // init ground detector
    ground_detector = GroundDetect::create(
        {
            .resolution = resolution,
            .hit_probability = hit_probability,
            .miss_probability = miss_probability,
            .submap_radius = submap_radius,
            .robot_height = robot_height,
        });
    map_builder = MapBuilder::create(
        {
            .resolution = resolution,
            .submap_radius = submap_radius,
        },
        chunk_map);

    double fx = 7.188560000000e+02;
    double fy = 7.188560000000e+02;
    double cx = 6.071928000000e+02;
    double cy = 1.852157000000e+02;
    Eigen::Matrix4f trans;
    trans << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
        -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
        9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
        0, 0, 0, 1;
    image_kp.reset(new ImageKeyPoints(trans, fx, fy, cx, cy));
}

void internal::Builder::iter(int index, const clf::Frame &data)
{
    auto cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr>(data.data[0]);
    auto image = std::any_cast<cv::Mat3b>(data.data[1]);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    if (cloud->size() == 0)
        return;

    size_t submap_index = index;
    auto detect_info = ground_detector->process(cloud);
    auto submap_info = std::make_shared<SubmapInfo>();
    submap_info->detect = detect_info;
    submap_info->has_old_pose = false;
    submap_info->inited = false;

    submap_info->frame_kp = (*image_kp)(*cloud, image);

    map_builder->submaps.emplace(submap_index, submap_info);

    auto &current = map_builder->submaps.at(submap_index);
    current->pose = data.pose.cast<double>();
    current->inited = true;

    need_update_submaps.push_back(submap_index);

    std::cout << submap_index << std::endl;
}

void internal::Builder::process()
{
    map_builder->draw_multi<8, 8>(need_update_submaps);

    static constexpr int obstacle_base = -1;

    // std::vector<cv::Mat> descs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr kpPC(new pcl::PointCloud<pcl::PointXYZ>);
    ImageKeyPoints::FrameKP keypoints;
    for (const auto &frame : map_builder->submaps)
    {
        Eigen::Isometry3f trans = frame.second->pose.cast<float>();
        for (const auto &kp : frame.second->frame_kp)
        {
            // if (std::get<0>(kp).getVector3fMap().norm() > 50)
            //     continue;
            pcl::PointXYZ p;
            p.getVector3fMap() = trans * std::get<0>(kp).getVector3fMap();
            // TODO: check here
            ChunkMap::CellIndex ci;
            if (!chunk_map->query(p.getVector3fMap(), ci))
                continue;
            float height = std::get<0>(kp).getVector3fMap().z() - obstacle_base;
            if (height > 0 && height < obstacle_type_desc.max_height)
            {
                auto wrapped = ObstacleTypeDesc::wrapper(chunk_map->at(ci.chunk).getLayers()[ci.layer].occupancy(ci.cell));
                auto value = uint16_t(wrapped(int(height) * obstacle_bits, obstacle_bits));
                if (value != 0 && value <= ObstacleProb::init_value)
                    continue;
            }

            //
            kpPC->push_back(p);
            // descs.push_back(std::get<1>(kp));
            keypoints.emplace_back(std::make_tuple(p, std::get<1>(kp)));
        }
    }

    map_builder->saveMap("map.chunkmap");

    pcl::io::savePCDFileBinary("kp.pcd", *kpPC);

    std::ofstream kpofs("keypoints.pack", std::ios::binary);
    saveKeyPoints(kpofs, keypoints);
    kpofs.close();
}

struct ImageLocalizerImpl : public ImageLocalizer
{
    ImageKeyPoints::FrameKP keypoints;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat1f K;
    Eigen::Matrix4f cptrans;

    ImageLocalizerImpl();
    virtual Eigen::Matrix4f operator()(const clf::Frame &data, const Eigen::Matrix4f &lastPose) override;
};

ImageLocalizer::Ptr ImageLocalizer::create()
{
    return ImageLocalizer::Ptr(new ImageLocalizerImpl());
}

ImageLocalizerImpl::ImageLocalizerImpl()
{
    std::ifstream kpifs("keypoints.pack", std::ios::binary);
    loadKeyPoints(kpifs, keypoints);
    kpifs.close();

#if 1
    matcher.reset(new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(12, 20, 2)));
    std::vector<int> localMIndices;
    {
        cv::Mat1b descMat(cv::Size(32, keypoints.size()));
        for (int i = 0; i < keypoints.size(); i++)
        {
            const auto &kp = keypoints[i];
            localMIndices.push_back(i);
            std::get<1>(kp).copyTo(descMat.row(i));
        }
        matcher->add(descMat);
    }
#else
    matcher.reset(new cv::BFMatcher(cv::NORM_HAMMING));
    {
        cv::Mat1b descMat(cv::Size(32, keypoints.size()));
        for (int i = 0; i < keypoints.size(); i++)
        {
            const auto &kp = keypoints[i];
            std::get<1>(kp).copyTo(descMat.row(i));
        }
        matcher->add(descMat);
    }
#endif
    matcher->train();

    // TODO:
    K = cv::Mat1f::zeros(3, 3);
    K(0, 0) = 7.188560000000e+02;
    K(1, 1) = 7.188560000000e+02;
    K(0, 2) = 6.071928000000e+02;
    K(1, 2) = 1.852157000000e+02;
    K(2, 2) = 1.0;

    // TODO:
    cptrans << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
        -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
        9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
        0, 0, 0, 1;
}

Eigen::Matrix4f makeTransform(const cv::Mat1f &rvec, const cv::Mat1f &tvec)
{
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    Eigen::Vector3f ervec(rvec(0, 0), rvec(1, 0), rvec(2, 0));
    result.block<3, 3>(0, 0) = Eigen::AngleAxisf(ervec.norm(), ervec.normalized()).matrix();
    result.block<3, 1>(0, 3) = Eigen::Vector3f{tvec(0, 0), tvec(1, 0), tvec(2, 0)};
    return result;
}

Eigen::Matrix4f ImageLocalizerImpl::operator()(const clf::Frame &data, const Eigen::Matrix4f &lastPose)
{

    auto cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr>(data.data[0]);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    if (cloud->size() == 0)
        return Eigen::Matrix4f::Zero();

    auto image = std::any_cast<cv::Mat3b>(data.data[1]);
    auto frame_kp = (*image_kp)(image);

    // auto time_start = std::chrono::steady_clock::now();

#if 0
        std::vector<int> localMIndices;
        matcher.clear();
        {
            std::cout << "prepare stage" << std::endl;
            std::vector<cv::Mat1b> descMats;
            for (int i = 0; i < keypoints.size(); i++)
            {
                const auto &kp = keypoints[i];
                Eigen::Vector3f p = (pose.inverse() * std::get<0>(kp).getVector4fMap()).block<3, 1>(0, 0);
                // if (p.x() < -1 || p.norm() > 50 /*50*/ || std::abs(std::atan2(p.y(), p.x() - 1)) > M_PI_4 * 3)
                if (p.norm() > 180)
                    continue;
                localMIndices.push_back(i);
                descMats.push_back(std::get<1>(kp));
            }
            if (descMats.size() == 0)
                continue;
            cv::Mat1b descMat(cv::Size(32, descMats.size()));
            for (int i = 0; i < descMats.size(); i++)
                descMats[i].copyTo(descMat.row(i));
            matcher.add(descMat);
        }
#endif

    std::vector<int> camDebugI;

    std::vector<cv::Point3f> objP;
    std::vector<cv::Point2f> camP;

    std::vector<std::tuple<float, cv::Point3f, cv::Point2f, int>> matches;

#if 0
        for (const auto &kp : frame_kp)
        {
            std::vector<cv::DMatch> matched;
            matcher.match(std::get<1>(kp), matched);
            if (matched[0].distance >= 12) // TODO:
                continue;
            camP.push_back(std::get<0>(kp));
            pcl::PointXYZ globalP = std::get<0>(keypoints[matched[0].trainIdx]);
            // globalP.getVector4fMap() = cptrans * globalP.getVector4fMap();
            objP.emplace_back(globalP.x, globalP.y, globalP.z);
        }
#else

    auto frame_kp3d = image_kp->debug(*cloud, image);

    for (int i = 0; i < frame_kp.size(); i++)
    {
#if 1
        if (std::get<0>(frame_kp3d[i]).x == INFINITY)
            continue;
#endif
        const auto &kp = frame_kp[i];
        std::vector<std::vector<cv::DMatch>> matched;
        matcher->knnMatch(std::get<1>(kp), matched, 1);
        for (const auto &m : matched[0])
        {
            if (m.distance > 32)
                // if (m.distance > 24)
                // if (m.distance > 28)
                continue;
            // pcl::PointXYZ globalP = std::get<0>(keypoints[localMIndices[m.trainIdx]]);
            pcl::PointXYZ globalP = std::get<0>(keypoints[m.trainIdx]);

#if 0
            if ((globalP.getVector3fMap() - (pose * std::get<0>(frame_kp3d[i]).getVector4fMap()).block<3, 1>(0, 0)).norm() > 10.0f)
                continue;
#endif

#if 0
            Eigen::Vector3f localer = (cptrans * pose.inverse() * globalP.getVector4fMap()).block<3, 1>(0, 0);
            cv::Point2f off = cv::Point2f{localer.x() * K(0, 0) / localer.z() + K(0, 2), localer.y() * K(1, 1) / localer.z() + K(1, 2)} - std::get<0>(kp);
            if (std::sqrt(off.x * off.x + off.y * off.y) > 200)
                continue;
#endif

            matches.push_back(std::make_tuple(m.distance, cv::Point3f(globalP.x, globalP.y, globalP.z), std::get<0>(kp), i));
        }
    }
    std::sort(matches.begin(), matches.end(), [](const auto &a, const auto &b) -> bool
              { return std::get<0>(a) < std::get<0>(b); });
    int mth = matches.size();
    // if (mth > 10)
    //     mth *= 0.8;
    for (int i = 0; i < mth; i++)
    {
        objP.push_back(std::get<1>(matches[i]));
        camP.push_back(std::get<2>(matches[i]));
        camDebugI.push_back(std::get<3>(matches[i]));
    }
#endif

#if 0
        {
            std::cout << "debug stage" << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB> save;
            auto frame_kp3d = image_kp->debug(*cloud, image);

            // line
            int dropped = 0;
            for (int i = 0; i < camP.size(); i++)
            {
                if (std::get<0>(frame_kp3d[camDebugI[i]]).x == INFINITY)
                {
                    dropped += 1;
                    continue;
                }
                Eigen::Vector3f start = (pose * std::get<0>(frame_kp3d[camDebugI[i]]).getVector4fMap()).block<3, 1>(0, 0);
                Eigen::Vector3f end{objP[i].x, objP[i].y, objP[i].z};
                Eigen::Vector3f direct = (end - start).normalized() * 0.1f;
                // {
                //     pcl::PointXYZI p;
                //     p.getVector3fMap() = start;
                //     p.intensity = 1;
                //     save.push_back(p);
                // }
                while ((end - start).norm() > 0.1)
                {
                    pcl::PointXYZRGB p;
                    p.getVector3fMap() = start;
                    // p.intensity = 2;
                    p.getBGRVector3cMap() = pcl::Vector3c{255, 0, 0};
                    save.push_back(p);
                    start += direct;
                }
            }
            std::cout << "dropped: " << dropped / (double)camP.size() << std::endl
                      << "useful: " << camP.size() - dropped << std::endl;

#if 1
            // map
            for (const auto id : localMIndices)
            {
                pcl::PointXYZRGB p;
                p.getVector3fMap() = std::get<0>(keypoints[id]).getVector3fMap();
                // p.intensity = 0;
                p.getBGRVector3cMap() = pcl::Vector3c{0, 0, 255};
                save.push_back(p);
            }
#endif

            // frame
            for (const auto &it : frame_kp3d)
            {
                if (std::get<0>(it).x == INFINITY)
                    continue;
                pcl::PointXYZRGB p;
                p.getVector4fMap() = pose * std::get<0>(it).getVector4fMap();
                // p.intensity = 1;
                p.getBGRVector3cMap() = pcl::Vector3c{0, 255, 0};
                save.push_back(p);
            }

            // lidar
            for (const auto &op : *cloud)
            {
                pcl::PointXYZRGB p;
                p.getVector4fMap() = pose * op.getVector4fMap();
                p.getBGRVector3cMap() = pcl::Vector3c{255, 255, 255};
                save.push_back(p);
            }
            pcl::io::savePCDFileBinary("debug/" + std::to_string(index) + ".pcd", save);

            cv::Mat3b mark = image.clone();
            // for (int i = 0; i < frame_kp.size(); i++)
            // {
            //     cv::circle(mark, std::get<0>(frame_kp[i]), 4, cv::Scalar(255, 0, 0), 1);
            //     if (std::get<0>(frame_kp3d[i]).x == INFINITY)
            //         continue;
            //     Eigen::Vector3f p = (cptrans * std::get<0>(frame_kp3d[i]).getVector4fMap()).block<3, 1>(0, 0);
            //     cv::Point2f pp;
            //     pp.x = p.x() * K(0, 0) / p.z() + K(0, 2);
            //     pp.y = p.y() * K(1, 1) / p.z() + K(1, 2);
            //     cv::circle(mark, pp, 4, cv::Scalar(0, 255, 255), 1);
            // }
            for (int i = 0; i < camP.size(); i++)
            {
                if (std::get<0>(frame_kp3d[camDebugI[i]]).x == INFINITY)
                    continue;
                cv::circle(mark, camP[i], 4, cv::Scalar(0, 0, 255), 1);
                Eigen::Vector3f p = (cptrans * std::get<0>(frame_kp3d[camDebugI[i]]).getVector4fMap()).block<3, 1>(0, 0);
                cv::Point2f pp;
                pp.x = p.x() * K(0, 0) / p.z() + K(0, 2);
                pp.y = p.y() * K(1, 1) / p.z() + K(1, 2);
                cv::circle(mark, pp, 4, cv::Scalar(0, 255, 0), 1);

                Eigen::Vector3f localer = (cptrans * pose.inverse() * Eigen::Vector4f{objP[i].x, objP[i].y, objP[i].z, 1.0f}).block<3, 1>(0, 0);
                cv::Point2f localer2d = cv::Point2f{localer.x() * K(0, 0) / localer.z() + K(0, 2), localer.y() * K(1, 1) / localer.z() + K(1, 2)};
                cv::line(mark, camP[i], localer2d, cv::Scalar(255, 0, 0), 2);
            }
            cv::imwrite("debug/" + std::to_string(index) + ".png", mark);
            // if (index == 4)
            //     return 0;
        }
#endif

    if (camP.size() < 4)
        return Eigen::Matrix4f::Zero();

    cv::Mat rvec, tvec;
    std::vector<int> inlier;
    if (!cv::solvePnPRansac(objP, camP, K, cv::Mat(), rvec, tvec, false, 100, 8.0 /*8.0*/, 0.9 /*0.9*/, inlier))
        return Eigen::Matrix4f::Zero();

    // std::cout << "inlier: " << (double)inlier.size() / camP.size() << std::endl;
    // if (!cv::solvePnPGeneric(objP, camP, K, cv::Mat(), rvec, tvec, false, cv::SolvePnPMethod::SOLVEPNP_EPNP))
    //     continue;
    // if (!cv::solvePnP(objP, camP, K, cv::Mat(), rvec, tvec))
    //     continue;
    // Eigen::Matrix4f oldpose = pose;

    Eigen::Matrix4f pose = makeTransform(rvec, tvec).inverse() * cptrans;

    // if ((oldpose * pose.inverse()).block<3, 1>(0, 3).norm() > 16.0f)
    //     pose = oldpose;

    // auto time_stop = std::chrono::steady_clock::now();

    return pose;
}


struct LidaLocalizerImpl : public LidaLocalizer
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obsCloud;

    LidaLocalizerImpl() {
        chunk_map->desc_type = ChunkMap::DescType::uncompressed;
        std::ifstream ifs("map.chunkmap", std::ios::binary);
        ChunkMap::load(ifs, chunk_map);
        ifs.close();
        obsCloud = chunk_map->generateObstacleCloud();
        icp.setMaxCorrespondenceDistance(4.0f);
        icp.setMaximumIterations(30);
        icp.setEuclideanFitnessEpsilon(0.001);
        icp.setInputTarget(obsCloud);
    }
    virtual Eigen::Matrix4f operator()(const clf::Frame &data, const Eigen::Matrix4f &lastPose) override;
};

LidaLocalizer::Ptr LidaLocalizer::create()
{
    return LidaLocalizer::Ptr(new LidaLocalizerImpl());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateLocalCloud(const ObstacleFeature::Segments &seg, const ObstacleFeature::Bits &bit, const ObstacleTypeDesc::Type &obstacle, float resolution, float radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int r = 0; r < obstacle.rows; r++)
        for (int c = 0; c < obstacle.cols; c++)
        {
            uint64_t value = ObstacleTypeDesc_at(seg, bit, obstacle, r, c);
            for (int b = 0; b < seg; b++)
                if ((value & (1 << b)) != 0)
                {
                    pcl::PointXYZ p;
                    p.x = c * resolution - radius;
                    p.y = r * resolution - radius;
                    p.z = b;
                    cloud->push_back(p);
                }
        }
    return cloud;
}

Eigen::Matrix4f LidaLocalizerImpl::operator()(const clf::Frame &data, const Eigen::Matrix4f &lastPose)
{
    auto cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr>(data.data[0]);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    if (cloud->size() == 0)
        return Eigen::Matrix4f::Zero();

    Eigen::Vector3f limit = lastPose.block<3,1>(0,3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tglobalPC(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &p : *obsCloud)
    {
        if (p.x < limit.x() - 100 || p.x > limit.x() + 100)
            continue;
        if (p.y < limit.y() - 100 || p.y > limit.y() + 100)
            continue;
        tglobalPC->push_back(p);
    }
    icp.setInputTarget(tglobalPC);

    auto detect_info = ground_detector->process(cloud);
    auto localPC = generateLocalCloud(chunk_map->obstacle_segments, chunk_map->obstacle_bits, detect_info->obstacle_info, chunk_map->resolution(), submap_radius);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);

    icp.setInputSource(localPC);
    icp.align(*temp, lastPose);
    return icp.getFinalTransformation();
}
