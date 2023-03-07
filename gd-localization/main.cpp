#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include "gd/mutli_execute.h"
#include <pcl/registration/icp.h>
// #include "icp/icp.h"
#include "icp/corr_est.h"
#include <chrono>

pcl::PointCloud<pcl::PointXYZ>::Ptr loadBinPC(const std::string &filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    FILE *f = fopen(filename.c_str(), "rb");
    if (!f)
        return nullptr;
    float buf[4];
    while (fread(buf, sizeof(float), 4, f))
    {
        pcl::PointXYZ p;
        p.x = buf[0];
        p.y = buf[1];
        p.z = buf[2];
        // p.intensity = buf[3];
        cloud->push_back(p);
    }
    fclose(f);
    return cloud;
}

Eigen::Matrix4f parsePose(const std::string &data)
{
    std::stringstream ss;
    ss << data;
    float a[12];
    ss >> a[0] >> a[1] >> a[2] >> a[3] >> a[4] >> a[5] >> a[6] >> a[7] >> a[8] >> a[9] >> a[10] >> a[11];
    Eigen::Matrix4f pose;
    pose << a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], 0, 0, 0, 1;
    return pose;
}

std::tuple<int, Eigen::Isometry3f> parseTQPose(const std::string &data)
{
    std::stringstream ss;
    ss << data;
    int index;
    float timestamp;
    float a[7];
    ss >> index >> timestamp >> a[0] >> a[1] >> a[2] >> a[3] >> a[4] >> a[5] >> a[6];
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.rotate(Eigen::Quaternionf(a[6], a[3], a[4], a[5]));
    pose.pretranslate(Eigen::Vector3f{a[0], a[1], a[2]});
    return {index, pose};
}

Eigen::Isometry3f poseCalib(const Eigen::Matrix4f &calib, const Eigen::Matrix4f &pose)
{
    Eigen::Matrix4f mat = calib.inverse() * pose * calib;
    Eigen::Isometry3f ret = Eigen::Isometry3f::Identity();
    ret.rotate(mat.block<3, 3>(0, 0));
    ret.pretranslate(mat.block<3, 1>(0, 3));
    return ret;
}

struct InfoDesc
{
    int type; // 0: kitti-bin, 1: pcd
    std::string filename;
    Eigen::Isometry3f pose;
};

namespace fs = std::filesystem;

std::vector<InfoDesc> loadKITTIData(const std::string &baseFolder)
{
    Eigen::Matrix4f calib;
    calib << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03, -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01, 0, 0, 0, 1;
    std::vector<InfoDesc> descs;
    std::ifstream ifs(baseFolder + "/poses.txt");
    std::string line;
    int index = 0;
    char nameBuffer[7] = {};
    while (std::getline(ifs, line))
    {
        if (line == "")
            continue;
        sprintf(nameBuffer, "%06d", index);
        index += 1;
        descs.push_back({
            .type = 0,
            .filename = baseFolder + "/velodyne/" + nameBuffer + ".bin",
            .pose = poseCalib(calib, parsePose(line)),
        });
    }
    return descs;
}

std::vector<InfoDesc> loadMulRanData(const std::string &baseFolder)
{
    // Eigen::Matrix4f calib;
    // calib << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03, -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01, 0, 0, 0, 1;
    Eigen::Isometry3f temp = Eigen::Isometry3f::Identity();
    Eigen::Matrix4f calib = temp.rotate(
                                    Eigen::AngleAxisf(179.6654 * M_PI / 180.0, Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(0.0003 * M_PI / 180.0, Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(0.0001 * M_PI / 180.0, Eigen::Vector3f::UnitX()))
                                .pretranslate(Eigen::Vector3f{1.7042, -0.021, 1.8047})
                                .matrix();
    std::vector<InfoDesc> descs;
    std::ifstream ifs(baseFolder + "/pair.txt");
    std::string line;
    // int index = 0;
    // char nameBuffer[7] = {};
    while (std::getline(ifs, line))
    {
        if (line == "")
            continue;
        // sprintf(nameBuffer, "%06d", index);
        // index += 1;
        descs.push_back({
            .type = 0,
            .filename = baseFolder + "/Ouster/" + line.substr(0, 19) + ".bin",
            .pose = poseCalib(calib, parsePose(line.substr(20))),
        });
    }
    Eigen::Isometry3f basePose = Eigen::Isometry3f::Identity();
    basePose.pretranslate(Eigen::Vector3f{-329604, -4027910, 24});
    for (auto &it : descs)
    {
        it.pose = basePose.inverse() * it.pose;
    }
    return descs;
}

std::vector<InfoDesc> loadAplloData(const std::string &baseFolder)
{
    std::vector<InfoDesc> descs;
    std::ifstream ifs(baseFolder + "/poses/gt_poses.txt");
    std::string line;
    while (std::getline(ifs, line))
    {
        if (line == "")
            continue;
        auto data = parseTQPose(line);
        descs.push_back({
            .type = 1,
            .filename = baseFolder + "/pcds/" + std::to_string(std::get<0>(data)) + ".pcd",
            .pose = std::get<1>(data),
        });
    }
    Eigen::Isometry3f basePose = Eigen::Isometry3f::Identity();
    basePose.pretranslate(Eigen::Vector3f{586433.190940, 4139159.755922, -21.101942});
    for (auto &it : descs)
    {
        it.pose = basePose.inverse() * it.pose;
    }
    return descs;
}

std::vector<InfoDesc> loadAplloDataAll(const std::vector<std::string> &folders)
{
    std::vector<InfoDesc> ret;
    for (const auto &it : folders)
    {
        auto arr = loadAplloData(it);
        std::copy(arr.begin(), arr.end(), std::back_insert_iterator(ret));
    }
    return ret;
}

#include "gd/ground_detect.h"
#include "gd/map_builder.h"
#include "gd/lidarRoadDetect.h"

ChunkMap::Ptr chunk_map;
GroundDetect::Ptr ground_detector;
MapBuilder::Ptr map_builder;
ImageKeyPoints::Ptr image_kp;

float submap_radius;

// #define MAP_ITER
// #define BUILD_MAP
// #define LOCALIZATION
#define NEWL

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

LidarIris::FeatureDesc featureDecompress(const ChunkMap::CompressedDesc &compDesc)
{
    LidarIris::FeatureDesc desc;
    desc.img = cv::imdecode(compDesc.img, -1);
    desc.T = cv::imdecode(compDesc.T, -1);
    desc.M = cv::imdecode(compDesc.M, -1);
    return desc;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr makeCurbPC(const cv::Mat1f &hits)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ret(new pcl::PointCloud<pcl::PointXYZ>);
    float da = 2 * M_PI / hits.cols;
    for (int i = 0; i < hits.cols; i++)
    {
        // if(i < 60 || i> 300) continue;
        float dis = hits(0, i);
        if (dis <= 0)
        {
            continue;
        }
        pcl::PointXYZ p;
        p.x = dis * std::cos(da * i);
        p.y = dis * std::sin(da * i);
        p.z = 0;
        ret->push_back(p);
    }
    return ret;
}

#define KITTI00
// #define KITTI05
// #define KITTI07
// #define KAIST01
// #define KAIST02
// #define ApolloMap
// #define ApolloTest

#if defined(KITTI00) || defined(KITTI05) || defined(KITTI07)
#define KITTI
#endif
#if defined(KAIST01) || defined(KAIST02)
#define MULRAN
#endif
#if defined(ApolloMap) || defined(ApolloTest)
#define APOLLO
#endif

cv::Mat3b loadImage(const std::string &baseFolder, const std::string &filename)
{
    std::string file = fs::path(filename).filename().generic_string();
    file = file.substr(0, file.size() - 3) + "png";
#ifdef KITTI00
    return cv::imread(baseFolder + "/image_2/" + file);
#endif
    return cv::Mat3b();
}

Eigen::Matrix4f makeTransform(const cv::Mat1f &rvec, const cv::Mat1f &tvec)
{
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    Eigen::Vector3f ervec(rvec(0, 0), rvec(1, 0), rvec(2, 0));
    result.block<3, 3>(0, 0) = Eigen::AngleAxisf(ervec.norm(), ervec.normalized()).matrix();
    result.block<3, 1>(0, 3) = Eigen::Vector3f{tvec(0, 0), tvec(1, 0), tvec(2, 0)};
    return result;
}

int main()
{
    // std::string baseFolder = "H:/数据集/kitti-07/07";
#ifdef KITTI00
    // std::string baseFolder = "/mnt/h/数据集/kitti_full/00";
    // std::string baseFolder = "/mnt/h/数据集/kitti";
    std::string baseFolder = "/media/azurity/数据/数据集/kitti";
#endif
#ifdef KITTI07
    std::string baseFolder = "/mnt/h/数据集/kitti-07/07";
#endif
#ifdef KAIST01
    std::string baseFolder = "/mnt/h/数据集/MulRan/KAIST01";
#endif
#ifdef KAIST02
    std::string baseFolder = "/mnt/h/数据集/MulRan/KAIST02";
#endif
#ifdef ApolloMap
    std::vector<std::string> baseFolder = {
        "/mnt/d/新数据集/ColumbiaPark/ColumbiaPark/Apollo-SourthBay/MapData/ColumbiaPark/2018-09-21/1",
        "/mnt/d/新数据集/ColumbiaPark/ColumbiaPark/Apollo-SourthBay/MapData/ColumbiaPark/2018-09-21/2",
        "/mnt/d/新数据集/ColumbiaPark/ColumbiaPark/Apollo-SourthBay/MapData/ColumbiaPark/2018-09-21/3",
        "/mnt/d/新数据集/ColumbiaPark/ColumbiaPark/Apollo-SourthBay/MapData/ColumbiaPark/2018-09-21/4",
    };
#endif
#ifdef ApolloTest
    std::vector<std::string> baseFolder = {
        "/mnt/d/新数据集/ColumbiaPark/ColumbiaPark/Apollo-SourthBay/TestData/ColumbiaPark/2018-10-11",
    };
#endif
#ifdef KITTI
    auto fullDesc = loadKITTIData(baseFolder);
#endif
#ifdef MULRAN
    auto fullDesc = loadMulRanData(baseFolder);
#endif
#ifdef APOLLO
    auto tempfullDesc = loadAplloDataAll(baseFolder);
    std::vector<InfoDesc> fullDesc;
    std::copy(tempfullDesc.begin() + 100, tempfullDesc.end(), std::back_inserter(fullDesc));
#endif
    init();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr poses(new pcl::PointCloud<pcl::PointXYZ>);
    // for (const auto & it : fullDesc) {
    //     pcl::PointXYZ p;
    //     p.getArray3fMap() = it.pose.translation();
    //     poses->push_back(p);
    // }
    // pcl::io::savePCDFileBinary("test.pcd", *poses);
    // return 0;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr pc1(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile("cloud.pcd", *pc1);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pc2(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile(fullDesc[0].filename, *pc2);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pct(new pcl::PointCloud<pcl::PointXYZI>);
    // for (const auto & p : *pc1)
    // {
    //     pcl::PointXYZI np;
    //     np.getArray3fMap() = p.getArray3fMap();
    //     np.intensity = 0;
    //     pct->push_back(np);
    // }
    // for (const auto & p : *pc2)
    // {
    //     pcl::PointXYZI np;
    //     np.getArray3fMap() = fullDesc[0].pose * p.getArray3fMap();
    //     np.intensity = 1;
    //     pct->push_back(np);
    // }
    // pcl::io::savePCDFileBinary("total.pcd", *pct);
    // return 0;

#ifdef MAP_ITER
    std::vector<InfoDesc> subDesc;
    // for (int i = 0; i < fullDesc.size(); i += 5)
    for (int i = 0; i < fullDesc.size(); i += 5)
    {
        subDesc.push_back(fullDesc[i]);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr curbPC(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<size_t> need_update_submaps;
    // for (int index = 0; index < subDesc.size() && index < 64; index++)
    for (int index = 56; index < subDesc.size() && index < 57; index++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        if (subDesc[index].type == 0)
            cloud = loadBinPC(subDesc[index].filename);
        else if (subDesc[index].type == 1)
        {
            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(subDesc[index].filename, *cloud);
        }
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        if (cloud->size() == 0)
            continue;

        size_t submap_index = index;
        auto detect_info = ground_detector->process(cloud);
        auto submap_info = std::make_shared<SubmapInfo>();
        submap_info->detect = detect_info;
        submap_info->has_old_pose = false;
        submap_info->inited = false;
        map_builder->submaps.emplace(submap_index, submap_info);

        auto &current = map_builder->submaps.at(submap_index);
        current->pose = subDesc[index].pose.cast<double>();
        current->pose = Eigen::Isometry3d::Identity();
        current->inited = true;

        need_update_submaps.push_back(submap_index);

        // auto localCurb = makeCurbPC(detect_info->map_info);

        // pcl::io::savePCDFileBinary(std::to_string(index) + "_c.pcd", *localCurb);

        // pcl::transformPointCloud(*localCurb, *localCurb, subDesc[index].pose);
        // *curbPC += *localCurb;

        std::cout << submap_index << std::endl;

        // if ((index & 0x0f) == 0x0f) {
        if (index > 47)
        {
            map_builder->draw_multi<8, 8>(need_update_submaps);
            auto cloud = chunk_map->generateObstacleCloudProb(); // generateObstacleCloud();
            // pcl::io::savePCDFileBinary("iter/" + std::to_string(index - 48) +".pcd", *cloud);
            pcl::io::savePCDFileBinary("special.pcd", *cloud);
            // auto cloud2 = chunk_map->generateObstacleCloud();
            // pcl::io::savePCDFileBinary("iterori/" + std::to_string(index - 48) +".pcd", *cloud2);
        }
    }
    return 0;

    // for(const auto &key : chunk_map->keys())
    // {
    //     const auto &chunk = chunk_map->at(key);
    //     for(const auto &l : chunk.getLayers())
    //     {
    //         for(int r = 0; r < l.occupancy.rows; r++)
    //             for(int c = 0; c < l.occupancy.cols; c++)
    //                 if(l.observe(r,c) == 255)
    //                 {
    //                     if(l.occupancy(r,c) < 124)
    //                     {
    //                         pcl::PointXYZ p;
    //                         p.x = key.x * chunk_map->chunkBase() + c * chunk_map->resolution();
    //                         p.y = key.y * chunk_map->chunkBase() + r * chunk_map->resolution();
    //                         p.z = l.elevation(r, c);
    //                         curbPC->push_back(p);
    //                     }
    //                 }
    //     }
    // }

    // map_builder->saveMap("map.chunkmap");

    // auto cloud = chunk_map->generateObstacleCloud();
    // pcl::io::savePCDFileBinary("cloud.pcd", *cloud);
    // pcl::io::savePCDFileBinary("curb.pcd", *curbPC);
#endif

#ifdef BUILD_MAP
    std::vector<InfoDesc> subDesc;
    for (int i = 0; i < fullDesc.size(); i += 5)
    // for (int i = 0; i < fullDesc.size(); i += 20)
    {
        subDesc.push_back(fullDesc[i]);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr curbPC(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<size_t> need_update_submaps;
    for (int index = 0; index < subDesc.size(); index++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        if (subDesc[index].type == 0)
            cloud = loadBinPC(subDesc[index].filename);
        else if (subDesc[index].type == 1)
        {
            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(subDesc[index].filename, *cloud);
        }
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        if (cloud->size() == 0)
            continue;

        size_t submap_index = index;
        auto detect_info = ground_detector->process(cloud);
        auto submap_info = std::make_shared<SubmapInfo>();
        submap_info->detect = detect_info;
        submap_info->has_old_pose = false;
        submap_info->inited = false;

        cv::Mat3b image = loadImage(baseFolder, subDesc[index].filename);
        submap_info->frame_kp = (*image_kp)(*cloud, image);

        map_builder->submaps.emplace(submap_index, submap_info);

        auto &current = map_builder->submaps.at(submap_index);
        current->pose = subDesc[index].pose.cast<double>();
        current->inited = true;

        need_update_submaps.push_back(submap_index);

        // auto localCurb = makeCurbPC(detect_info->map_info);

        // pcl::io::savePCDFileBinary(std::to_string(index) + "_c.pcd", *localCurb);

        // pcl::transformPointCloud(*localCurb, *localCurb, subDesc[index].pose);
        // *curbPC += *localCurb;

        std::cout << submap_index << std::endl;
    }
    map_builder->draw_multi<8, 8>(need_update_submaps);

    for (const auto &key : chunk_map->keys())
    {
        const auto &chunk = chunk_map->at(key);
        for (const auto &l : chunk.getLayers())
        {
            for (int r = 0; r < l.occupancy.rows; r++)
                for (int c = 0; c < l.occupancy.cols; c++)
                    if (l.observe(r, c) == 255)
                    {
                        if (l.occupancy(r, c) < 124)
                        {
                            pcl::PointXYZ p;
                            p.x = key.x * chunk_map->chunkBase() + c * chunk_map->resolution();
                            p.y = key.y * chunk_map->chunkBase() + r * chunk_map->resolution();
                            p.z = l.elevation(r, c);
                            curbPC->push_back(p);
                        }
                    }
        }
    }

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

#if 0
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(12, 20, 2));
    matcher.add(descs);
    matcher.train();

    pcl::PointCloud<pcl::PointXYZI>::Ptr matchPC(new pcl::PointCloud<pcl::PointXYZI>);

    std::vector<cv::Mat> query;
    for(const auto &kp : map_builder->submaps[0]->frame_kp)
    {
        pcl::PointXYZI p;
        Eigen::Isometry3f trans = map_builder->submaps[0]->pose.cast<float>();
        p.getVector3fMap() = trans * std::get<0>(kp).getVector3fMap();
        p.intensity = 0;
        matchPC->push_back(p);
        query.push_back(std::get<1>(kp));
    }
    // std::vector<std::vector<cv::DMatch>> matched;
    // matcher.knnMatch(query, matched, 1);
    // center=avg(global-local)
    for (const auto &q : query)
    {
        std::vector<cv::DMatch> matched;
        matcher.match(q, matched);
        pcl::PointXYZI p;
        p.getVector3fMap() = kpPC->points[matched[0].imgIdx].getVector3fMap();
        p.z += 0.5;
        p.intensity = 1;
        matchPC->push_back(p);
    }
#endif

    map_builder->saveMap("map.chunkmap");

    // *kpPC += *curbPC;

    auto cloud = chunk_map->generateObstacleCloud();
    pcl::io::savePCDFileBinary("cloud.pcd", *cloud);
    pcl::io::savePCDFileBinary("curb.pcd", *curbPC);
    pcl::io::savePCDFileBinary("kp.pcd", *kpPC);
    // pcl::io::savePCDFile("match.pcd", *matchPC);

    std::ofstream kpofs("keypoints.pack", std::ios::binary);
    saveKeyPoints(kpofs, keypoints);
    kpofs.close();

#elif defined(NEWL)
    std::ofstream logfile("log.txt");

    ImageKeyPoints::FrameKP keypoints;
    std::ifstream kpifs("keypoints.pack", std::ios::binary);
    loadKeyPoints(kpifs, keypoints);
    kpifs.close();
#if 1
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(12, 20, 2));
#if 1
    std::vector<int> localMIndices;
    {
        cv::Mat1b descMat(cv::Size(32, keypoints.size()));
        for (int i = 0; i < keypoints.size(); i++)
        {
            const auto &kp = keypoints[i];
            localMIndices.push_back(i);
            std::get<1>(kp).copyTo(descMat.row(i));
        }
        matcher.add(descMat);
    }
    matcher.train();
#endif
#else
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    {
        cv::Mat1b descMat(cv::Size(32, keypoints.size()));
        for (int i = 0; i < keypoints.size(); i++)
        {
            const auto &kp = keypoints[i];
            std::get<1>(kp).copyTo(descMat.row(i));
        }
        matcher.add(descMat);
    }
#endif
    pcl::PointCloud<pcl::PointXYZ>::Ptr odom(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr traj(new pcl::PointCloud<pcl::PointXYZ>);

    cv::Mat1f K = cv::Mat1f::zeros(3, 3);
    K(0, 0) = 7.188560000000e+02;
    K(1, 1) = 7.188560000000e+02;
    K(0, 2) = 6.071928000000e+02;
    K(1, 2) = 1.852157000000e+02;
    K(2, 2) = 1.0;

    Eigen::Matrix4f pose = fullDesc[0].pose.matrix();

    Eigen::Matrix4f cptrans;
    cptrans << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
        -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
        9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
        0, 0, 0, 1;

    for (int index = 0; index < fullDesc.size(); index++)
    {
        std::cout << index << std::endl;
        // auto cloud = loadBinPC(fullDesc[index].filename);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        if (fullDesc[index].type == 0)
            cloud = loadBinPC(fullDesc[index].filename);
        else if (fullDesc[index].type == 1)
        {
            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(fullDesc[index].filename, *cloud);
        }
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        if (cloud->size() == 0)
            continue;

        cv::Mat3b image = loadImage(baseFolder, fullDesc[index].filename);
        // auto frame_kp = (*image_kp)(*cloud, image);
        auto frame_kp = (*image_kp)(image);

        auto time_start = std::chrono::steady_clock::now();

#pragma region main_part

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
        // for (const auto &kp : frame_kp)
        auto frame_kp3d = image_kp->debug(*cloud, image);
        //
        std::cout << "match stage" << std::endl;
        for (int i = 0; i < frame_kp.size(); i++)
        {
#if 1
            if (std::get<0>(frame_kp3d[i]).x == INFINITY)
                continue;
#endif
            const auto &kp = frame_kp[i];
            std::vector<std::vector<cv::DMatch>> matched;
            matcher.knnMatch(std::get<1>(kp), matched, 1);
            for (const auto &m : matched[0])
            {

                if (m.distance > 32)
                    // if (m.distance > 24)
                    // if (m.distance > 28)
                    continue;
                // camP.push_back(std::get<0>(kp));
                // camDebugI.push_back(i);
                pcl::PointXYZ globalP = std::get<0>(keypoints[localMIndices[m.trainIdx]]);
                // objP.emplace_back(globalP.x, globalP.y, globalP.z);

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
            // camP.push_back(std::get<0>(kp));
            // pcl::PointXYZ globalP = std::get<0>(keypoints[matched[0].trainIdx]);
            // objP.emplace_back(globalP.x, globalP.y, globalP.z);
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
            continue;

        std::cout << "pnp stage" << std::endl;
        cv::Mat rvec, tvec;
        std::vector<int> inlier;
        if (!cv::solvePnPRansac(objP, camP, K, cv::Mat(), rvec, tvec, false, 100, 8.0 /*8.0*/, 0.9 /*0.9*/, inlier))
            continue;
        std::cout << "inlier: " << (double)inlier.size() / camP.size() << std::endl;
        // if (!cv::solvePnPGeneric(objP, camP, K, cv::Mat(), rvec, tvec, false, cv::SolvePnPMethod::SOLVEPNP_EPNP))
        //     continue;
        // if (!cv::solvePnP(objP, camP, K, cv::Mat(), rvec, tvec))
        //     continue;
        // Eigen::Matrix4f oldpose = pose;
        pose = makeTransform(rvec, tvec);
        // pose = cptrans.inverse() * pose;
        pose = pose.inverse() * cptrans;
        // if ((oldpose * pose.inverse()).block<3, 1>(0, 3).norm() > 16.0f)
        //     pose = oldpose;

#pragma endregion

        auto time_stop = std::chrono::steady_clock::now();

        std::cout << "judge stage" << std::endl;
        Eigen::Matrix4f delta = pose.inverse() * fullDesc[index].pose.matrix();
        // std::cout << delta << std::endl;
        Eigen::Vector3f dt = delta.block<3, 1>(0, 3);
        Eigen::Quaternionf dq = Eigen::Quaternionf(delta.block<3, 3>(0, 0));
        std::cout << "move error: " << dt.norm() << std::endl
                  << "rot error: " << Eigen::AngleAxisf(dq).angle() << std::endl
                  << "time cost: " << std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start).count() << std::endl;
        logfile << index << " " << dt.norm() << " " << Eigen::AngleAxisf(dq).angle() << " " << std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start).count() << std::endl;

        {
            pcl::PointXYZ p;
            p.getArray3fMap() = fullDesc[index].pose.translation();
            traj->push_back(p);
            p.getArray3fMap() = pose.block<3, 1>(0, 3);
            odom->push_back(p);
        }
        // break;
        // if(dt.norm() > 20) {
        //     pcl::io::savePCDFileBinary("current.pcd", *cloud);
        //     pcl::io::savePCDFileBinary("currentO.pcd", *localPC);
        //     // pcl::io::savePCDFileBinary("currentC.pcd", *localCurb);
        //     break;
        // }
    }
    pcl::io::savePCDFileBinary("traj.pcd", *traj);
    pcl::io::savePCDFileBinary("odm.pcd", *odom);
    logfile.close();

#elif defined(LOCALIZATION)
    LidarIris iris(4, 18, 1.6, 0.75, 2);

    std::ofstream logfile("log.txt");

    chunk_map->desc_type = ChunkMap::DescType::uncompressed;
    std::ifstream ifs("map.chunkmap", std::ios::binary);
    ChunkMap::load(ifs, chunk_map);
    ifs.close();
    pcl::PointCloud<pcl::PointXYZ>::Ptr traj(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr odom(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalPC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("cloud.pcd", *globalPC);
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCPC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("curb.pcd", *globalCPC);

    ImageKeyPoints::FrameKP keypoints;
    std::ifstream kpifs("keypoints.pack", std::ios::binary);
    loadKeyPoints(kpifs, keypoints);
    kpifs.close();
#if 1
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(12, 20, 2));
    {
        // std::vector<cv::Mat> descs;
        // for (const auto &kp : keypoints)
        // {
        //     descs.push_back(std::get<1>(kp));
        // }
        // matcher.add(descMat);
        cv::Mat1b descMat(cv::Size(32, keypoints.size()));
        for (int i = 0; i < keypoints.size(); i++)
        {
            const auto &kp = keypoints[i];
            std::get<1>(kp).copyTo(descMat.row(i));
        }
        matcher.add(descMat);
    }
    matcher.train();
#else
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    {
        cv::Mat1b descMat(cv::Size(32, keypoints.size()));
        for (int i = 0; i < keypoints.size(); i++)
        {
            const auto &kp = keypoints[i];
            std::get<1>(kp).copyTo(descMat.row(i));
        }
        matcher.add(descMat);
    }
#endif
    pcl::PointCloud<pcl::PointXYZ>::Ptr kpodom(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    auto corrEst = new chunkmap::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, float>();
    icp.correspondence_estimation_.reset(corrEst);
    // specICP::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(4.0f);
    icp.setMaximumIterations(30);
    icp.setEuclideanFitnessEpsilon(0.001);
    icp.setInputTarget(globalPC);

#if 0
    specICP::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> cicp;
    cicp.setMaxCorrespondenceDistance(4.0f);
    cicp.setMaximumIterations(30);
    cicp.setEuclideanFitnessEpsilon(0.001);
    cicp.setInputTarget(globalCPC);
#endif

#if 0
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> licp;
    licp.setMaxCorrespondenceDistance(2.0f);
    licp.setMaximumIterations(30);
    licp.setEuclideanFitnessEpsilon(0.001);
#endif

    Eigen::Matrix4f pose = fullDesc[0].pose.matrix(); // Eigen::Matrix4f::Identity();
    bool inited = false;
    for (int index = 0; index < fullDesc.size(); index++)
    {
        std::cout << index << std::endl;
        // auto cloud = loadBinPC(fullDesc[index].filename);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        if (fullDesc[index].type == 0)
            cloud = loadBinPC(fullDesc[index].filename);
        else if (fullDesc[index].type == 1)
        {
            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(fullDesc[index].filename, *cloud);
        }
        // pcl::io::savePCDFileBinary("current.pcd", *cloud);
        // if(index == 431) {
        //     break;
        // }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        if (cloud->size() == 0)
            continue;

        cv::Mat3b image = loadImage(baseFolder, fullDesc[index].filename);
        auto frame_kp = (*image_kp)(*cloud, image);

        Eigen::Vector3f kp_center{0, 0, 0};
        float N = 0;
        for (const auto &kp : frame_kp)
        {
            std::vector<cv::DMatch> matched;
            matcher.match(std::get<1>(kp), matched);
            if (matched[0].distance >= 6)
                continue;
            float k = 1.0;
            N += k;
            kp_center += (std::get<0>(keypoints[matched[0].trainIdx]).getVector3fMap() - std::get<0>(kp).getVector3fMap()) * k;
        }
        if (N > 0)
        {
            kp_center /= N; // frame_kp.size();
            pcl::PointXYZ kpCP;
            kpCP.getVector3fMap() = kp_center;
            kpodom->push_back(kpCP);
        }

        auto time_start = std::chrono::steady_clock::now();

        Eigen::Vector3f limit = fullDesc[index].pose.translation();
        pcl::PointCloud<pcl::PointXYZ>::Ptr tglobalPC(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &p : *globalPC)
        {
            if (p.x < limit.x() - 100 || p.x > limit.x() + 100)
                continue;
            if (p.y < limit.y() - 100 || p.y > limit.y() + 100)
                continue;
            tglobalPC->push_back(p);
        }
        icp.setInputTarget(tglobalPC);

        auto time_start1 = std::chrono::steady_clock::now();

        size_t submap_index = index;
        auto detect_info = ground_detector->process(cloud);
        auto localFeature = featureDecompress(detect_info->feature);
        auto localPC = generateLocalCloud(chunk_map->obstacle_segments, chunk_map->obstacle_bits, detect_info->obstacle_info, chunk_map->resolution(), submap_radius);

        // pcl::io::savePCDFileBinary("src.pcd", *localPC);
        // std::cout << fullDesc[index].pose.matrix() << std::endl;
        // return 0;

        auto time_mid = std::chrono::steady_clock::now();
#if 0
        auto localCurb = makeCurbPC(detect_info->map_info);
#endif
        //
#if 0
        static auto matcher = [&](const LidarIris::FeatureDesc *feature1, const LidarIris::FeatureDesc *feature2, int *bias, float *score)
        {
            *score = iris.Compare(*feature1, *feature2, bias);
        };
        std::vector<int> nlist;
        for (int n = 0; n < chunk_map->key_frames.size(); n++)
        {
            const auto &kf = chunk_map->key_frames[n];
            // if (inited)
            {
                float distance = (kf.pose.block<3, 1>(0, 3) - pose.block<3, 1>(0, 3)).norm();
                if (distance > 15.0)
                // if (distance > 3.0)
                    continue;
                float angle = std::abs(Eigen::AngleAxisf(kf.pose.block<3,3>(0,0).inverse() * pose.block<3,3>(0,0)).angle());
                if (angle * M_PI / 360.0 > 10)
                    continue;
            }
            nlist.push_back(n);
        }
#endif
#if 0
        if(nlist.size()  != 0)
        {
            inited = true;
            std::vector<std::function<void()>> tasks;
            std::vector<int> biases(nlist.size());
            std::vector<float> scores(nlist.size());
            for (int n = 0; n < nlist.size(); n++)
            {
                const auto &kf = chunk_map->key_frames[nlist[n]];
                biases[n] = 0;
                scores[n] = 0.5;
                tasks.push_back(std::bind(matcher, &std::get<0>(kf.feature), &localFeature, &biases[n], &scores[n]));
            }
            multi_execute<8>(tasks);
            int min_n = 0;
            float min_score = 0.5;
            for (int n = 0; n < nlist.size(); n++)
            {
                if (scores[n] < min_score)
                {
                    min_score = scores[n];
                    min_n = n;
                }
            }
            //
            Eigen::AngleAxisf biasMat = Eigen::AngleAxisf(biases[min_n] * M_PI / 180.0, Eigen::Vector3f::UnitZ());
            Eigen::Matrix4f localTrans = Eigen::Matrix4f::Identity();
            localTrans.block<3, 3>(0, 0) = biasMat.toRotationMatrix();
            pose = chunk_map->key_frames[nlist[min_n]].pose * localTrans;
        }
#endif
#if 0
        if(inited)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
            licp.setInputSource(localPC);
            licp.align(*temp, Eigen::Matrix4f::Identity());
            pose = pose * icp.getFinalTransformation();
            inited = true;
        }
        licp.setInputTarget(localPC);
#endif

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);

#if 0
        cicp.setInputSource(localCurb);
        cicp.align(*temp, pose);
        pose = cicp.getFinalTransformation();
#endif
        corrEst->__counter = 0;

        icp.setInputSource(localPC);
        // icp.setInputSource(cloud);
        // icp.setInputSource(localCurb);
        icp.align(*temp, pose);
        pose = icp.getFinalTransformation();

        auto time_stop = std::chrono::steady_clock::now();

        // std::cout << "GT pose:" << fullDesc[index].pose.matrix() << std::endl;

        Eigen::Matrix4f delta = pose.inverse() * fullDesc[index].pose.matrix();
        // std::cout << delta << std::endl;
        Eigen::Vector3f dt = delta.block<3, 1>(0, 3);
        Eigen::Quaternionf dq = Eigen::Quaternionf(delta.block<3, 3>(0, 0));
        std::cout << "move error: " << dt.norm() << std::endl
                  << "rot error: " << Eigen::AngleAxisf(dq).angle() << std::endl
                  << "time cost: " << std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start).count() << std::endl
                  << "step n: " << corrEst->__counter << std::endl;
        logfile << index << " " << dt.norm() << " " << Eigen::AngleAxisf(dq).angle() << " " << std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start).count() << " " << corrEst->__counter << std::endl;

        {
            pcl::PointXYZ p;
            p.getArray3fMap() = fullDesc[index].pose.translation();
            traj->push_back(p);
            p.getArray3fMap() = pose.block<3, 1>(0, 3);
            odom->push_back(p);
        }
        // break;
        // if(dt.norm() > 20) {
        //     pcl::io::savePCDFileBinary("current.pcd", *cloud);
        //     pcl::io::savePCDFileBinary("currentO.pcd", *localPC);
        //     // pcl::io::savePCDFileBinary("currentC.pcd", *localCurb);
        //     break;
        // }
    }
    pcl::io::savePCDFileBinary("traj.pcd", *traj);
    pcl::io::savePCDFileBinary("odm.pcd", *odom);
    pcl::io::savePCDFileBinary("kpodom.pcd", *kpodom);
    logfile.close();
#endif
    return 0;
}
