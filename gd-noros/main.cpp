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
// ImageKeyPoints::Ptr image_kp;

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
    // image_kp.reset(new ImageKeyPoints(trans, fx, fy, cx, cy));
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

#if 0
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
#endif

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

    std::vector<InfoDesc> subDesc;
    for (int i = 0; i < fullDesc.size(); i += 5)
    // for (int i = 0; i < fullDesc.size(); i += 20)
    {
        subDesc.push_back(fullDesc[i]);
    }

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

        // cv::Mat3b image = loadImage(baseFolder, subDesc[index].filename);
        // submap_info->frame_kp = (*image_kp)(*cloud, image);

        map_builder->submaps.emplace(submap_index, submap_info);

        auto &current = map_builder->submaps.at(submap_index);
        current->pose = subDesc[index].pose.cast<double>();
        current->inited = true;

        need_update_submaps.push_back(submap_index);

        std::cout << submap_index << std::endl;
    }
    map_builder->draw_multi<8, 8>(need_update_submaps);

    map_builder->saveMap("map.chunkmap");

    return 0;
}
