#include "data.h"
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

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


#ifdef KITTI00
std::string BASE_FOLDER = "/media/azurity/数据/数据集/kitti";
#endif

#ifdef KITTI
namespace internal
{
    std::any loadBinPC(const std::string &filename)
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

    std::any loadImage(const std::string &filename)
    {
        cv::Mat3b ret = cv::imread(filename);
        return ret;
    }
}

void initCLF()
{
    clf::GetterMap[clf::LiDAR_DATA_FLAG] = internal::loadBinPC;
    clf::GetterMap[clf::IMAGE_DATA_FLAG] = internal::loadImage;
}

clf::TypeDesc TYPE_DESC{clf::LiDAR_DATA_FLAG, clf::IMAGE_DATA_FLAG};

std::vector<clf::FrameDesc> listData(const std::string &baseFolder)
{
    Eigen::Matrix4f calib;
    calib << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03, -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01, 0, 0, 0, 1;
    std::vector<clf::FrameDesc> descs;
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
            .pose = poseCalib(calib, parsePose(line)).matrix(),
            .dataPath = {
                baseFolder + "/velodyne/" + nameBuffer + ".bin",
                baseFolder + "/image_2/" + nameBuffer + ".png",
            },
        });
    }
    return descs;
}
#endif
