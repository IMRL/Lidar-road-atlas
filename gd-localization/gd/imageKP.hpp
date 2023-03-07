#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>

struct ImageKeyPoints
{
    using Ptr = std::shared_ptr<ImageKeyPoints>;
    static constexpr double radius = 5.0;
    static constexpr int MaxN = 64;

    using KP = std::tuple<pcl::PointXYZ, cv::Mat1b /*[1x32]*/>;
    using KP2D = std::tuple<cv::Point2f, cv::Mat1b /*[1x32]*/>;
    using FrameKP = std::vector<KP>;
    using FrameKP2D = std::vector<KP2D>;
    cv::Ptr<cv::ORB> orb;
    Eigen::Matrix4f transform;
    double fx;
    double fy;
    double cx;
    double cy;

    ImageKeyPoints(const Eigen::Matrix4f &transform, double fx, double fy, double cx, double cy)
        : transform(transform), fx(fx), fy(fy), cx(cx), cy(cy)
    {
        orb = cv::ORB::create(5000, 1.2, 8, 19, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    }

    FrameKP operator()(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat3b &image)
    {
#if 0
        std::vector<cv::KeyPoint> kps;
        cv::Mat1b feature;
        orb->detectAndCompute(image, cv::Mat(), kps, feature);
#else
        auto kps = getFeatures(image);
#endif

        pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXY>);
        pcl::PointCloud<pcl::PointXY>::Ptr kpc(new pcl::PointCloud<pcl::PointXY>);
        std::vector<std::vector<std::tuple<double, pcl::PointXYZ>>> kpV(kps.size());
        for (const auto &p : kps)
        {
            pcl::PointXY pp;
#if 0
            pp.x = p.pt.x;
            pp.y = p.pt.y;
#else
            pp.x = std::get<0>(p).x;
            pp.y = std::get<0>(p).y;
#endif
            kpc->push_back(pp);
        }
        kdtree->setInputCloud(kpc);

        std::vector<int> is(MaxN);
        std::vector<float> dis(MaxN);
        for (auto &p : cloud)
        {
            pcl::PointXYZ pp;
            pp.getArray4fMap() = transform * p.getArray4fMap().matrix();
            bool found = false;
            if (pp.z > 0)
            {
                float x = (pp.x * fx / pp.z) + cx;
                float y = (pp.y * fy / pp.z) + cy;
                pcl::PointXY p2d;
                p2d.x = x;
                p2d.y = y;
                int n = kdtree->radiusSearch(p2d, radius, is, dis, MaxN);
                // double d = pp.getArray3fMap().matrix().norm();
                double d = p.getArray3fMap().matrix().norm();
                for (int i=0;i<n;i++)
                {
                    kpV[is[i]].push_back(std::make_tuple(d, p));
                }
            }
        }

        FrameKP result;
        for (int i = 0; i < kpV.size(); i++)
        {
            const auto &v = kpV[i];
            if (v.size() > 0)
            {
                double min_d = std::get<0>(v[0]);
                pcl::PointXYZ min_p = std::get<1>(v[0]);
                for (const auto it : v)
                {
                    if (std::get<0>(it) < min_d)
                    {
                        min_d = std::get<0>(it);
                        min_p = std::get<1>(it);
                    }
                }
#if 0
                result.push_back(std::make_tuple(min_p, feature.row(i)));
#else
                result.push_back(std::make_tuple(min_p, std::get<1>(kps[i])));
#endif
            }
        }
        return result;
    }


    FrameKP2D operator()(const cv::Mat3b &image)
    {
#if 0
        FrameKP2D result;
        std::vector<cv::KeyPoint> kps;
        cv::Mat1b feature;
        orb->detectAndCompute(image, cv::Mat(), kps, feature);
        for(int i=0;i<kps.size();i++)
        {
            result.emplace_back(std::make_tuple(kps[i].pt, feature.row(i)));
        }
        return result;
#else
        return getFeatures(image);
#endif
    }

    FrameKP debug(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat3b &image)
    {
        auto kps = getFeatures(image);

        pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXY>);
        pcl::PointCloud<pcl::PointXY>::Ptr kpc(new pcl::PointCloud<pcl::PointXY>);
        std::vector<std::vector<std::tuple<double, pcl::PointXYZ>>> kpV(kps.size());
        for (const auto &p : kps)
        {
            pcl::PointXY pp;
            pp.x = std::get<0>(p).x;
            pp.y = std::get<0>(p).y;
            kpc->push_back(pp);
        }
        kdtree->setInputCloud(kpc);

        std::vector<int> is(MaxN);
        std::vector<float> dis(MaxN);
        for (auto &p : cloud)
        {
            pcl::PointXYZ pp;
            pp.getArray4fMap() = transform * p.getArray4fMap().matrix();
            bool found = false;
            if (pp.z > 0)
            {
                float x = (pp.x * fx / pp.z) + cx;
                float y = (pp.y * fy / pp.z) + cy;
                pcl::PointXY p2d;
                p2d.x = x;
                p2d.y = y;
                int n = kdtree->radiusSearch(p2d, radius, is, dis, MaxN);
                double d = pp.getArray3fMap().matrix().norm();
                for (int i=0;i<n;i++)
                {
                    kpV[is[i]].push_back(std::make_tuple(d, p));
                }
            }
        }

        FrameKP result;
        for (int i = 0; i < kpV.size(); i++)
        {
            const auto &v = kpV[i];
            if (v.size() > 0)
            {
                double min_d = std::get<0>(v[0]);
                pcl::PointXYZ min_p = std::get<1>(v[0]);
                for (const auto it : v)
                {
                    if (std::get<0>(it) < min_d)
                    {
                        min_d = std::get<0>(it);
                        min_p = std::get<1>(it);
                    }
                }
                result.push_back(std::make_tuple(min_p, std::get<1>(kps[i])));
            }
            else
            {
                pcl::PointXYZ invalid;
                invalid.x = INFINITY;
                result.push_back(std::make_tuple(invalid, std::get<1>(kps[i])));
            }
        }
        return result;
    }

private:
    FrameKP2D getFeatures(const cv::Mat3b &image);
};

void loadKeyPoints(std::istream &stream, ImageKeyPoints::FrameKP &keypoints);
void saveKeyPoints(std::ostream &stream, const ImageKeyPoints::FrameKP &keypoints);
