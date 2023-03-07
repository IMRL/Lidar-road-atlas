#include "imageKP.hpp"
#define MSGPACK_USE_DEFINE_MAP
#include <msgpack.hpp>

static int NNeed = 2048;
static int BinNeed = 512;

struct KPBin
{
    cv::Rect rect;
    std::vector<cv::KeyPoint> points;
};

ImageKeyPoints::FrameKP2D ImageKeyPoints::getFeatures(const cv::Mat3b &image)
{
    std::vector<cv::KeyPoint> kps;
    orb->detect(image, kps);
    std::vector<KPBin> bins;
    bins.push_back(KPBin{.rect = cv::Rect(0, 0, image.cols, image.rows)});
    bins[0].points.swap(kps);
    if (bins[0].points.size() > NNeed)
    {
        while (bins.size() < BinNeed)
        {
            std::vector<KPBin> newBins;
            for (const auto &ori : bins)
            {
                KPBin lt, rt, lb, rb;
                int left = ori.rect.x;
                int top = ori.rect.y;
                int w = ori.rect.width;
                int h = ori.rect.height;
                lt.rect = cv::Rect(left, top, w / 2, h / 2);
                rt.rect = cv::Rect(left + w / 2, top, w - w / 2, h / 2);
                lb.rect = cv::Rect(left, top + h / 2, w / 2, h - h / 2);
                rb.rect = cv::Rect(left + w / 2, top + h / 2, w - w / 2, h - h / 2);
                for (const auto &pt : ori.points)
                {
                    if (lt.rect.contains(pt.pt))
                    {
                        lt.points.push_back(pt);
                    }
                    else if (rt.rect.contains(pt.pt))
                    {
                        rt.points.push_back(pt);
                    }
                    else if (lb.rect.contains(pt.pt))
                    {
                        lb.points.push_back(pt);
                    }
                    else if (rb.rect.contains(pt.pt))
                    {
                        rb.points.push_back(pt);
                    }
                }
                if (lt.points.size() > 0)
                    newBins.push_back(lt);
                if (rt.points.size() > 0)
                    newBins.push_back(rt);
                if (lb.points.size() > 0)
                    newBins.push_back(lb);
                if (rb.points.size() > 0)
                    newBins.push_back(rb);
            }
            bins.swap(newBins);
        }
    }
    kps.clear();
    for (auto &bin : bins)
    {
        std::sort(bin.points.begin(), bin.points.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) -> bool
                  { return a.response > b.response; });
        for (int i = 0; i < 4 && i < bin.points.size(); i++)
            if (bin.points[i].octave >= 0)
                kps.push_back(bin.points[i]);
    }
    std::sort(kps.begin(), kps.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) -> bool
              { return a.response > b.response; });
    kps = std::vector<cv::KeyPoint>(kps.begin(), kps.begin() + std::min(NNeed, (int)kps.size()));
    // for (const auto &pt : bin.points)
    //     kps.push_back(pt);
    cv::Mat1b features;
    orb->compute(image, kps, features);
    FrameKP2D result;
    for (int i = 0; i < kps.size(); i++)
    {
        result.emplace_back(std::make_tuple(kps[i].pt, features.row(i)));
    }
    return result;
}

struct KeyPointHelper
{
    float x;
    float y;
    float z;
    std::vector<uint8_t> feature;
    MSGPACK_DEFINE(x, y, z, feature);
};

void loadKeyPoints(std::istream &stream, ImageKeyPoints::FrameKP &keypoints)
{
    std::vector<KeyPointHelper> result;
    {
        static const size_t try_read_size = 1024;
        msgpack::unpacker unp;
        while (true)
        {
            unp.reserve_buffer(try_read_size);
            size_t actual_read_size = stream.readsome(unp.buffer(), try_read_size);
            unp.buffer_consumed(actual_read_size);
            msgpack::object_handle handle;
            if (unp.next(handle))
            {
                result = msgpack::object(handle.get()).as<std::vector<KeyPointHelper>>();
                break;
            }
        }
    }
    for (const auto &item : result)
    {
        pcl::PointXYZ p;
        p.x = item.x;
        p.y = item.y;
        p.z = item.z;
        cv::Mat1b feature(cv::Size(32, 1));
        memcpy(feature.data, item.feature.data(), feature.total() * feature.elemSize());
        keypoints.emplace_back(std::make_tuple(p, feature));
    }
}

void saveKeyPoints(std::ostream &stream, const ImageKeyPoints::FrameKP &keypoints)
{
    std::vector<KeyPointHelper> result;
    for (const auto &item : keypoints)
    {
        KeyPointHelper kp;
        const auto &p = std::get<0>(item);
        const auto &feature = std::get<1>(item);
        kp.x = p.x;
        kp.y = p.y;
        kp.z = p.z;
        kp.feature.reserve(feature.total() * feature.elemSize());
        kp.feature.assign((uint8_t *)feature.data, (uint8_t *)feature.data + feature.total() * feature.elemSize());
        result.push_back(kp);
    }
    msgpack::pack(stream, result);
}
