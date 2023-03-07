#ifndef _COMMON_LOCALIZATION_FRAMEWORK_
#define _COMMON_LOCALIZATION_FRAMEWORK_

#include <any>
#include <functional>

// #include <ranges>
#include <range/v3/range.hpp>
#include <range/v3/view.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Eigen>

#include <iostream>

namespace clf
{
    constexpr static int LiDAR_DATA_FLAG = 1;
    constexpr static int IMAGE_DATA_FLAG = 2;

    using DataGetter = std::function<std::any(const std::string &path)>;
    extern std::unordered_map<int, DataGetter> GetterMap;

    using TypeDesc = std::vector<int>;

    struct Frame
    {
        Eigen::Matrix4f pose;
        std::vector<std::any> data;
    };

    struct FrameDesc
    {
        Eigen::Matrix4f pose;
        std::vector<std::string> dataPath;
    };

    inline Frame loadData(const FrameDesc &desc, const TypeDesc &typeDesc)
    {
        Frame ret;
        ret.pose = desc.pose;
        ret.data.clear();
        for (int i = 0; i < desc.dataPath.size(); i++)
        {
            ret.data.push_back(GetterMap[typeDesc[i]](desc.dataPath[i]));
        }
        return ret;
    }

    struct Tester
    {
        using Localer = std::function<Eigen::Matrix4f(const Frame &data, const Eigen::Matrix4f &lastPose)>;
        using Guard = std::function<bool(const Eigen::Matrix4f &eval, const Eigen::Matrix4f &gt)>;
        std::vector<Eigen::Matrix4f> GTPose;
        std::vector<Eigen::Matrix4f> EvalPose;

        struct BlankGuard
        {
            bool operator()(const Eigen::Matrix4f &eval, const Eigen::Matrix4f &gt)
            {
                return true;
            }
        };

        template <typename R>
        static Tester test(const Localer &localer, R &&desc, const TypeDesc &typeDesc, const Guard &guard = BlankGuard())
        {
            static_assert(ranges::range<R>);
            Tester result;
            bool inited = false;
            Eigen::Matrix4f lastPose;
            int index = 0;
            for (const FrameDesc &item : desc)
            {
                std::cout << index << std::endl;
                index += 1;
                auto data = loadData(item, typeDesc);
                if (!inited)
                {
                    lastPose = item.pose;
                    inited = true;
                }
                auto pose = localer(data, lastPose);

                result.GTPose.push_back(data.pose);
                result.EvalPose.push_back(pose);

                if (!guard(pose, data.pose))
                {
                    break;
                }

                lastPose = pose;
            }
            return result;
        }
    };
}

#endif
