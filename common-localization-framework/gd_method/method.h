#include "../clf/clf.h"
#include <memory>

void init();

namespace internal
{
    struct Builder
    {
        std::vector<size_t> need_update_submaps;
        void iter(int index, const clf::Frame &data);
        void process();
    };
}

template <typename R>
void buildMap(R &&desc, const clf::TypeDesc &typeDesc)
{
    internal::Builder b;
    static_assert(ranges::range<R>);
    int index = 0;
    for (const clf::FrameDesc &item : desc)
    {

        auto data = loadData(item, typeDesc);
        b.iter(index, data);
        index += 1;
    }
    b.process();
}

struct ImageLocalizer
{
    using Ptr = std::shared_ptr<ImageLocalizer>;
    virtual Eigen::Matrix4f operator()(const clf::Frame &data, const Eigen::Matrix4f &lastPose) = 0;
    static Ptr create();
};

struct LidaLocalizer
{
    using Ptr = std::shared_ptr<LidaLocalizer>;
    virtual Eigen::Matrix4f operator()(const clf::Frame &data, const Eigen::Matrix4f &lastPose) = 0;
    static Ptr create();
};
