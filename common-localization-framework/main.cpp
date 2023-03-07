#include "clf/clf.h"
#include "data/data.h"
#include "gd_method/method.h"
#include <fstream>
#include <fmt/format.h>

// #define BUILD_MAP
#define LOCALIZATION

void analyze(const clf::Tester &data)
{
    std::ofstream errorFile("error.log");
    std::ofstream errDetailFile("errDetail.log");
    for (int i = 0; i < data.GTPose.size(); i++)
    {
        Eigen::Matrix4f err = data.GTPose[i].inverse() * data.EvalPose[i];
        Eigen::Vector3f tERR = err.block<3, 1>(0, 3);
        Eigen::AngleAxisf qERR = Eigen::AngleAxisf(err.block<3, 3>(0, 0));
        errorFile << fmt::format("{:4f} {:4f}\n", tERR.norm(), qERR.angle());
        errDetailFile << fmt::format("{} {} {} {} {} {} {}\n", tERR.x(), tERR.y(), tERR.z(), qERR.angle(), qERR.axis().x(), qERR.axis().y(), qERR.axis().z());
    }
    errorFile.close();
    errDetailFile.close();
}

int main()
{
    initCLF();

    auto lists = listData(BASE_FOLDER);

#ifdef BUILD_MAP
    init();
    buildMap(ranges::view::iota(0, int(lists.size())) |
                 ranges::view::filter([](int i)
                                    { return i % 5 == 0; }) |
                 ranges::view::transform([&](int i) -> clf::FrameDesc
                                       { return lists[i]; }),
             TYPE_DESC);
#elif defined(LOCALIZATION)
    init();
    // auto localizer = LidaLocalizer::create();
    auto localizer = ImageLocalizer::create();
    auto result = clf::Tester::test([&](auto &a, const auto &b)
                                    { return (*localizer)(a, b); },
                                    ranges::view::all(lists) /*| ranges::view::take(100)*/, TYPE_DESC);
    analyze(result);
#endif

    return 0;
}