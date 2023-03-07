#include "../clf/clf.h"

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

extern std::string BASE_FOLDER;

void initCLF();
std::vector<clf::FrameDesc> listData(const std::string &baseFolder);
extern clf::TypeDesc TYPE_DESC;
