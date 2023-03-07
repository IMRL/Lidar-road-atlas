find_package(Magnum REQUIRED GL Sdl2Application)
find_package(MagnumIntegration REQUIRED ImGui)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(Threads)
find_package(OpenCV)
find_package(Eigen3)
find_package(PCL)

include_directories(${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

corrade_add_resource(MapShader_RESOURCES map-object/resources.conf)

file(GLOB arc_ball_source "arc-ball/*.cpp")

file(GLOB_RECURSE chunkmap_source "chunkmap/*.cpp")

file(GLOB_RECURSE map_source "map-object/*.cpp")

add_executable(ChunkmapViewer Application.cpp ${arc_ball_source} ${chunkmap_source} ${map_source} ${MapShader_RESOURCES})
target_link_libraries(ChunkmapViewer PRIVATE
    Magnum::Application
    Magnum::DebugTools
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders
    MagnumIntegration::ImGui
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    Threads::Threads)

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ChunkmapViewer)
