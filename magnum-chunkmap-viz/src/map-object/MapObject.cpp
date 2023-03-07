#include <Corrade/Containers/Array.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Mesh.h>
#include "MapObject.h"

using namespace Magnum;

struct Vertex
{
    Vector4 position;
    Vector2 textureCoordinates;
};

struct CloudVertex
{
    Vector3 position;
};

MapObject::MapObject(Object3D *parent, ChunkMap::Ptr chunkmap)
    : _object{parent}, _chunkmap(chunkmap), _obstacleObject{&_object}, _obtacleDrawable{_obstacleObject, _obstacleShader, _obstacleMesh, obstacleGroup}
{
    _originColorMat = cv::Mat1b::zeros(1, 256);
    for (int i = 0; i < 256; i++)
        _originColorMat(0, i) = i;
    colormap
        .setMinificationFilter(SamplerFilter::Nearest)
        .setMagnificationFilter(SamplerFilter::Nearest)
        .setWrapping(SamplerWrapping::ClampToEdge)
        .setMinLod(0)
        .setMaxLod(0)
        .setStorage(1, GL::TextureFormat::RGB8, {256, 1});
    _pieceShader.bindColormapTexture(colormap);

    uint32_t chunkSize = chunkmap->chunkSize();
    float chunkBase = chunkmap->chunkBase();
    float chunkUnit = chunkBase / float(chunkSize);
    float offset = chunkUnit / 2.0f;
    uint32_t texSize = uint32_t(std::pow(2, std::ceil(std::log2(float(chunkSize)))));
    float texUnit = 1.0f / float(texSize);
    float texOffset = texUnit / 2.0f;

    // step 1: create piece mesh
    std::vector<Vertex> vertices;
    std::vector<UnsignedInt> indices;

    int index = 0;
    for (uint32_t y = 0; y < chunkSize; y++)
        for (uint32_t x = 0; x < chunkSize; x++)
        {
            vertices.push_back({{x * chunkUnit - offset, y * chunkUnit - offset, 0.0f, 1.0f}, {x * texUnit + texOffset, y * texUnit + texOffset}});
            vertices.push_back({{x * chunkUnit + offset, y * chunkUnit - offset, 0.0f, 1.0f}, {x * texUnit + texOffset, y * texUnit + texOffset}});
            vertices.push_back({{x * chunkUnit - offset, y * chunkUnit + offset, 0.0f, 1.0f}, {x * texUnit + texOffset, y * texUnit + texOffset}});
            vertices.push_back({{x * chunkUnit + offset, y * chunkUnit + offset, 0.0f, 1.0f}, {x * texUnit + texOffset, y * texUnit + texOffset}});
            indices.push_back(index * 4 + 0);
            indices.push_back(index * 4 + 1);
            indices.push_back(index * 4 + 2);
            indices.push_back(index * 4 + 2);
            indices.push_back(index * 4 + 1);
            indices.push_back(index * 4 + 3);
            index += 1;
        }
    auto verticesView = Containers::arrayView(vertices.data(), vertices.size());
    auto indicesView = Containers::arrayView(indices.data(), indices.size());

    _pieceMesh.setCount(Containers::arraySize(indicesView))
        .addVertexBuffer(GL::Buffer{verticesView}, 0,
                         PieceShader::Position{},
                         PieceShader::TextureCoordinates{})
        .setIndexBuffer(GL::Buffer{indicesView}, 0,
                        GL::MeshIndexType::UnsignedInt);

    // step 2: create piece object
    const Vector2i imageSize{int(texSize), int(texSize)};
    for (const auto &key : chunkmap->keys())
    {
        const auto &layers = chunkmap->at(key).getLayers();
        // for (const auto &layer : layers)
        for (int l = 0; l < layers.size(); l++)
        {
            const auto &layer = layers[l];
            cv::Mat1f elevation = cv::Mat1f::zeros(texSize, texSize);
            layer.elevation.copyTo(elevation(cv::Rect(0, 0, chunkSize, chunkSize)));
            ImageView2D elevationView{PixelFormat::R32F, imageSize, Corrade::Containers::ArrayView<const void>(elevation.ptr(), imageSize.product() * sizeof(float))};

            _pieces.push_back(std::make_shared<PieceObject>(&_object, _pieceShader, _pieceMesh, piecesGroup, std::tuple<int, int, int>{key.x, key.y, l}));
            _pieces.back()->_object.translate({key.x * chunkBase, key.y * chunkBase, 0.0f});
            _pieces
                .back()
                ->_texElevation.setMinificationFilter(SamplerFilter::Nearest)
                .setMagnificationFilter(SamplerFilter::Nearest)
                .setWrapping(SamplerWrapping::ClampToEdge)
                .setMinLod(0)
                .setMaxLod(0)
                .setStorage(1, GL::TextureFormat::R32F, imageSize)
                .setSubImage(0, {}, elevationView);
            _pieces
                .back()
                ->_texOccupancy.setMinificationFilter(SamplerFilter::Nearest)
                .setMagnificationFilter(SamplerFilter::Nearest)
                .setWrapping(SamplerWrapping::ClampToEdge)
                .setMinLod(0)
                .setMaxLod(0)
                .setStorage(1, GL::TextureFormat::RG8, imageSize);
        }
    }
}

void MapObject::initColorScheme(ColorScheme scheme)
{
    uint32_t chunkSize = _chunkmap->chunkSize();
    uint32_t texSize = uint32_t(std::pow(2, std::ceil(std::log2(float(chunkSize)))));
    const Vector2i imageSize{int(texSize), int(texSize)};

    switch (scheme)
    {
    case ColorScheme::OCCUPANCY:
    {
        for (const auto &piece : _pieces)
        {
            const auto &layer = _chunkmap->at({std::get<0>(piece->_index), std::get<1>(piece->_index)}).getLayers().at(std::get<2>(piece->_index));

            cv::Mat2b colored = cv::Mat2b::zeros(texSize, texSize);
            cv::Mat1b occupancy = cv::Mat1b::zeros(layer.occupancy.size());
            layer.occupancy.convertTo(occupancy, CV_8U);
            cv::Mat2b merged;
            cv::merge(std::vector<cv::Mat1b>{occupancy, layer.observe}, merged);
            merged.copyTo(colored(cv::Rect(0, 0, chunkSize, chunkSize)));
            ImageView2D occupancyView{PixelFormat::RG8Unorm, imageSize, Corrade::Containers::ArrayView<const void>(colored.ptr(), imageSize.product() * 2)};
            piece->_texOccupancy.setSubImage(0, {}, occupancyView);
        }
    }
    break;
    case ColorScheme::ELEVATION:
    {
        double minE = std::numeric_limits<double>::max(), maxE = std::numeric_limits<double>::lowest();
        for (const auto &key : _chunkmap->keys())
        {
            const auto &layers = _chunkmap->at(key).getLayers();
            for (const auto &layer : layers)
            {
                double mini, maxi;
                cv::minMaxIdx(layer.elevation, &mini, &maxi, nullptr, nullptr, layer.observe != 0);
                minE = std::min(minE, mini);
                maxE = std::max(maxE, maxi);
            }
        }
        double alpha = 255.0 / (maxE - minE);
        double beta = (-minE) * alpha;
        for (const auto &piece : _pieces)
        {
            const auto &layer = _chunkmap->at({std::get<0>(piece->_index), std::get<1>(piece->_index)}).getLayers().at(std::get<2>(piece->_index));

            cv::Mat2b colored = cv::Mat2b::zeros(texSize, texSize);
            cv::Mat1b occupancy = cv::Mat1b::zeros(layer.occupancy.size());
            layer.elevation.convertTo(occupancy, CV_8U, alpha, beta);
            cv::Mat2b merged;
            cv::merge(std::vector<cv::Mat1b>{occupancy, layer.observe}, merged);
            merged.copyTo(colored(cv::Rect(0, 0, chunkSize, chunkSize)));
            ImageView2D occupancyView{PixelFormat::RG8Unorm, imageSize, Corrade::Containers::ArrayView<const void>(colored.ptr(), imageSize.product() * 2)};
            piece->_texOccupancy.setSubImage(0, {}, occupancyView);
        }
    }
    break;
    case ColorScheme::LOG_SIGMA:
    {
        double minE = std::numeric_limits<double>::max(), maxE = std::numeric_limits<double>::lowest();
        for (const auto &key : _chunkmap->keys())
        {
            const auto &layers = _chunkmap->at(key).getLayers();
            for (const auto &layer : layers)
            {
                double mini, maxi;
                cv::Mat1f logged;
                cv::log(layer.elevation_sigma, logged);
                cv::minMaxIdx(logged, &mini, &maxi, nullptr, nullptr, layer.observe != 0);
                minE = std::min(minE, mini);
                maxE = std::max(maxE, maxi);
            }
        }
        double alpha = 255.0 / (maxE - minE);
        double beta = (-minE) * alpha;
        for (const auto &piece : _pieces)
        {
            const auto &layer = _chunkmap->at({std::get<0>(piece->_index), std::get<1>(piece->_index)}).getLayers().at(std::get<2>(piece->_index));

            cv::Mat2b colored = cv::Mat2b::zeros(texSize, texSize);
            cv::Mat1b occupancy = cv::Mat1b::zeros(layer.occupancy.size());
            cv::Mat1f logged;
            cv::log(layer.elevation_sigma, logged);
            logged.convertTo(occupancy, CV_8U, alpha, beta);
            cv::Mat2b merged;
            cv::merge(std::vector<cv::Mat1b>{occupancy, layer.observe}, merged);
            merged.copyTo(colored(cv::Rect(0, 0, chunkSize, chunkSize)));
            ImageView2D occupancyView{PixelFormat::RG8Unorm, imageSize, Corrade::Containers::ArrayView<const void>(colored.ptr(), imageSize.product() * 2)};
            piece->_texOccupancy.setSubImage(0, {}, occupancyView);
        }
    }
    break;
    }
}

void MapObject::loadPoint(float baseHeight, float segHeight)
{
    auto cloud = _chunkmap->generateObstacleCloud(baseHeight, segHeight);
    std::vector<CloudVertex> cloudVertices;
    for (const auto &p : *cloud)
    {
        cloudVertices.push_back({{p.x, p.y, p.z}});
    }
    auto view = Containers::arrayView(cloudVertices.data(), cloudVertices.size());
    _obstacleMesh.setPrimitive(MeshPrimitive::Points)
        .setCount(Containers::arraySize(view))
        .addVertexBuffer(GL::Buffer{view}, 0, Shaders::FlatGL3D::Position{});
}

void MapObject::initColormap(int colormapType)
{
    cv::Mat3b color;
    if (colormapType < 0)
    {
        cv::cvtColor(_originColorMat, color, cv::COLOR_GRAY2BGR);
    }
    else
    {
        cv::applyColorMap(_originColorMat, color, colormapType);
    }
    const Vector2i size{256, 1};
    ImageView2D view{PixelFormat::RGB8Unorm, size, Corrade::Containers::ArrayView<const void>(color.ptr(), size.product() * 3)};
    colormap.setSubImage(0, {}, view);
}
