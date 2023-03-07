#ifndef _MAP_OBJECT_H_
#define _MAP_OBJECT_H_

#include <Magnum/GL/Mesh.h>
#include <Magnum/Shaders/Flat.h>
#include "PieceObject.h"
#include "../chunkmap/chunk_map.h"

class ObstacleDrawable : public Magnum::SceneGraph::Drawable3D
{
public:
    explicit ObstacleDrawable(Object3D &object,
                              Magnum::Shaders::FlatGL3D &shader, Magnum::GL::Mesh &mesh,
                              Magnum::SceneGraph::DrawableGroup3D &drawables)
        : Magnum::SceneGraph::Drawable3D{object, &drawables}, _shader(shader),
          _mesh(mesh) {}

    void draw(const Magnum::Matrix4 &transformation, Magnum::SceneGraph::Camera3D &camera)
    {
        _shader
            .setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
            .draw(_mesh);
    }

private:
    Magnum::Shaders::FlatGL3D &_shader;
    Magnum::GL::Mesh &_mesh;
};

class MapObject
{
public:
    enum class ColorScheme
    {
        OCCUPANCY,
        ELEVATION,
        LOG_SIGMA
    };

    MapObject(Object3D *parent, ChunkMap::Ptr chunkmap);

    Magnum::SceneGraph::DrawableGroup3D piecesGroup;
    Magnum::SceneGraph::DrawableGroup3D obstacleGroup;

    void setPointColor(const Magnum::Color4 &color)
    {
        _obstacleShader.setColor(color);
    }

    void initColorScheme(ColorScheme scheme);
    void loadPoint(float baseHeight, float segHeight);
    void initColormap(int colormapType);

    Magnum::GL::Texture2D colormap;

private:
    Object3D _object;
    ChunkMap::Ptr _chunkmap;

    Magnum::GL::Mesh _pieceMesh;
    PieceShader _pieceShader;

    cv::Mat1b _originColorMat;

    std::vector<std::shared_ptr<PieceObject>> _pieces;

    Magnum::GL::Mesh _obstacleMesh;
    Magnum::Shaders::FlatGL3D _obstacleShader;
    Object3D _obstacleObject;
    ObstacleDrawable _obtacleDrawable;
};

#endif
