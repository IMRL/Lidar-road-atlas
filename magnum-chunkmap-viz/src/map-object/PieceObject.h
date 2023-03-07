#ifndef _PIECE_OBJECT_H_
#define _PIECE_OBJECT_H_

#include <memory>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include "PieceShader.h"

using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;

class PieceObject
{
    class PieceDrawable : public Magnum::SceneGraph::Drawable3D
    {
    public:
        explicit PieceDrawable(Object3D &object,
                               PieceShader &shader, Magnum::GL::Mesh &mesh,
                               Magnum::SceneGraph::DrawableGroup3D &drawables,
                               Magnum::GL::Texture2D &elevation, Magnum::GL::Texture2D &occupancy)
            : Magnum::SceneGraph::Drawable3D{object, &drawables}, _shader(shader),
              _mesh(mesh), _elevation(elevation), _occupancy(occupancy) {}

        void draw(const Magnum::Matrix4 &transformation, Magnum::SceneGraph::Camera3D &camera)
        {
            _shader
                .setTransformationMatrix(transformation)
                .setProjectionMatrix(camera.projectionMatrix())
                .bindElevationTexture(_elevation)
                .bindOccupancyTexture(_occupancy)
                .draw(_mesh);
        }

    private:
        PieceShader &_shader;
        Magnum::GL::Mesh &_mesh;
        Magnum::GL::Texture2D &_elevation;
        Magnum::GL::Texture2D &_occupancy;
    };

public:
    explicit PieceObject(Object3D *parent,
                         PieceShader &shader, Magnum::GL::Mesh &mesh,
                         Magnum::SceneGraph::DrawableGroup3D &drawables, std::tuple<int,int,int> index)
        : _object{parent}, _drawable{_object, shader, mesh, drawables, _texElevation, _texOccupancy}, _index(index) {}

    Object3D _object;
    std::tuple<int,int,int> _index;
    Magnum::GL::Texture2D _texElevation;
    Magnum::GL::Texture2D _texOccupancy;

private:
    PieceDrawable _drawable;
};

#endif
