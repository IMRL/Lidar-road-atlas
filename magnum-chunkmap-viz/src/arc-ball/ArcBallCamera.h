#ifndef Magnum_Examples_ArcBallCamera_h
#define Magnum_Examples_ArcBallCamera_h
/*
    This file is part of Magnum.
    Original authors — credit is appreciated but not required:
        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
        2020, 2021, 2022 — Vladimír Vondruš <mosra@centrum.cz>
        2020 — Nghia Truong <nghiatruong.vn@gmail.com>
    This is free and unencumbered software released into the public domain.
    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.
    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/AbstractTranslationRotation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>

#include "ArcBall.h"

/* Arcball camera implementation integrated into the SceneGraph */
class ArcBallCamera: public ArcBall {
    public:
        template<class Transformation> ArcBallCamera(
            Magnum::SceneGraph::Scene<Transformation>& scene,
            const Magnum::Vector3& cameraPosition, const Magnum::Vector3& viewCenter,
            const Magnum::Vector3& upDir, Magnum::Deg fov, const Magnum::Vector2i& windowSize,
            const Magnum::Vector2i& viewportSize):
            ArcBall{cameraPosition, viewCenter, upDir, fov, windowSize}
        {
            /* Create a camera object of a concrete type */
            auto* cameraObject = new Magnum::SceneGraph::Object<Transformation>{&scene};
            (*(_camera = new Magnum::SceneGraph::Camera3D{*cameraObject}))
                .setAspectRatioPolicy(Magnum::SceneGraph::AspectRatioPolicy::Extend)
                .setProjectionMatrix(Magnum::Matrix4::perspectiveProjection(
                    fov, Magnum::Vector2{windowSize}.aspectRatio(), 0.01f, 10000.0f))
                .setViewport(viewportSize);

            /* Save the abstract transformation interface and initialize the
               camera position through that */
            (*(_cameraObject = cameraObject))
                .rotate(transformation().rotation())
                .translate(transformation().translation());
        }

        /* Update screen and viewport size after the window has been resized */
        void reshape(const Magnum::Vector2i& windowSize, const Magnum::Vector2i& viewportSize) {
            _windowSize = windowSize;
            _camera->setViewport(viewportSize);
        }

        /* Update the SceneGraph camera if arcball has been changed */
        bool update() {
            /* call the internal update */
            if(!updateTransformation()) return false;

            (*_cameraObject)
                .resetTransformation()
                .rotate(transformation().rotation())
                .translate(transformation().translation());
            return true;
        }

        /* Draw objects using the internal scenegraph camera */
        void draw(Magnum::SceneGraph::DrawableGroup3D& drawables) {
            _camera->draw(drawables);
        }

        /* Accessor to the raw camera object */
        Magnum::SceneGraph::Camera3D& camera() const { return *_camera; }

    private:
        Magnum::SceneGraph::AbstractTranslationRotation3D* _cameraObject{};
        Magnum::SceneGraph::Camera3D* _camera{};
};

#endif
