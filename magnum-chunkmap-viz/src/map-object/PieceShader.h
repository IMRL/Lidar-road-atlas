#ifndef _PIECE_SHADER_H_
#define _PIECE_SHADER_H_

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Shaders/Generic.h>

class PieceShader : public Magnum::GL::AbstractShaderProgram
{
public:
    typedef Magnum::GL::Attribute<0, Magnum::Vector4> Position;
    typedef Magnum::GL::Attribute<1, Magnum::Vector2> TextureCoordinates;

    explicit PieceShader();

    PieceShader &setTransformationMatrix(const Magnum::Matrix4 &transformation)
    {
        setUniform(_transformationUniform, transformation);
        return *this;
    }

    PieceShader &setProjectionMatrix(const Magnum::Matrix4 &projection)
    {
        setUniform(_projectionUniform, projection);
        return *this;
    }

    PieceShader &bindElevationTexture(Magnum::GL::Texture2D &texture)
    {
        texture.bind(ElevationTextureUnit);
        return *this;
    }

    PieceShader &bindOccupancyTexture(Magnum::GL::Texture2D &texture)
    {
        texture.bind(OccupancyTextureUnit);
        return *this;
    }

    PieceShader &bindColormapTexture(Magnum::GL::Texture2D &texture)
    {
        texture.bind(ColormapTextureUnit);
        return *this;
    }

private:
    enum : Magnum::Int
    {
        ElevationTextureUnit = 0,
        OccupancyTextureUnit = 1,
        ColormapTextureUnit = 2
    };

    Magnum::Int _transformationUniform;
    Magnum::Int _projectionUniform;
};

#endif
