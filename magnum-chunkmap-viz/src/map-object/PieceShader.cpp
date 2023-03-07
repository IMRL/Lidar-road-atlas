#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Resource.h>

#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>

#include "PieceShader.h"

using namespace Magnum;

PieceShader::PieceShader()
{
    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);

    const Utility::Resource rs{"piece-shader"};

    GL::Shader vert{GL::Version::GL330, GL::Shader::Type::Vertex};
    GL::Shader frag{GL::Version::GL330, GL::Shader::Type::Fragment};

    vert.addSource(rs.getString("PieceShader.vert"));
    frag.addSource(rs.getString("PieceShader.frag"));

    CORRADE_INTERNAL_ASSERT_OUTPUT(GL::Shader::compile({vert, frag}));

    attachShaders({vert, frag});

    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    _transformationUniform = uniformLocation("transformationMatrix");
    _projectionUniform = uniformLocation("projectionMatrix");

    setUniform(uniformLocation("u_elevation"), ElevationTextureUnit);
    setUniform(uniformLocation("u_occupancy"), OccupancyTextureUnit);
    setUniform(uniformLocation("u_colormap"), ColormapTextureUnit);
}
