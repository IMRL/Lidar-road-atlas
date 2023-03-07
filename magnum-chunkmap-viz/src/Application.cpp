#include <fstream>
#include <memory>
#include <mutex>
#include <thread>

#include <Corrade/Containers/Optional.h>
#include <Magnum/Math/Color.h>

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>

#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/ImGuiIntegration/Widgets.h>
#include <Magnum/ImGuiIntegration/Context.hpp>

#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>

#include "font.h"
#include "imgui-widgets/imfilebrowser.h"
#include "arc-ball/ArcBallCamera.h"
#include "map-object/MapObject.h"

static const std::vector<std::pair<std::string, int>> colormapEnum{
    {"GRAY", -1},
    {"AUTUMN", cv::COLORMAP_AUTUMN},
    {"BONE", cv::COLORMAP_BONE},
    {"JET", cv::COLORMAP_JET},
    {"WINTER", cv::COLORMAP_WINTER},
    {"RAINBOW", cv::COLORMAP_RAINBOW},
    {"OCEAN", cv::COLORMAP_OCEAN},
    {"SUMMER", cv::COLORMAP_SUMMER},
    {"SPRING", cv::COLORMAP_SPRING},
    {"COOL", cv::COLORMAP_COOL},
    {"HSV", cv::COLORMAP_HSV},
    {"PINK", cv::COLORMAP_PINK},
    {"HOT", cv::COLORMAP_HOT},
    {"PARULA", cv::COLORMAP_PARULA},
    {"MAGMA", cv::COLORMAP_MAGMA},
    {"INFERNO", cv::COLORMAP_INFERNO},
    {"PLASMA", cv::COLORMAP_PLASMA},
    {"VIRIDIS", cv::COLORMAP_VIRIDIS},
    {"CIVIDIS", cv::COLORMAP_CIVIDIS},
    {"TWILIGHT", cv::COLORMAP_TWILIGHT},
    {"TWILIGHT_SHIFTED", cv::COLORMAP_TWILIGHT_SHIFTED},
    {"TURBO", cv::COLORMAP_TURBO},
    {"DEEPGREEN", cv::COLORMAP_DEEPGREEN},
};

static const std::vector<std::pair<std::string, MapObject::ColorScheme>> schemeEnum{
    {"occupancy", MapObject::ColorScheme::OCCUPANCY},
    {"elevation", MapObject::ColorScheme::ELEVATION},
    {"log sigma", MapObject::ColorScheme::LOG_SIGMA},
};

using namespace Magnum;
using namespace Math::Literals;

using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;

class Application : public Platform::Application
{
public:
    explicit Application(const Arguments &arguments);

    void viewportEvent(ViewportEvent &event) override;

    void keyPressEvent(KeyEvent &event) override;
    void keyReleaseEvent(KeyEvent &event) override;

    void mousePressEvent(MouseEvent &event) override;
    void mouseReleaseEvent(MouseEvent &event) override;
    void mouseMoveEvent(MouseMoveEvent &event) override;
    void mouseScrollEvent(MouseScrollEvent &event) override;
    void textInputEvent(TextInputEvent &event) override;

private:
    void drawEvent() override;

    void drawUI();

    void loadMap(const std::string &file);

    bool _init = false;
    ImGuiIntegration::Context _imgui{NoCreate};
    ImFont *font;
    ImGui::FileBrowser fileDialog;
    std::string _statusText = "map not load";
    bool _displayMap = true;
    bool _displayObstacle = true;
    Color4 _clearColor = 0x808080ff_rgbaf;
    Color4 _obstacleColor = 0xff0000ff_rgbaf;
    int colormapIndex = 0;
    int schemeIndex = 0;
    uint8_t _pointSize = 1;
    float _pointBaseHeight = 0.0f;
    float _pointSegHeight = 1.0f;

    Scene3D _scene;
    Corrade::Containers::Optional<ArcBallCamera> _arcballCamera;

    ChunkMap::Ptr _map;
    std::shared_ptr<MapObject> _mapObject;
};

Application::Application(const Arguments &arguments) : Platform::Application{arguments, NoCreate}
{
    {
        Configuration conf;
        conf.setTitle("chunkmap viewer")
            .setWindowFlags(Configuration::WindowFlag::Resizable);
        GLConfiguration glConf;
        // glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if (!tryCreate(conf, glConf))
        {
            create(conf, glConf.setSampleCount(0));
        }
    }

    /* Set up the camera */
    {
        const Vector3 eye = Vector3::xAxis(10.0f);
        const Vector3 center{};
        const Vector3 up = Vector3::zAxis();
        _arcballCamera.emplace(_scene, eye, center, up, 45.0_degf,
                               windowSize(), framebufferSize());
    }

#pragma region init_imgui
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    {
        auto imFonts = ImGui::GetIO().Fonts;
        ImFontConfig fontConfig;
        fontConfig.FontDataOwnedByAtlas = false;
        font = imFonts->AddFontDefault(&fontConfig);
        fontConfig.MergeMode = true;
        imFonts->AddFontFromFileTTF(SYS_FONT "/" NOTO_CJK, 16, &fontConfig, imFonts->GetGlyphRangesChineseFull());
        imFonts->Build();
    }
    _imgui = ImGuiIntegration::Context(*ImGui::GetCurrentContext(),
                                       Vector2{windowSize()} / dpiScaling(),
                                       windowSize(), framebufferSize());

    GL::Renderer::setClearColor(_clearColor);

    fileDialog.SetTitle("Open Map...");
    fileDialog.SetTypeFilters({".chunkmap"});

    /* Set up proper blending to be used by ImGui. There's a great chance
       you'll need this exact behavior for the rest of your scene. If not, set
       this only for the drawFrame() call. */
    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add,
                                   GL::Renderer::BlendEquation::Add);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                   GL::Renderer::BlendFunction::OneMinusSourceAlpha);
#pragma endregion

#if !defined(MAGNUM_TARGET_WEBGL) && !defined(CORRADE_TARGET_ANDROID)
    /* Have some sane speed, please */
    // setMinimalLoopPeriod(33);
    setMinimalLoopPeriod(16);
#endif
}

void Application::drawEvent()
{
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

    _arcballCamera->update();

    if (_mapObject)
    {
        GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
        if (_displayMap)
            _arcballCamera->draw(_mapObject->piecesGroup);
        if (_displayObstacle)
            _arcballCamera->draw(_mapObject->obstacleGroup);
        GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    }

#pragma region imgui
    _imgui.newFrame();

    /* Enable text input, if needed */
    if (ImGui::GetIO().WantTextInput && !isTextInputActive())
        startTextInput();
    else if (!ImGui::GetIO().WantTextInput && isTextInputActive())
        stopTextInput();

    drawUI();

    /* Update application cursor */
    _imgui.updateApplicationCursor(*this);

    /* Set appropriate states. If you only draw ImGui, it is sufficient to
       just enable blending and scissor test in the constructor. */
    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);

    _imgui.drawFrame();

    /* Reset state. Only needed if you want to draw something else with
       different state after. */
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::disable(GL::Renderer::Feature::Blending);
#pragma endregion

    swapBuffers();
    redraw();
}

void Application::viewportEvent(ViewportEvent &event)
{
    GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});

    _arcballCamera->reshape(event.windowSize(), event.framebufferSize());

    _imgui.relayout(Vector2{event.windowSize()} / event.dpiScaling(),
                    event.windowSize(), event.framebufferSize());
}

void Application::keyPressEvent(KeyEvent &event)
{
    if (_imgui.handleKeyPressEvent(event))
        return;

    switch (event.key())
    {
    case KeyEvent::Key::L:
        if (_arcballCamera->lagging() > 0.0f)
        {
            Debug{} << "Lagging disabled";
            _arcballCamera->setLagging(0.0f);
        }
        else
        {
            Debug{} << "Lagging enabled";
            _arcballCamera->setLagging(0.85f);
        }
        break;
    case KeyEvent::Key::R:
        _arcballCamera->reset();
        break;

    default:
        return;
    }
    event.setAccepted();
}

void Application::keyReleaseEvent(KeyEvent &event)
{
    if (_imgui.handleKeyReleaseEvent(event))
        return;
}

void Application::mousePressEvent(MouseEvent &event)
{
    if (_imgui.handleMousePressEvent(event))
        return;

    _arcballCamera->initTransformation(event.position());
    event.setAccepted();
}

void Application::mouseReleaseEvent(MouseEvent &event)
{
    if (_imgui.handleMouseReleaseEvent(event))
        return;
}

void Application::mouseMoveEvent(MouseMoveEvent &event)
{
    if (_imgui.handleMouseMoveEvent(event))
        return;

    if (event.buttons())
    {
        // if (event.modifiers() & MouseMoveEvent::Modifier::Shift)
        if (event.buttons() & MouseMoveEvent::Button::Middle)
            _arcballCamera->translate(event.position());
        else
            _arcballCamera->rotate(event.position());
        event.setAccepted();
    }
}

void Application::mouseScrollEvent(MouseScrollEvent &event)
{
    if (_imgui.handleMouseScrollEvent(event))
    {
        /* Prevent scrolling the page */
        event.setAccepted();
        return;
    }

    const Float delta = event.offset().y();
    if (Math::abs(delta) > 1.0e-2f)
    {
        _arcballCamera->zoom(delta);
        event.setAccepted();
    }
}

void Application::textInputEvent(TextInputEvent &event)
{
    if (_imgui.handleTextInputEvent(event))
        return;
}

void Application::drawUI()
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::MenuItem("Open Map", "CTRL+O"))
        {
            fileDialog.Open();
        }
        ImGui::EndMainMenuBar();
    }
    if (!_init)
    {
        ImGui::SetNextWindowPos(ImVec2{16, 32});
        ImGui::SetNextWindowSize(ImVec2{200, 450});
    }
    if (ImGui::Begin("config & status"))
    {
        ImGui::Text("STATUS: %s", _statusText.c_str());
        ImGui::Text("FPS: %.1lf", double(ImGui::GetIO().Framerate));
        if (_mapObject)
        {
            if (ImGui::ColorEdit3("background color", _clearColor.data(), ImGuiColorEditFlags_NoInputs))
                GL::Renderer::setClearColor(_clearColor);
            if (ImGui::ColorEdit3("obstacle color", _obstacleColor.data(), ImGuiColorEditFlags_NoInputs))
                _mapObject->setPointColor(_obstacleColor);
            ImGui::Checkbox("display map", &_displayMap);
            ImGui::Checkbox("display obstacle", &_displayObstacle);
            ImGui::Text("colormap:");
            if (ImGui::Combo(
                    "###colormap", &colormapIndex, [](void *, int idx, const char **out)
                    { *out = colormapEnum[idx].first.c_str(); return true; },
                    (void *)&colormapEnum, colormapEnum.size()))
            {
                _mapObject->initColormap(colormapEnum[colormapIndex].second);
            }
            ImGui::Text("color scheme:");
            if (ImGui::Combo(
                    "###colorscheme", &schemeIndex, [](void *, int idx, const char **out)
                    { *out = schemeEnum[idx].first.c_str(); return true; },
                    (void *)&schemeEnum, schemeEnum.size()))
            {
                _mapObject->initColorScheme(schemeEnum[schemeIndex].second);
            }
            ImGuiIntegration::image(_mapObject->colormap, {180, 16});
            ImGui::Text("point size:");
            if (ImGui::InputScalar("###point size", ImGuiDataType_U8, &_pointSize))
            {
                if (_pointSize == 0)
                {
                    _pointSize = 1;
                }
                GL::Renderer::setPointSize(_pointSize);
            }
            if (ImGui::TreeNode("obstacle detail"))
            {
                ImGui::Text("base height:");
                ImGui::InputScalar("###base height", ImGuiDataType_Float, &_pointBaseHeight);
                ImGui::Text("segment height:");
                ImGui::InputScalar("###seg height", ImGuiDataType_Float, &_pointSegHeight);
                if (ImGui::Button("reload##point"))
                {
                    _mapObject->loadPoint(_pointBaseHeight, _pointSegHeight);
                }
                ImGui::TreePop();
            }
        }
    }
    ImGui::End();

    fileDialog.Display();

    if (fileDialog.HasSelected())
    {
        std::string file = fileDialog.GetSelected().string();
        fileDialog.ClearSelected();
        _statusText = "loading";
        loadMap(file);
    }
    _init = true;
}

void Application::loadMap(const std::string &file)
{
    _map.reset(new ChunkMap(ChunkMap::Config{}));
    std::ifstream ifs(file, std::ios::binary);
    ChunkMap::load(ifs, _map);
    ifs.close();
    auto mapObject = std::make_shared<MapObject>(&_scene, _map);

    mapObject->setPointColor(_obstacleColor);
    mapObject->initColorScheme(schemeEnum[schemeIndex].second);
    mapObject->loadPoint(_pointBaseHeight, _pointSegHeight);
    mapObject->initColormap(colormapEnum[colormapIndex].second);

    _mapObject = mapObject;
    _statusText = "loaded";
}

MAGNUM_APPLICATION_MAIN(Application)
