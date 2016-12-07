#include "Integrator.hpp"

#include "renderer/TraceableScene.hpp"

#include "cameras/Camera.hpp"

#include "math/BitManip.hpp"

#include "io/FileUtils.hpp"
#include "io/ImageIO.hpp"
#include "io/Scene.hpp"

#include "Timer.hpp"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <lodepng/lodepng.h>
#include <algorithm>

#include "image.h"
#include "interface.h"
#include "window.h"
extern Thread mainThread;
ImageF B, G, R;
Image image;

namespace Tungsten {

static Path incrementalFilename(const Path &dstFile, const std::string &suffix, bool overwrite)
{
    Path dstPath = (dstFile.stripExtension() + suffix) + dstFile.extension();
    if (overwrite)
        return std::move(dstPath);

    Path barePath = dstPath.stripExtension();
    Path extension = dstPath.extension();

    int index = 0;
    while (dstPath.exists())
        dstPath = (barePath + tfm::format("%03d", ++index)) + extension;

    return std::move(dstPath);
}

Integrator::Integrator()
: _scene(nullptr),
  _currentSpp(0),
  _nextSpp(0)
{
}

Integrator::~Integrator()
{
}

void Integrator::advanceSpp()
{
    _nextSpp = min(_currentSpp + _scene->rendererSettings().sppStep(), _scene->rendererSettings().spp());
}

void Integrator::writeBuffers(const std::string&, bool)
{
    Vec2u res = _scene->cam().resolution();
#if 1
    if(!B) B = ImageF(res.x(), res.y());
    if(!G) G = ImageF(res.x(), res.y());
    if(!R) R = ImageF(res.x(), res.y());
    assert_(!_scene->cam()._colorBuffer->_bufferB);
    for(uint32 i: range(B.ref::size)) {
        Vec3f bgr = _scene->cam()._colorBuffer->_bufferA[i];
        B[i] = bgr[2];
        G[i] = bgr[1];
        R[i] = bgr[0];
    }
#endif
#if 1
    std::unique_ptr<Vec3f[]> hdr(new Vec3f[res.product()]);

    for (uint32 y = 0; y < res.y(); ++y)
        for (uint32 x = 0; x < res.x(); ++x)
            hdr[x + y*res.x()] = _scene->cam().getLinear(x, y);

    image = Image(res.x(), res.y());
    for (uint32 i = 0; i < res.product(); ++i) {
        Vec3c v(clamp(Vec3i(_scene->cam().tonemap(hdr[i])*255.0f), Vec3i(0), Vec3i(255)));
        image[i] = byte4(v[2], v[1], v[0], 0xFF);
    }
#endif
}

void Integrator::saveOutputs()
{
    writeBuffers("", _scene->rendererSettings().overwriteOutputFiles());
}

void Integrator::saveCheckpoint()
{
    writeBuffers("_checkpoint", true);
}

// Computes a hash of everything in the scene except the renderer settings
// This is done by serializing everything to JSON and hashing the resulting string
static uint64 sceneHash(Scene &scene)
{
    rapidjson::Document document;
    document.SetObject();
    *(static_cast<rapidjson::Value *>(&document)) = scene.toJson(document.GetAllocator());
    document.RemoveMember("renderer");

    rapidjson::GenericStringBuffer<rapidjson::UTF8<>> buffer;
    rapidjson::Writer<rapidjson::GenericStringBuffer<rapidjson::UTF8<>>> jsonWriter(buffer);
    document.Accept(jsonWriter);

    return BitManip::hash(buffer.GetString());
}

void Integrator::saveRenderResumeData(Scene &scene)
{
    Path path = _scene->rendererSettings().resumeRenderFile();
    OutputStreamHandle out = FileUtils::openOutputStream(path);
    if (!out) {
        DBG("Failed to open render resume state at '%s'", path);
        return;
    }

    // TODO: Camera splat buffer not saved/reconstructed
    rapidjson::Document document;
    document.SetObject();
    document.AddMember("current_spp", _currentSpp, document.GetAllocator());
    document.AddMember("adaptive_sampling", _scene->rendererSettings().useAdaptiveSampling(), document.GetAllocator());
    document.AddMember("stratified_sampler", _scene->rendererSettings().useSobol(), document.GetAllocator());

    FileUtils::streamWrite(out, JsonUtils::jsonToString(document));
    uint64 jsonHash = sceneHash(scene);
    FileUtils::streamWrite(out, jsonHash);
    _scene->cam().serializeOutputBuffers(out);
    saveState(out);
}

bool Integrator::resumeRender(Scene &scene)
{
    InputStreamHandle in = FileUtils::openInputStream(_scene->rendererSettings().resumeRenderFile());
    if (!in)
        return false;

    std::string json = FileUtils::streamRead<std::string>(in);
    rapidjson::Document document;
    document.Parse<0>(json.c_str());
    if (document.HasParseError() || !document.IsObject())
        return false;

    bool adaptiveSampling, stratifiedSampler;
    if (!JsonUtils::fromJson(document, "adaptive_sampling", adaptiveSampling)
            || adaptiveSampling != _scene->rendererSettings().useAdaptiveSampling())
        return false;
    if (!JsonUtils::fromJson(document, "stratified_sampler", stratifiedSampler)
            || stratifiedSampler != _scene->rendererSettings().useSobol())
        return false;

    uint32 jsonSpp;
    if (!JsonUtils::fromJson(document, "current_spp", jsonSpp))
        return false;

    uint64 jsonHash;
    FileUtils::streamRead(in, jsonHash);
    if (jsonHash != sceneHash(scene))
        return false;

    _scene->cam().deserializeOutputBuffers(in);
    loadState(in);

    _currentSpp = jsonSpp;
    advanceSpp();

    return true;
}

}
