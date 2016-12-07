#include "Shared.hpp"
#define Type typename
#include "matrix.h"
#include "interface.h"
#include "window.h"
#include "view-widget.h"
#include "variant.h"
#include "simd.h"
#include "parallel.h"

inline Variant parseJSON(TextData& s) {
    s.whileAny(" \t\n\r"_);
    if(s.match("true")) return true;
    else if(s.match("false")) return false;
    else if(s.match('"')) {
        return copyRef(s.until('"'));
    }
    else if(s.match('{')) {
        Dict dict;
        s.whileAny(" \t\n\r"_);
        if(!s.match('}')) for(;;) {
            s.skip('"');
            string key = s.until('"');
            assert_(key && !dict.contains(key));
            s.whileAny(" \t\n\r"_);
            s.skip(':');
            Variant value = parseJSON(s);
            dict.insertSorted(copyRef(key), ::move(value));
            s.whileAny(" \t\n\r"_);
            if(s.match(',')) { s.whileAny(" \t\n\r"_); continue; }
            if(s.match('}')) break;
            error("Expected , or }"_);
        }
        return dict;
    }
    else if(s.match('[')) {
        array<Variant> list;
        s.whileAny(" \t\n\r"_);
        if(!s.match(']')) for(;;) {
            Variant value = parseJSON(s);
            list.append( ::move(value) );
            s.whileAny(" \t\n\r"_);
            if(s.match(',')) continue;
            if(s.match(']')) break;
            error("Expected , or ]"_);
        }
        return list;
    }
    else {
        string d = s.whileDecimal();
        if(d) return parseDecimal(d);
        else error("Unexpected"_, s.peek(16));
    }
}

const vec2 Vec2(ref<Variant> v) { return vec2((float)v[0],(float)v[1]); }
const vec3 Vec3(ref<Variant> v) { return vec3((float)v[0],(float)v[1],(float)v[2]); }

const mat4 transform(const Dict& object) {
    const Dict& t = object.at("transform");
    mat4 transform;
    const vec3 look_at = Vec3(t.at("look_at"));
    const vec3 position = Vec3(t.at("position"));
    const vec3 z = normalize(look_at - position);
    vec3 y = Vec3(t.at("up"));
    y = normalize(y - dot(y,z)*z); // Projects up on plane orthogonal to z
    const vec3 x = cross(y, z);
    transform[0] = vec4(x, 0);
    transform[1] = vec4(y, 0);
    transform[2] = vec4(z, 0);
    transform[3] = vec4(-(position+16.f*z), 1);
    return transform;
}

mat4 parseCamera(ref<byte> file) {
    TextData s (file);
    Variant root = parseJSON(s);
    const Dict& camera = root.dict.at("camera");
    mat4 modelView = ::transform( camera ).inverse();
    modelView.rotateZ(PI); // -Z (FIXME)
    modelView = mat4().rotateZ(PI) * modelView;
    return modelView;
}

static mat4 shearedPerspective(const float s, const float t) { // Sheared perspective (rectification)
    const float S = 2*s-1, T = 2*t-1; // [0,1] -> [-1, 1]
    const float left = (-1-S), right = (1-S);
    const float bottom = (-1-T), top = (1-T);
    mat4 M;
    M(0,0) = 2 / (right-left);
    M(1,1) = 2 / (top-bottom);
    M(0,2) = (right+left) / (right-left);
    M(1,2) = (top+bottom) / (top-bottom);
    const float near = 1, far = 16;
    M(2,2) = - (far+near) / (far-near);
    M(2,3) = - 2*far*near / (far-near);
    M(3,2) = - 1;
    M(3,3) = 0;
    //M.translate(vec3(-10*S,-10*T,0));
    M.translate(vec3(-S,-T,0));
    M.translate(vec3(0,0,-1)); // 0 -> -1 (Z-)
    return M;
}

static Folder tmp {"/var/tmp/light",currentWorkingDirectory(), true};

struct Render {
    Render() {
        Folder cacheFolder {"teapot", tmp, true};
        for(string file: cacheFolder.list(Files)) remove(file, cacheFolder);

        const int N = 5;
        uint2 size (1024, 1024);

        File file(str(N)+'x'+str(N)+'x'+strx(size), cacheFolder, Flags(ReadWrite|Create));
        size_t byteSize = 4ull*N*N*size.y*size.x*sizeof(half);
        assert_(byteSize <= 16ull*1024*1024*1024);
        file.resize(byteSize);
        Map map (file, Map::Prot(Map::Read|Map::Write));
        mref<half> field = mcast<half>(map);
        assert_(field.size == 4ull*N*N*size.y*size.x);
        //field.clear(); // Explicitly clears to avoid performance skew from clear on page faults (and forces memory allocation)

        mat4 camera = parseCamera(readFile(arguments()[0]));

        Tungsten::EmbreeUtil::initDevice();
        Tungsten::ThreadUtils::startThreads(8);
        std::unique_ptr<Tungsten::Scene> _scene;
        _scene.reset(Tungsten::Scene::load(Tungsten::Path("scene.json")));
        _scene->loadResources();
        _scene->rendererSettings().setSpp(1);

        Time time (true); Time lastReport (true);
        //parallel_for(0, N*N, [&](uint unused threadID, size_t stIndex) {
        for(int stIndex: range(N*N)) {
            Time time {true};

            int sIndex = stIndex%N, tIndex = stIndex/N;
            if(lastReport.seconds()>1) { log(strD(stIndex,N*N)); lastReport.reset(); }

            //ImageH Z (unsafeRef(field.slice(((0ull*N+tIndex)*N+sIndex)*size.y*size.x, size.y*size.x)), size);
            ImageH B (unsafeRef(field.slice(((1ull*N+tIndex)*N+sIndex)*size.y*size.x, size.y*size.x)), size);
            ImageH G (unsafeRef(field.slice(((2ull*N+tIndex)*N+sIndex)*size.y*size.x, size.y*size.x)), size);
            ImageH R (unsafeRef(field.slice(((3ull*N+tIndex)*N+sIndex)*size.y*size.x, size.y*size.x)), size);

            // Sheared perspective (rectification)
            const float s = sIndex/float(N-1), t = tIndex/float(N-1);
            //::render(size, , B, G, R);
            _scene->camera()->M = shearedPerspective(s, t) * camera;

            using namespace Tungsten;
            std::unique_ptr<Tungsten::TraceableScene> _flattenedScene;
            _flattenedScene.reset(_scene->makeTraceable(readCycleCounter()));
            Tungsten::Integrator& integrator = _flattenedScene->integrator();
            Time renderTime {true};
            integrator.startRender([](){});
            integrator.waitForCompletion();
            Tungsten::Vec2u res = _scene->camera()->resolution();
            for(uint32 i: range(B.ref::size)) { // FIXME: SIMD
                Tungsten::Vec3f bgr = _scene->camera()->_colorBuffer->_bufferA[i];
                B[i] = bgr[2];
                G[i] = bgr[1];
                R[i] = bgr[0];
            }
            assert_(renderTime.nanoseconds()*8 > time.nanoseconds()*7, strD(renderTime, time));
            log(time);
        }
        log("Rendered",strx(uint2(N)),"x",strx(size),"images in", time);
    }
}; //prerender;

struct ViewControl : virtual Widget {
    vec2 viewYawPitch = vec2(0, 0); // Current view angles

    struct {
        vec2 cursor;
        vec2 viewYawPitch;
    } dragStart {0, 0};

    // Orbital ("turntable") view control
    virtual bool mouseEvent(vec2 cursor, vec2 size, Event event, Button button, Widget*&) override {
        if(event == Press) {
            dragStart = {cursor, viewYawPitch};
            return true;
        }
        if(event==Motion && button==LeftButton) {
            viewYawPitch = dragStart.viewYawPitch + float(2*PI) * (cursor - dragStart.cursor) / size;
            viewYawPitch.x = clamp<float>(-PI/3, viewYawPitch.x, PI/3);
            viewYawPitch.y = clamp<float>(-PI/3, viewYawPitch.y, PI/3);
        }
        else return false;
        return true;
    }
};

struct ViewApp {
    string name;
    vec2 min, max;
    uint2 imageCount;
    uint2 imageSize;
    Map map;
    ref<half> field;

    bool orthographic = false;

    ViewWidget view {uint2(1024,1024), {this, &ViewApp::render}};
    unique<Window> window = nullptr;

    ViewApp() {
        assert_(arguments());
        load(arguments()[0]);
        window = ::window(&view);
        window->setTitle(name);
        window->actions[Key('o')] = [this]{ orthographic=!orthographic; window->render(); };
    }
    void load(string name) {
        field = {};
        map = Map();
        imageCount = 0;
        imageSize = 0;
        this->name = name;
        Folder tmp (name, ::tmp, true);

        for(string name: tmp.list(Files)) {
            TextData s (name);
            imageCount.x = s.integer(false);
            if(!s.match('x')) continue;
            imageCount.y = s.integer(false);
            if(!s.match('x')) continue;
            imageSize.x = s.integer(false);
            if(!s.match('x')) continue;
            imageSize.y = s.integer(false);
            assert_(!s);
            map = Map(name, tmp);
            field = cast<half>(map);
            break;
        }
        assert_(imageCount && imageSize);

        if(window) {
            window->setSize();
            window->setTitle(name);
        }
    }
    Image render(uint2 targetSize, vec2 angles) {
        Image target (targetSize);

        const float s = (angles.x+PI/3)/(2*PI/3), t = (angles.y+PI/3)/(2*PI/3);
        mat4 M = shearedPerspective(s, t);

        assert_(imageCount.x == imageCount.y);

        parallel_chunk(target.size.y, [this, &target, M](uint, uint start, uint sizeI) {
            const uint targetStride = target.size.x;
            const uint size1 = imageSize.x *1;
            const uint size2 = imageSize.y *size1;
            const uint size3 = imageCount.x*size2;
            const uint64 size4 = uint64(imageCount.y)*uint64(size3);
            const struct Image4DH : ref<half> {
                uint4 size;
                Image4DH(uint2 imageCount, uint2 imageSize, ref<half> data) : ref<half>(data), size(imageCount.y, imageCount.x, imageSize.y, imageSize.x) {}
                const half& operator ()(uint s, uint t, uint u, uint v) const {
                    assert_(t < size[0] && s < size[1] && v < size[2] && u < size[3], int(s), int(t), int(u), int(v));
                    size_t index = ((uint64(t)*size[1]+s)*size[2]+v)*size[3]+u;
                    assert_(index < ref<half>::size, int(index), ref<half>::size, int(s), int(t), int(u), int(v), size);
                    return operator[](index);
                }
            } fieldB {imageCount, imageSize, field.slice(1*size4, size4)},
              fieldG {imageCount, imageSize, field.slice(2*size4, size4)},
              fieldR {imageCount, imageSize, field.slice(3*size4, size4)};
            assert_(imageSize.x%2==0); // Gather 32bit / half
            const v8ui sample4D = {    0,           size1/2,         size2/2,       (size2+size1)/2,
                                              size3/2, (size3+size1)/2, (size3+size2)/2, (size3+size2+size1)/2};
            for(int targetY: range(start, start+sizeI)) for(int targetX: range(target.size.x)) {
                size_t targetIndex = targetY*targetStride + targetX;
                const vec3 O = M.inverse() * vec3(2.f*targetX/float(targetStride-1)-1, 2.f*targetY/float(target.size.y-1)-1, -1);
                const vec3 P = M.inverse() * vec3(2.f*targetX/float(targetStride-1)-1, 2.f*targetY/float(target.size.y-1)-1, +1);
                const vec3 d = normalize(P-O);

                const vec3 n (0,0,1);
                const float nd = dot(n, d);
                const vec3 n_nd = n / nd;

                const vec2 Pst = O.xy() + dot(n_nd, vec3(0,0,1)-O) * d.xy();
                const vec2 ST = (Pst+vec2(1))/2.f;
                const vec2 Puv = O.xy() + dot(n_nd, vec3(0,0,0)-O) * d.xy();
                const vec2 UV = (Puv+vec2(1))/2.f;

                const vec2 st = vec2(0x1p-16) + vec2(1-0x1p-16) * ST * vec2(imageCount-uint2(1));
                const vec2 uv_uncorrected = vec2(1-0x1p-16) * UV * vec2(imageSize-uint2(1));

                if(st[0] < -0 || st[1] < -0) { target[targetIndex]=byte4(0xFF,0,0,0xFF); continue; }
                const uint sIndex = uint(st[0]), tIndex = uint(st[1]);
                if(sIndex >= uint(imageCount.x)-1 || tIndex >= uint(imageCount.y)-1) { target[targetIndex]=byte4(0,0xFF,0xFF,0xFF); continue; }

                bgr3f S = 0;
                {
                    const uint uIndex = uint(uv_uncorrected[0]), vIndex = uint(uv_uncorrected[1]);
                    if(uv_uncorrected[0] < 0 || uv_uncorrected[1] < 0) { target[targetIndex]=byte4(0,0,0xFF,0xFF); continue; }
                    if(uIndex >= uint(imageSize.x)-1 || vIndex >= uint(imageSize.y)-1) { target[targetIndex]=byte4(0xFF,0xFF,0,0xFF); continue; }
                    const size_t base = uint64(tIndex)*uint64(size3) + sIndex*size2 + vIndex*size1 + uIndex;
                    const v16sf B = toFloat(v16hf(gather(reinterpret_cast<const float*>(fieldB.data+base), sample4D)));
                    const v16sf G = toFloat(v16hf(gather(reinterpret_cast<const float*>(fieldG.data+base), sample4D)));
                    const v16sf R = toFloat(v16hf(gather(reinterpret_cast<const float*>(fieldR.data+base), sample4D)));

                    const v4sf x = {st[1], st[0], uv_uncorrected[1], uv_uncorrected[0]}; // tsvu
                    const v8sf X = __builtin_shufflevector(x, x, 0,1,2,3, 0,1,2,3);
                    static const v8sf _00001111f = {0,0,0,0,1,1,1,1};
                    const v8sf w_1mw = abs(X - floor(X) - _00001111f); // fract(x), 1-fract(x)
                    const v16sf w01 = shuffle(w_1mw, w_1mw, 4,4,4,4,4,4,4,4, 0,0,0,0,0,0,0,0)  // ttttttttTTTTTTTT
                            * shuffle(w_1mw, w_1mw, 5,5,5,5,1,1,1,1, 5,5,5,5,1,1,1,1)  // ssssSSSSssssSSSS
                            * shuffle(w_1mw, w_1mw, 6,6,2,2,6,6,2,2, 6,6,2,2,6,6,2,2)  // vvVVvvVVvvVVvvVV
                            * shuffle(w_1mw, w_1mw, 7,3,7,3,7,3,7,3, 7,3,7,3,7,3,7,3); // uUuUuUuUuUuUuUuU
                    S = bgr3f(dot(w01, B), dot(w01, G), dot(w01, R));
                }
                const uint b = ::min(0xFFFu, uint(0xFFF*S.b));
                const uint g = ::min(0xFFFu, uint(0xFFF*S.g));
                const uint r = ::min(0xFFFu, uint(0xFFF*S.r));
                extern uint8 sRGB_forward[0x1000];
                target[targetIndex] = byte4(sRGB_forward[b], sRGB_forward[g], sRGB_forward[r], 0xFF);
                //target[targetIndex] = byte4(byte3(float(0xFF)*S), 0xFF);
            }
        });
        return target;
    }
} view;
