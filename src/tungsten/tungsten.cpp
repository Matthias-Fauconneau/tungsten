#include "Version.hpp"
#include "Shared.hpp"

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
    M.translate(vec3(-10*S,-10*T,0));
    M.translate(vec3(0,0,-1)); // 0 -> -1 (Z-)
    return M;
}

void render(uint2 unused targetSize, mat4 M, const ImageH& B, const ImageH& G, const ImageH& R) {
    Tungsten::CliParser parser("tungsten", "[options] scene1 [scene2 [scene3...]]");
    Tungsten::StandaloneRenderer renderer(parser, std::cout);
    parser._operands.push_back("scene.json");
    renderer.setup();
    renderer.renderScene();
    Tungsten::Vec2u res = renderer._scene->camera()->resolution();
    for(uint32 i: range(B.ref::size)) { // FIXME: SIMD
        Tungsten::Vec3f bgr = renderer._scene->camera()->_colorBuffer->_bufferA[i];
        B[i] = bgr[2];
        G[i] = bgr[1];
        R[i] = bgr[0];
    }
}

#if 1
struct Render {
    Render() {
        const Folder& sourceFolder = currentWorkingDirectory();
        static Folder tmp {"/var/tmp/light",currentWorkingDirectory(), true};
        Folder cacheFolder {"teapot", tmp, true};
        for(string file: cacheFolder.list(Files)) remove(file, cacheFolder);

        const int N = 33;
        uint2 size (1280, 1024);

        File file(str(N)+'x'+str(N)+'x'+strx(size), cacheFolder, Flags(ReadWrite|Create));
        size_t byteSize = 4ull*N*N*size.y*size.x*sizeof(half);
        assert_(byteSize <= 16ull*1024*1024*1024);
        file.resize(byteSize);
        Map map (file, Map::Prot(Map::Read|Map::Write));
        mref<half> field = mcast<half>(map);
        assert_(field.size == 4ull*N*N*size.y*size.x);
        //field.clear(); // Explicitly clears to avoid performance skew from clear on page faults (and forces memory allocation)

        mat4 camera = parseCamera(readFile(arguments()[0]));

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
            ::render(size, shearedPerspective(s, t) * camera, B, G, R);
            log("Total", time);
#if 0
#if 1
            const half* targetB = B.begin();
            const half* targetG = G.begin();
            const half* targetR = R.begin();
            for(uint i=0; i<size.y*size.x; i+=8) {
                extern ImageF B, G, R;
                *(v8hf*)(targetB+i) = toHalf(*(v8sf*)(B.data+i));
                *(v8hf*)(targetG+i) = toHalf(*(v8sf*)(G.data+i));
                *(v8hf*)(targetR+i) = toHalf(*(v8sf*)(R.data+i));
            }
#else
            assert_(source.size == size);
            extern half sRGB_reverse_half[0x100];
#if 0
            const uint8* src = (const uint8*)source.data;
            const half* targetB = B.data;
            const half* targetG = G.data;
            const half* targetR = R.data;
            for(uint i=0; i<size.y*size.x; i+=8) {
                v32ub v = *(v32ub*)(src+i*4);
                v8ub b = __builtin_shufflevector(v, v, 0*4+0, 1*4+0, 2*4+0, 3*4+0, 4*4+0, 5*4+0, 6*4+0, 7*4+0);
                v8ub g = __builtin_shufflevector(v, v, 0*4+1, 1*4+1, 2*4+1, 3*4+1, 4*4+1, 5*4+1, 6*4+1, 7*4+1);
                v8ub r = __builtin_shufflevector(v, v, 0*4+2, 1*4+2, 2*4+2, 3*4+2, 4*4+2, 5*4+2, 6*4+2, 7*4+2);
                v16hf B = (v16hf)gather(reinterpret_cast<float*>(sRGB_reverse_half), __builtin_convertvector(b, v8ui));
                v16hf G = (v16hf)gather(reinterpret_cast<float*>(sRGB_reverse_half), __builtin_convertvector(g, v8ui));
                v16hf R = (v16hf)gather(reinterpret_cast<float*>(sRGB_reverse_half), __builtin_convertvector(r, v8ui));
                *(v8hf*)(targetB+i) = __builtin_shufflevector(B, B, 0*2, 1*2, 2*2, 3*2, 4*2, 5*2, 6*2, 7*2);
                *(v8hf*)(targetG+i) = __builtin_shufflevector(G, G, 0*2, 1*2, 2*2, 3*2, 4*2, 5*2, 6*2, 7*2);
                *(v8hf*)(targetR+i) = __builtin_shufflevector(R, R, 0*2, 1*2, 2*2, 3*2, 4*2, 5*2, 6*2, 7*2);
            }
#else
            for(uint y: range(size.y)) for(uint x: range(size.x)) {
                B(x, y) = sRGB_reverse_half[source(x, y).b];
                G(x, y) = sRGB_reverse_half[source(x, y).g];
                R(x, y) = sRGB_reverse_half[source(x, y).r];
            }
#endif
#endif
#endif
        }//);
        log("Rendered",strx(uint2(N)),"x",strx(size),"images in", time);
    }
} prerender;
#endif

#if 0
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
    ViewWidget view {uint2(1280,1024), {this, &ViewApp::render}};
    unique<Window> window = nullptr;
    Image target;

    ViewApp() {
        window = ::window(&view);
    }
    Image render(uint2 targetSize, vec2 angles) {
        const float s = (angles.x+PI/3)/(2*PI/3), t = (angles.y+PI/3)/(2*PI/3);
        ::render(targetSize, s, t);
#if 1
        extern ImageF B, G, R;
        if(!target) target = Image(B.size);
        extern uint8 sRGB_forward[0x1000];
        for(uint i=0; i<B.ref::size; i++) {
            const uint b = ::min(0xFFFu, uint(0xFFF*B[i]));
            const uint g = ::min(0xFFFu, uint(0xFFF*G[i]));
            const uint r = ::min(0xFFFu, uint(0xFFF*R[i]));
            target[i] = byte4(sRGB_forward[b], sRGB_forward[g], sRGB_forward[r], 0xFF);
        }
        return unsafeShare(target);
#else
        extern Image image;
        return unsafeShare(image);
#endif
    }
} view;
#endif
