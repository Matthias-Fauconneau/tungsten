#include "Version.hpp"
#include "Shared.hpp"

#include "matrix.h"
#include "interface.h"
#include "window.h"
#include "view-widget.h"
#include "variant.h"

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
#if 1
    const vec3 look_at = Vec3(t.at("look_at"));
    const vec3 position = Vec3(t.at("position"));
    const vec3 z = normalize(look_at - position);
    vec3 y = Vec3(t.at("up"));
    y = normalize(y - dot(y,z)*z); // Projects up on plane orthogonal to z
    const vec3 x = cross(y, z);
    transform[0] = vec4(x, 0);
    transform[1] = vec4(y, 0);
    transform[2] = vec4(z, 0);
    transform[3] = vec4(-position, 1);
#else
    if(t.contains("position")) {
        ref<Variant> position = t.at("position");
        transform.translate(vec3((float)position[0],(float)position[1],(float)position[2]));
    }
    if(t.contains("scale")) {
        ref<Variant> scale = t.at("scale");
        transform.scale(vec3((float)scale[0], (float)scale[1], (float)scale[2]));
    }
    if(t.contains("rotation")) {
        ref<Variant> rotation = t.at("rotation");
        transform.rotateX(rotation[0]*PI/180);
        transform.rotateY(rotation[1]*PI/180);
        transform.rotateZ(rotation[2]*PI/180);
    }
    error(t);
#endif
    return transform;
}

mat4 parseCamera(ref<byte> file) {
    TextData s (file);
    Variant root = parseJSON(s);
    const Dict& camera = root.dict.at("camera");
    mat4 modelView = ::transform( camera ).inverse();
    //const float fov = float(camera.at("fov"))*PI/180;
    //const float S = 1/(tan(fov/2));
    //const vec2 resolution = Vec2(camera.at("resolution"));
    /*mat4 M; // perspective
    M(0,0) = S;
    M(1,1) = S*resolution.y/resolution.x;
    const float near = 0.01, far = 100; // FIXME
    M(2,2) = -far/(far-near);
    M(2,3) = -1;
    M(3,2) = -(far*near)/(far-near);
    M(3,3) = 0;
    return M * modelView;*/
    modelView.rotateZ(PI); // -Z
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
    const float near = 1-1./2, far = 1+1./2;
    M(2,2) = - (far+near) / (far-near);
    M(2,3) = - 2*far*near / (far-near);
    M(3,2) = - 1;
    M(3,3) = 0;
    M.translate(vec3(-S,-T,0));
    M.translate(vec3(0,0,-1)); // 0 -> -1 (Z-)
    return M;
}

Image render(uint2 targetSize, float s, float t) {
    /*mat4 transform;
    transform.translate(vec3(23.1701, 16.7142, -0.172476)); // 23.1701, t+16.7142, s+-0.172476
    //transform.rotateX(-PI/5);
    transform.rotateX(PI);*/
    //transform.rotateY(PI/2);
    mat4 camera = parseCamera(readFile(arguments()[0]));

    // Sheared perspective (rectification)
    extern mat4 M;
    M = shearedPerspective(s, t) * camera;

    {
        using namespace Tungsten;

        extern int argc; extern const char** argv;
        CliParser parser("tungsten", "[options] scene1 [scene2 [scene3...]]");

        StandaloneRenderer renderer(parser, std::cout);

        parser.parse(argc, argv);

        if (parser.isPresent(OPT_VERSION)) {
            std::cout << "tungsten, version " << VERSION_STRING << std::endl;
            std::exit(0);
        }

        renderer.setup();

        while (renderer.renderScene());
    }

    extern Image image;
    assert_(image.size == targetSize);
    return ::move(image);
}

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

    ViewApp() {
        window = ::window(&view);
    }
    Image render(uint2 targetSize, vec2 angles) {
        const float s = (angles.x+PI/3)/(2*PI/3), t = (angles.y+PI/3)/(2*PI/3);
        return ::render(targetSize, s, t);
    }
} view;
