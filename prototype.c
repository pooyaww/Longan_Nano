#include <stdbool.h>
#include <float.h>
#include <stdfix.h>

#include <SDL2/SDL.h>

typedef float scalar_t;

const scalar_t SCL_MAX = FLT_MAX;
static inline scalar_t coss(scalar_t t)  { return cosf(t) ; }
static inline scalar_t sins(scalar_t t)  { return sinf(t) ; }
static inline scalar_t sqrts(scalar_t t) { return sqrtf(t); }
static inline scalar_t sq(scalar_t t)    { return t * t; }
static inline scalar_t fabss(scalar_t t) { return fabsf(t); }
static inline scalar_t fmins(scalar_t a, scalar_t b) { return fminf(a, b); }
static inline scalar_t fmaxs(scalar_t a, scalar_t b) { return fmaxf(a, b); }

typedef struct {
    scalar_t x;
    scalar_t y;
    scalar_t z;
} vec3_t;

static inline vec3_t vec3(scalar_t x, scalar_t y, scalar_t z) {
    vec3_t v = { x, y, z };
    return v;
}

static inline vec3_t vec3_add(vec3_t a, vec3_t b) {
    vec3_t v = { a.x + b.x, a.y + b.y, a.z + b.z };
    return v;
}

static inline vec3_t vec3_sub(vec3_t a, vec3_t b) {
    vec3_t v = { a.x - b.x, a.y - b.y, a.z - b.z };
    return v;
}

static inline vec3_t vec3_mul(vec3_t a, vec3_t b) {
    vec3_t v = { a.x * b.x, a.y * b.y, a.z * b.z };
    return v;
}

static inline vec3_t vec3_mulf(vec3_t a, scalar_t s) {
    vec3_t v = { a.x *s, a.y * s, a.z * s };
    return v;
}

static inline vec3_t vec3_cross(vec3_t a, vec3_t b) {
    vec3_t v = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
    return v;
}

static inline scalar_t vec3_dot(vec3_t a, vec3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline vec3_t vec3_norm(vec3_t a) {
    scalar_t inv = (scalar_t)1.0 / sqrtf(vec3_dot(a, a));
    return vec3_mulf(a, inv);
}

static inline vec3_t vec3_neg(vec3_t a) {
    vec3_t v = { -a.x, -a.y, -a.z };
    return v;
}

typedef struct {
    vec3_t  org;
    vec3_t  dir;
    vec3_t idir;
} ray_t;

typedef struct {
    vec3_t org;
    vec3_t dir;
    vec3_t right;
    vec3_t up;
} cam_t;

typedef struct {
    uint8_t r : 5;
    uint8_t g : 6;
    uint8_t b : 5;
} rgb_t;

rgb_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    rgb_t c = { r, g, b };
    return c;
}

typedef struct {
    scalar_t t;
    vec3_t   n;
} hit_t;

static inline hit_t hit(scalar_t t, vec3_t n) {
    hit_t h = { t, n };
    return h;
}

static inline hit_t intersect_sphere(ray_t ray, hit_t h, scalar_t tmin, vec3_t center, scalar_t sq_radius) {
    vec3_t oc  = vec3_sub(ray.org, center);
    scalar_t a = vec3_dot(ray.dir, ray.dir);
    scalar_t b = 2 * vec3_dot(ray.dir, oc);
    scalar_t c = vec3_dot(oc, oc) - sq_radius;

    scalar_t delta = b * b - 4 * a * c;
    if (delta >= 0) {
        scalar_t inv = (scalar_t)0.5 / a;
        scalar_t t0 = -(b + sqrts(delta)) * inv;
        scalar_t t1 = -(b - sqrts(delta)) * inv;
        scalar_t t = fmins(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
        if (t > tmin && t < h.t) {
            vec3_t p = vec3_add(vec3_mulf(ray.dir, t), ray.org);
            return hit(t, vec3_norm(vec3_sub(p, center)));
        }
    }

    return h;
}

static inline hit_t intersect_cylinder(ray_t ray, hit_t h, scalar_t tmin, vec3_t center, vec3_t dir, scalar_t sq_radius, scalar_t height) {
    vec3_t oc = vec3_sub(ray.org, center);

    scalar_t d_oc = vec3_dot(dir, oc);
    scalar_t d_rd = vec3_dot(dir, ray.dir);

    // Assumes that dir is normalized
    scalar_t a = vec3_dot(ray.dir, ray.dir) - d_rd * d_rd;
    scalar_t b = 2 * (vec3_dot(ray.dir, oc) - d_oc * d_rd);
    scalar_t c = vec3_dot(oc, oc) - d_oc * d_oc - sq_radius;

    scalar_t delta = b * b - 4 * a * c;
    if (delta >= 0) {
        scalar_t inv = (scalar_t)0.5 / a;
        scalar_t t0 = -(b + sqrts(delta)) * inv;
        scalar_t t1 = -(b - sqrts(delta)) * inv;
        scalar_t t = fmins(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
        if (t > tmin && t < h.t) {
            vec3_t p = vec3_add(vec3_mulf(ray.dir, t), ray.org);
            vec3_t q = vec3_add(center, vec3_mulf(dir, vec3_dot(dir, vec3_sub(p, center))));
            if (fabss(vec3_dot(vec3_sub(p, center), dir)) < height) {
                // Hit the side of the cylinder
                return hit(t, vec3_norm(vec3_sub(p, q)));
            } else {
                // Test the caps
                scalar_t inv = (scalar_t)1.0 / d_rd;
                scalar_t t0 = (  height - d_oc) * inv;
                // Uncomment to introduce lower cap
                //scalar_t t1 = (- height - d_oc) * inv;
                //scalar_t t = fmins(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
                scalar_t t = t0;
                vec3_t r = vec3_sub(vec3_add(vec3_mulf(ray.dir, t), ray.org), center);
                if (vec3_dot(r, r) - sq(vec3_dot(r, dir)) <= sq_radius)
                    return hit(t, t == t0 ? dir : vec3_neg(dir));
            }
        }
    }

    return h;
}

static inline hit_t intersect_box(ray_t ray, hit_t h, scalar_t tmin, vec3_t min, vec3_t max) {
    scalar_t tmin_x, tmax_x;
    {
        scalar_t t0 = (min.x - ray.org.x) * ray.idir.x;
        scalar_t t1 = (max.x - ray.org.x) * ray.idir.x;
        tmin_x = fmins(t0, t1);
        tmax_x = fmaxs(t0, t1);
    }
    scalar_t tmin_y, tmax_y;
    {
        scalar_t t0 = (min.y - ray.org.y) * ray.idir.y;
        scalar_t t1 = (max.y - ray.org.y) * ray.idir.y;
        tmin_y = fmins(t0, t1);
        tmax_y = fmaxs(t0, t1);
    }
    scalar_t tmin_z, tmax_z;
    {
        scalar_t t0 = (min.z - ray.org.z) * ray.idir.z;
        scalar_t t1 = (max.z - ray.org.z) * ray.idir.z;
        tmin_z = fmins(t0, t1);
        tmax_z = fmaxs(t0, t1);
    }
    scalar_t t0 = fmaxs(tmin, fmaxs(tmin_x, fmaxs(tmin_y, tmin_z)));
    scalar_t t1 = fmins(h.t,  fmins(tmax_x, fmins(tmax_y, tmax_z)));
    if (t0 < t1) {
        vec3_t n;
        if (t0 == tmin_x)      n = vec3(ray.dir.x > 0 ? -1 : 1, 0, 0);
        else if (t0 == tmin_y) n = vec3(0, ray.dir.y > 0 ? -1 : 1, 0);
        else                   n = vec3(0, 0, ray.dir.z > 0 ? -1 : 1);
        return hit(t0, n);
    }
    return h;
}

static inline rgb_t trace(scalar_t t, ray_t ray) {
    hit_t h = hit(SCL_MAX, vec3(0, 1, 0));
    // For testing:
    //h = intersect_box(ray, h, 0, vec3(1, 1, 0), vec3(3, 1.5, 1));
    //h = intersect_sphere(ray, h, 0, vec3(2, 0, 1), 1);
    //h = intersect_cylinder(ray, h, 0, vec3(0, 0, 1), vec3(0, 1, 0), 1, 1);

    h = intersect_box(ray, h, 0, vec3(-2, -0.5, -4), vec3(2, 1, 4));
    h = intersect_cylinder(ray, h, 0, vec3(-1, 1, -3), vec3(0, 1, 0), 0.5, 0.5);
    h = intersect_cylinder(ray, h, 0, vec3( 1, 1, -3), vec3(0, 1, 0), 0.5, 0.5);
    h = intersect_cylinder(ray, h, 0, vec3(-1, 1, -1), vec3(0, 1, 0), 0.5, 0.5);
    h = intersect_cylinder(ray, h, 0, vec3( 1, 1, -1), vec3(0, 1, 0), 0.5, 0.5);
    h = intersect_cylinder(ray, h, 0, vec3(-1, 1,  1), vec3(0, 1, 0), 0.5, 0.5);
    h = intersect_cylinder(ray, h, 0, vec3( 1, 1,  1), vec3(0, 1, 0), 0.5, 0.5);
    h = intersect_cylinder(ray, h, 0, vec3(-1, 1,  3), vec3(0, 1, 0), 0.5, 0.5);
    h = intersect_cylinder(ray, h, 0, vec3( 1, 1,  3), vec3(0, 1, 0), 0.5, 0.5);

    if (h.t < SCL_MAX) {
        scalar_t kd = fmins(1, -vec3_dot(h.n, vec3_norm(ray.dir)));
        t = t - (int16_t)t;
        t = t < 0 ? -t : t;

        scalar_t k1 = fabss((1   - t) * (0.5 - t));
        scalar_t k2 = fabss((0   - t) * (1   - t));
        scalar_t k3 = fabss((0.5 - t) * (0   - t));
        scalar_t inv = (scalar_t)1.0 / (k1 + k2 + k3);

        scalar_t r = inv * ((k1 + k3) * 0.9 + k2 * 0.5);
        scalar_t g = inv * ((k1 + k3) * 0.1 + k2 * 0.8);
        scalar_t b = inv * ((k1 + k3) * 0.4 + k2 * 0.7);
        return rgb((r * kd) * ((1 << 5) - 1),
                   (g * kd) * ((1 << 6) - 1),
                   (b * kd) * ((1 << 5) - 1));
    }
    return  rgb(255, 255, 255);
}

static inline cam_t gen_cam(scalar_t t) {
    // Rotating around z
    const scalar_t r = 6;  // distance from z
    vec3_t eye = { r * coss(t), 3, r * sins(t) };
    vec3_t dir = { -eye.x, -3, -eye.z };
    vec3_t up  = { 0, 1, 0 };

    const scalar_t ratio   = 0.5; // height / width
    const scalar_t w = 1.0, h = w * ratio; // 1.6 ~= fov of 90 deg
    vec3_t right = vec3_norm(vec3_cross(dir, up));
    up = vec3_norm(vec3_cross(right, dir));
    dir = vec3_norm(dir);

    right = vec3_mulf(right, w);
    up    = vec3_mulf(up, h);

    cam_t cam = { eye, dir, right, up };
    return cam;
}

static inline ray_t gen_ray(cam_t cam, scalar_t x, scalar_t y) {
    ray_t ray;
    ray.org = cam.org;
    ray.dir = vec3_add(cam.dir, vec3_add(vec3_mulf(cam.right, 2 * x - 1), vec3_mulf(cam.up, 1 - 2 * y)));
    ray.idir = vec3((scalar_t)1.0 / ray.dir.x, (scalar_t)1.0 / ray.dir.y, (scalar_t)1.0 / ray.dir.z);
    return ray;
}

bool handle_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                return true;
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        return true;
                }
                break;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return 1;

    const int w = 128, h = 64, scale = 5;
    SDL_Window* win = SDL_CreateWindow("HaGrid", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w * scale, h * scale, 0);
    SDL_Surface* screen = SDL_GetWindowSurface(win);

    rgb_t img[w * h];
    cam_t cam;

    bool done = false;
    scalar_t t = 0;
    while (!done) {
        cam = gen_cam(t);
        for (uint8_t y = 0; y < h; y++) {
            for (uint8_t x = 0; x < w; x++) {
                ray_t ray = gen_ray(cam, (scalar_t)x / (scalar_t)w, (scalar_t)y / (scalar_t)h);
                img[(uint16_t)y * (uint16_t)w + (uint16_t)x] = trace(t, ray);
            }
        }
        t += (scalar_t)0.1;
        
        SDL_LockSurface(screen);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                for (int i = 0; i < scale; i++) {
                    uint8_t* row = (uint8_t*)screen->pixels + screen->pitch * (y * scale + i);
                    const uint8_t r = screen->format->Rshift / 8;
                    const uint8_t g = screen->format->Gshift / 8;
                    const uint8_t b = screen->format->Bshift / 8;
                    const uint8_t a = screen->format->Ashift / 8;
                    rgb_t pix = img[y * w + x];
                    for (int j = 0; j < scale; j++) {
                        row[(x * scale + j) * 4 + r] = pix.r * 255 / ((1 << 5) - 1);
                        row[(x * scale + j) * 4 + g] = pix.g * 255 / ((1 << 6) - 1);
                        row[(x * scale + j) * 4 + b] = pix.b * 255 / ((1 << 5) - 1);
                        row[(x * scale + j) * 4 + a] = 255;
                    }
                }
            }
        }
        SDL_UnlockSurface(screen);

        SDL_Delay(200);

        SDL_UpdateWindowSurface(win);
        done = handle_events();
    }

    SDL_DestroyWindow(win);
    SDL_Quit();

    return 0;
}
