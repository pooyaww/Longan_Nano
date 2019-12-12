#include <stdint.h>
#include "lcd/lcd.h"
#include <string.h>
#include <unistd.h>

#define FP_SHIFT 16
#define FP_SIZE 32

// 1.0 corresponds to 1<<FP_SHIFT
typedef int32_t fixed;
#define FP_MAX (~(fixed)(1<<(FP_SIZE-1)))
#define FP_ONE int2fp(1)

static inline fixed int2fp(int a) {
    return a << FP_SHIFT;
}

static inline fixed fpmul(fixed a, fixed b) {
    return (fixed)((a * (int64_t)b) >> FP_SHIFT);
}

static inline fixed fpdiv(fixed a, fixed b) {
    if (b == 0) return FP_MAX;
    return (fixed)(((int64_t)a << FP_SHIFT) / b);
}

static inline fixed fpmin(fixed a, fixed b) { return a < b ? a : b; }
static inline fixed fpmax(fixed a, fixed b) { return a > b ? a : b; }
static inline fixed fpabs(fixed a) { return a < 0 ? -a : a; }
static inline fixed fpsqr(fixed t) { return fpmul(t, t); }

static inline fixed fpsin(fixed x) {
    // taken from http://www.coranac.com/2009/07/sines/
    // @todo will break if FP_SHIFT != 16
    x = (fixed)(x * (int64_t)200000/314159);
    static const int qN = 13, qA = 16, qP = 15, qR = 2*qN-qP, qS = qN+qP+1-qA;
    x = x<<(30-16);
    if ((x^(x<<1)) < 0) x = (1<<31) - x;
    x = x>>(30-qN);
    return x * ((3<<qP) - (x*x>>qR)) >> qS;
}

static inline fixed fpcos(fixed x) {
    // adapted from http://www.coranac.com/2009/07/sines/
    // @todo will break if FP_SHIFT != 16
    x = (fixed)(x * (int64_t)200000/314159);
    x += FP_ONE;
    static const int qN = 13, qA = 16, qP = 15, qR = 2*qN-qP, qS = qN+qP+1-qA;
    x = x<<(30-16);
    if ((x^(x<<1)) < 0) x = (1<<31) - x;
    x = x>>(30-qN);
    return x * ((3<<qP) - (x*x>>qR)) >> qS;
}

static inline fixed fpsqrt(fixed a) {
    // taken from https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
    // @todo will break if FP_SHIFT != 16
    uint32_t t, q, b, r;
    r = a;
    b = 0x40000000;
    q = 0;
    while(b > 0x40) {
        t = q + b;
        if(r >= t) {
            r -= t;
            q = t + b; // equivalent to q += 2*b
        }
        r <<= 1;
        b >>= 1;
    }
    q >>= 8;
    return q;
}

typedef struct {
    union {
        struct { fixed x, y, z; };
        fixed v[3];
    };
} vec3_t;

static inline vec3_t vec3(fixed x, fixed y, fixed z) {
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
    vec3_t v = { fpmul(a.x, b.x), fpmul(a.y, b.y), fpmul(a.z, b.z) };
    return v;
}

static inline vec3_t vec3_mulf(vec3_t a, fixed s) {
    vec3_t v = { fpmul(a.x, s), fpmul(a.y, s), fpmul(a.z, s) };
    return v;
}

static inline vec3_t vec3_cross(vec3_t a, vec3_t b) {
    vec3_t v = {
        fpmul(a.y, b.z) - fpmul(a.z, b.y),
        fpmul(a.z, b.x) - fpmul(a.x, b.z),
        fpmul(a.x, b.y) - fpmul(a.y, b.x)
    };
    return v;
}

static inline fixed vec3_dot(vec3_t a, vec3_t b) {
    return fpmul(a.x, b.x) + fpmul(a.y, b.y) + fpmul(a.z, b.z);
}

static inline vec3_t vec3_norm(vec3_t a) {
    fixed inv = fpdiv(FP_ONE, fpsqrt(vec3_dot(a, a)));
    return vec3_mulf(a, inv);
}

static inline vec3_t vec3_neg(vec3_t a) {
    vec3_t v = { -a.x, -a.y, -a.z };
    return v;
}

typedef struct {
    vec3_t  org;
    vec3_t  dir;
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

static inline rgb_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    rgb_t c = { r, g, b };
    return c;
}

typedef struct {
    fixed t;
    vec3_t n;
} hit_t;

static inline hit_t hit(fixed t, vec3_t n) {
    hit_t h = { t, n };
    return h;
}

static void intersect_sphere(const ray_t* ray, hit_t* h, fixed tmin, vec3_t center, fixed sq_radius) {
    vec3_t oc  = vec3_sub(ray->org, center);
    fixed a = vec3_dot(ray->dir, ray->dir);
    fixed b = 2 * vec3_dot(ray->dir, oc);
    fixed c = vec3_dot(oc, oc) - sq_radius;

    fixed delta = fpmul(b, b) - 4 * fpmul(a, c);
    if (delta >= 0) {
        fixed inv = fpdiv(FP_ONE/2, a);
        fixed t0 = -fpmul(b + fpsqrt(delta), inv);
        fixed t1 = -fpmul(b - fpsqrt(delta), inv);
        fixed t = fpmin(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
        if (t > tmin && t < h->t) {
            vec3_t p = vec3_add(vec3_mulf(ray->dir, t), ray->org);
            h->t = t;
            h->n = vec3_norm(vec3_sub(p, center));
        }
    }
}

static void intersect_cylinder(const ray_t* ray, hit_t* h, fixed tmin, vec3_t center, vec3_t dir, fixed sq_radius, fixed height) {
    vec3_t oc = vec3_sub(ray->org, center);

    fixed d_oc = vec3_dot(dir, oc);
    fixed d_rd = vec3_dot(dir, ray->dir);

    // Assumes that dir is normalized
    fixed a = vec3_dot(ray->dir, ray->dir) - fpmul(d_rd, d_rd);
    fixed b = 2 * (vec3_dot(ray->dir, oc) - fpmul(d_oc, d_rd));
    fixed c = vec3_dot(oc, oc) - fpmul(d_oc, d_oc) - sq_radius;

    fixed delta = fpmul(b, b) - 4 * fpmul(a, c);
    if (delta >= 0) {
        fixed inv = fpdiv(FP_ONE/2, a);
        fixed t0 = -fpmul(b + fpsqrt(delta), inv);
        fixed t1 = -fpmul(b - fpsqrt(delta), inv);
        fixed t = fpmin(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
        if (t > tmin && t < h->t) {
            vec3_t p = vec3_add(vec3_mulf(ray->dir, t), ray->org);
            vec3_t q = vec3_add(center, vec3_mulf(dir, vec3_dot(dir, vec3_sub(p, center))));
            if (fpabs(vec3_dot(vec3_sub(p, center), dir)) < height) {
                // Hit the side of the cylinder
                h->t = t;
                h->n = vec3_sub(p, q);
            } else {
                // Test the caps
                fixed inv = fpdiv(FP_ONE, d_rd);
                fixed t0 = fpmul(height - d_oc, inv);
                // Uncomment to introduce lower cap
                //fixed t1 = (- height - d_oc) * inv;
                //fixed t = fminf(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
                fixed t = t0;
                vec3_t r = vec3_sub(vec3_add(vec3_mulf(ray->dir, t), ray->org), center);
                if (vec3_dot(r, r) - fpsqr(vec3_dot(r, dir)) <= sq_radius) {
                    h->t = t;
                    h->n = t == t0 ? dir : vec3_neg(dir);
                }
            }
        }
    }
}

static void intersect_box(const ray_t* ray, hit_t* h, fixed tmin, vec3_t min, vec3_t max) {
    fixed t0 = tmin, t1 = h->t;
    uint8_t axis = 0;
    for (uint8_t i = 0; i < 3; i++) {
        fixed tmin = fpdiv(min.v[i] - ray->org.v[i], ray->dir.v[i]);
        fixed tmax = fpdiv(max.v[i] - ray->org.v[i], ray->dir.v[i]);
        fixed tx = fpmin(tmin, tmax);
        if (t0 < tx) { t0 = tx; axis = i; }
        t1 = fpmin(t1, fpmax(tmin, tmax));
    }
    if (t0 < t1) {
        h->t = t0;
        h->n = vec3(0, 0, 0);
        h->n.v[axis] = int2fp(ray->dir.v[axis] > 0 ? -1 : 1);
    }
}

static rgb_t trace(fixed t, const ray_t* ray) {
    hit_t h = hit(FP_MAX, vec3(0, FP_ONE, 0));

    intersect_box(ray, &h, 0,
        vec3(-2*FP_ONE, -FP_ONE/2, -4*FP_ONE),
        vec3(2*FP_ONE, 1*FP_ONE, 4*FP_ONE)
    );

    for (uint8_t x = 0; x < 2; x++) {
        for (uint8_t y = 0; y < 4; y++) {
            intersect_cylinder(ray, &h, 0,
                vec3(int2fp(-1 + x * 2), int2fp(1), int2fp(-3 + y * 2)),
                vec3(int2fp(0), int2fp(1), int2fp(0)),
                FP_ONE/2,
                FP_ONE/2
            );
        }
    }

    if (h.t < FP_MAX) {
        fixed kd = fpmin(FP_ONE, -vec3_dot(h.n, vec3_norm(ray->dir)));
        t = t - (int16_t)t;
        t = t < 0 ? -t : t;

        fixed k1 = fpabs(fpmul(FP_ONE   - t, FP_ONE/2 - t));
        fixed k2 = fpabs(fpmul(0        - t, FP_ONE   - t));
        fixed k3 = fpabs(fpmul(FP_ONE/2 - t, 0        - t));
        fixed inv = fpdiv(FP_ONE, k1 + k2 + k3);

        fixed r = fpmul(inv, (k1 + k3) / 10 * 9 + k2 / 10 * 5);
        fixed g = fpmul(inv, (k1 + k3) / 10 * 1 + k2 / 10 * 8);
        fixed b = fpmul(inv, (k1 + k3) / 10 * 4 + k2 / 10 * 7);
        return rgb(fpmul(r, kd) * ((1 << 3) - 1) / FP_ONE,
                   fpmul(g, kd) * ((1 << 4) - 1) / FP_ONE,
                   fpmul(b, kd) * ((1 << 3) - 1) / FP_ONE);
    }
    return rgb(255, 255, 255);
}

static inline cam_t gen_cam(fixed t) {
    // Rotating around z
    const uint8_t r = 6;  // distance from z
    vec3_t eye = { r * fpcos(t), 3*FP_ONE, r * fpsin(t) };
    vec3_t dir = { -eye.x, -3*FP_ONE, -eye.z };
    vec3_t up  = { 0, FP_ONE, 0 };

    const fixed ratio = FP_ONE/2; // height / width
    const fixed w = FP_ONE, h = fpmul(w, ratio); // 1.6 ~= fov of 90 deg
    vec3_t right = vec3_norm(vec3_cross(dir, up));
    up = vec3_norm(vec3_cross(right, dir));
    dir = vec3_norm(dir);

    right = vec3_mulf(right, w);
    up    = vec3_mulf(up, h);

    cam_t cam = { eye, dir, right, up };
    return cam;
}

static inline ray_t gen_ray(cam_t cam, fixed x, fixed y) {
    ray_t ray;
    ray.org = cam.org;
    ray.dir = vec3_add(
        cam.dir,
        vec3_add(
            vec3_mulf(cam.right, 2 * x - FP_ONE),
            vec3_mulf(cam.up, FP_ONE - 2 * y)
        )
    );
    return ray;
}

int main(void) {

    Lcd_Init();
    LCD_Clear(WHITE);
    BACK_COLOR = WHITE;

    LEDR(1);
    LEDG(1);
    LEDB(1);

    const uint8_t w = 160, h = 80;
    float t = 0;
    while (1) {
        cam_t cam = gen_cam(t);
        LCD_Address_Set(0, 0, w - 1, h - 1);
        for (uint8_t y = 0; y < h; y++) {
            for (uint8_t x = 0; x < w; x++) {
                ray_t ray = gen_ray(cam, int2fp(x)/w, int2fp(y)/h);
                rgb_t color = trace(t, &ray);
                u16 data = color.b | (color.g << 5) | (color.r << 11);
                LCD_WR_DATA(data);
            }
        }
        t += FP_ONE/10;
    }
}
