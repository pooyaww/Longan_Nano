#include <avr/io.h>
#include <avr/cpufunc.h>
#include <stdint.h>
#include <float.h>

#define F_CPU 16000000UL
#include <util/delay.h>

#define CS  PB3 // Chip select
#define RST PB5 // Reset
#define DC  PB4 // Data/Command
#define SCL PB2 // Clock line
#define SDA PB0 // Data line

static void send_byte(uint8_t byte) {
    PORTB |= 1 << SCL;
    #pragma unroll
    for (int8_t i = 7; i >= 0; i--) {
        PORTB &= ~(1 << SCL);
        if (byte & 0x80)
            PORTB |=  (1 << SDA);
        else
            PORTB &= ~(1 << SDA);
        _delay_us(5);
        PORTB |= 1 << SCL;
        byte <<= 1;
        _delay_us(5);
    }
}

static void send_command(uint8_t cmd) {
    PORTB &= ~(1 << DC);
    PORTB &= ~(1 << CS);
    send_byte(cmd);
    PORTB |=  (1 << CS);
}

static void send_data(uint8_t cmd) {
    PORTB |=  (1 << DC);
    PORTB &= ~(1 << CS);
    send_byte(cmd);
    PORTB |=  (1 << CS);
}

static void fill(uint16_t col) {
    send_command(0x26); // Enable fill
    send_command(0x01);

    send_command(0x22); // Draw rectangle
    send_command(0x00);
    send_command(0x00);
    send_command(0x5F);
    send_command(0x3F);
    send_command((col & 0x3F) << 1);
    send_command((col >> 5) & 0x1F);
    send_command((col >> 10) & 0x3F);
    send_command((col & 0x3F) << 1);
    send_command((col >> 5) & 0x1F);
    send_command((col >> 10) & 0x3F);
}

static void cursor(uint8_t x, uint8_t y) {
    send_command(0x15);
    send_command(x);
    send_command(0x5F);

    send_command(0x75);
    send_command(y);
    send_command(0x3F);
}

static void init() {
    PORTB &= ~(1 << CS);

    PORTB |=  (1 << RST);
    _delay_ms(500);
    PORTB &= ~(1 << RST);
    _delay_ms(500);
    PORTB |=  (1 << RST);
    _delay_ms(500);

    send_command(0xAF); // Display OFF
    send_command(0xA0); // Set remap
    send_command(0x72); // RGB
    send_command(0xA1); // Start line
    send_command(0x00);
    send_command(0xA2); // Display offset
    send_command(0x00);

    send_command(0xA4); // Normal display
    send_command(0xA8); // Set multiplex
    send_command(0x3F); // 1/64 duty
    send_command(0xAD); // Set master
    send_command(0x8E); 
    send_command(0xB0); // Power mode
    send_command(0x0B);
    send_command(0xB1); // Precharge
    send_command(0x31);

    send_command(0xB3); // Clock div
    send_command(0xF0); // 7:4 -> freq., 3:0 -> div ratio

    send_command(0x8A); // Precharge A
    send_command(0x64);
    send_command(0x8B); // Precharge B
    send_command(0x78);
    send_command(0x8C); // Precharge C
    send_command(0x64);
    send_command(0xBB); // Precharge level
    send_command(0x3A);

    send_command(0xBE); // V_COMH
    send_command(0x3E);

    send_command(0x87); // Master current
    send_command(0x06);
    send_command(0x81); // Contrast A
    send_command(0x91);
    send_command(0x82); // Contrast B
    send_command(0x50);
    send_command(0x83); // Contrast C
    send_command(0x7D);

    send_command(0xAF); // Display ON
}

static inline float sq(float t)    { return t * t; }

typedef struct {
    union {
        struct { float x, y, z; };
        float v[3];
    };
} vec3_t;

static inline vec3_t vec3(float x, float y, float z) {
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

static inline vec3_t vec3_mulf(vec3_t a, float s) {
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

static inline float vec3_dot(vec3_t a, vec3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline vec3_t vec3_norm(vec3_t a) {
    float inv = (float)1.0 / sqrtf(vec3_dot(a, a));
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
    float t;
    vec3_t   n;
} hit_t;

static inline hit_t hit(float t, vec3_t n) {
    hit_t h = { t, n };
    return h;
}

static void intersect_sphere(const ray_t* ray, hit_t* h, float tmin, vec3_t center, float sq_radius) {
    vec3_t oc  = vec3_sub(ray->org, center);
    float a = vec3_dot(ray->dir, ray->dir);
    float b = 2 * vec3_dot(ray->dir, oc);
    float c = vec3_dot(oc, oc) - sq_radius;

    float delta = b * b - 4 * a * c;
    if (delta >= 0) {
        float inv = (float)0.5 / a;
        float t0 = -(b + sqrtf(delta)) * inv;
        float t1 = -(b - sqrtf(delta)) * inv;
        float t = fminf(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
        if (t > tmin && t < h->t) {
            vec3_t p = vec3_add(vec3_mulf(ray->dir, t), ray->org);
            h->t = t;
            h->n = vec3_norm(vec3_sub(p, center));
        }
    }
}

static void intersect_cylinder(const ray_t* ray, hit_t* h, float tmin, vec3_t center, vec3_t dir, float sq_radius, float height) {
    vec3_t oc = vec3_sub(ray->org, center);

    float d_oc = vec3_dot(dir, oc);
    float d_rd = vec3_dot(dir, ray->dir);

    // Assumes that dir is normalized
    float a = vec3_dot(ray->dir, ray->dir) - d_rd * d_rd;
    float b = 2 * (vec3_dot(ray->dir, oc) - d_oc * d_rd);
    float c = vec3_dot(oc, oc) - d_oc * d_oc - sq_radius;

    float delta = b * b - 4 * a * c;
    if (delta >= 0) {
        float inv = (float)0.5 / a;
        float t0 = -(b + sqrtf(delta)) * inv;
        float t1 = -(b - sqrtf(delta)) * inv;
        float t = fminf(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
        if (t > tmin && t < h->t) {
            vec3_t p = vec3_add(vec3_mulf(ray->dir, t), ray->org);
            vec3_t q = vec3_add(center, vec3_mulf(dir, vec3_dot(dir, vec3_sub(p, center))));
            if (fabsf(vec3_dot(vec3_sub(p, center), dir)) < height) {
                // Hit the side of the cylinder
                h->t = t;
                h->n = vec3_sub(p, q);
            } else {
                // Test the caps
                float inv = (float)1.0 / d_rd;
                float t0 = (height - d_oc) * inv;
                // Uncomment to introduce lower cap
                //float t1 = (- height - d_oc) * inv;
                //float t = fminf(t0 > tmin ? t0 : t1, t1 > tmin ? t1 : t0);
                float t = t0;
                vec3_t r = vec3_sub(vec3_add(vec3_mulf(ray->dir, t), ray->org), center);
                if (vec3_dot(r, r) - sq(vec3_dot(r, dir)) <= sq_radius) {
                    h->t = t;
                    h->n = t == t0 ? dir : vec3_neg(dir);
                }
            }
        }
    }
}

static void intersect_box(const ray_t* ray, hit_t* h, float tmin, vec3_t min, vec3_t max) {
    float t0 = tmin, t1 = h->t;
    uint8_t axis = 0;
    for (uint8_t i = 0; i < 3; i++) {
        float tmin = (min.v[i] - ray->org.v[i]) / ray->dir.v[i];
        float tmax = (max.v[i] - ray->org.v[i]) / ray->dir.v[i];
        float tx = fminf(tmin, tmax);
        if (t0 < tx) { t0 = tx; axis = i; }
        t1 = fminf(t1, fmaxf(tmin, tmax));
    }
    if (t0 < t1) {
        h->t = t0;
        h->n = vec3(0, 0, 0);
        h->n.v[axis] = ray->dir.v[axis] > 0 ? -1 : 1;
    }
}

static rgb_t trace(float t, const ray_t* ray) {
    hit_t h = hit(FLT_MAX, vec3(0, 1, 0));

    intersect_box(ray, &h, 0, vec3(-2, -0.5, -4), vec3(2, 1, 4));
    for (uint8_t x = 0; x < 2; x++) {
        for (uint8_t y = 0; y < 4; y++) {
            intersect_cylinder(ray, &h, 0, vec3(-1 + x * 2, 1, -3 + y * 2), vec3(0, 1, 0), 0.5, 0.5);
        }
    }

    if (h.t < FLT_MAX) {
        float kd = fminf(1, -vec3_dot(h.n, vec3_norm(ray->dir)));
        t = t - (int16_t)t;
        t = t < 0 ? -t : t;

        float k1 = fabsf((1   - t) * (0.5 - t));
        float k2 = fabsf((0   - t) * (1   - t));
        float k3 = fabsf((0.5 - t) * (0   - t));
        float inv = (float)1.0 / (k1 + k2 + k3);

        float r = inv * ((k1 + k3) * 0.9 + k2 * 0.5);
        float g = inv * ((k1 + k3) * 0.1 + k2 * 0.8);
        float b = inv * ((k1 + k3) * 0.4 + k2 * 0.7);
        return rgb((r * kd) * ((1 << 3) - 1),
                   (g * kd) * ((1 << 4) - 1),
                   (b * kd) * ((1 << 3) - 1));
    }
    return rgb(255, 255, 255);
}

static inline cam_t gen_cam(float t) {
    // Rotating around z
    const float r = 6;  // distance from z
    vec3_t eye = { r * cosf(t), 3, r * sinf(t) };
    vec3_t dir = { -eye.x, -3, -eye.z };
    vec3_t up  = { 0, 1, 0 };

    const float ratio   = 0.5; // height / width
    const float w = 1.0, h = w * ratio; // 1.6 ~= fov of 90 deg
    vec3_t right = vec3_norm(vec3_cross(dir, up));
    up = vec3_norm(vec3_cross(right, dir));
    dir = vec3_norm(dir);

    right = vec3_mulf(right, w);
    up    = vec3_mulf(up, h);

    cam_t cam = { eye, dir, right, up };
    return cam;
}

static inline ray_t gen_ray(cam_t cam, float x, float y) {
    ray_t ray;
    ray.org = cam.org;
    ray.dir = vec3_add(cam.dir, vec3_add(vec3_mulf(cam.right, 2 * x - 1), vec3_mulf(cam.up, 1 - 2 * y)));
    return ray;
}

int main(void) {
    DDRB = 0xFF;

    _delay_ms(10);
    init();

    fill(0xFFFF);

    const uint8_t w = 96, h = 64;
    float t = 0;
    while (1) {
        cam_t cam = gen_cam(t);
        cursor(0, 0);
        for (uint8_t y = 0; y < h; y++) {
            for (uint8_t x = 0; x < w; x++) {
                ray_t ray = gen_ray(cam, (float)x / (float)w, (float)y / (float)h);
                rgb_t color = trace(t, &ray);
                send_data((color.r << 3)   | (color.g >> 3));
                send_data(((color.g & 0x07) << 5) | color.b);
            }
        }
        t += (float)0.1;
    }
}
