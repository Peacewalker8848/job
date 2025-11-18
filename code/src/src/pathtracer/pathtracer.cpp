#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

    
PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
    Vector3D L_out;

    // 局部坐标系：z 轴对齐法线
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    // 出射方向（局部）
    Vector3D wo_world = -r.d;
    Vector3D wo = w2o * wo_world;

    int n_samples = ns_area_light;    // 或者你有专门的 hemisphereSamples，就用那个

    for (int i = 0; i < n_samples; i++) {

        // 1）在局部半球均匀采样
        Vector3D wi_local = hemisphereSampler->get_sample();   // Uniform 半球采样
        if (wi_local.z <= 0) continue;
        double pdf = 1.0 / (2.0 * PI); // 均匀半球

        // 2）转回世界坐标
        Vector3D wi_world = o2w * wi_local;

        // 3）发射 shadow ray，看会不会先撞其它物体
        Ray shadow_ray(isect.hit_p + EPS_D * isect.n, wi_world);
        Intersection light_isect;
        if (!bvh->intersect(shadow_ray, &light_isect)) continue;

        // 4）只统计光源的贡献（普通物体 get_emission = 0）
        Vector3D Le = light_isect.bsdf->get_emission();
        if (Le.x == 0 && Le.y == 0 && Le.z == 0) continue;

        // 5）BRDF * Li * cos / pdf
        Vector3D f = isect.bsdf->f(wo, wi_local);
        double cos_theta = wi_local.z;

        L_out += f * Le * cos_theta / pdf;
    }

    return L_out / n_samples;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D hit_p = r.o + r.d * isect.t;
    Vector3D wo = w2o * (-r.d);

    Vector3D L_out(0);

    for (SceneLight* light : scene->lights) {

        int sample_num = light->is_delta_light() ? 1 : ns_area_light;

        for (int i = 0; i < sample_num; i++) {

            Vector3D wi_world, Li;
            double dist_to_light, pdf;

            // 从光源采样
            Li = light->sample_L(hit_p, &wi_world, &dist_to_light, &pdf);
            if (pdf <= 0) continue;

            // 转换到局部坐标
            Vector3D wi = w2o * wi_world;

            // 夹角不对称，跳过
            if (wi.z <= 0) continue;

            // shadow ray
            Ray shadow(hit_p + isect.n * EPS_F, wi_world);
            shadow.max_t = dist_to_light - EPS_F;

            if (bvh->has_intersection(shadow)) continue;

            // BRDF
            Vector3D f = isect.bsdf->f(wo, wi);

            L_out += (Li * f * wi.z) / pdf;
        }
    }

    return L_out;

}
Vector3D PathTracer::zero_bounce_radiance(const Ray& r,
    const Intersection& isect) {
    // 零次反射：只有物体本身的自发光（光源）贡献

    // 如果是光源，返回其发光强度
    return isect.bsdf->get_emission();
}
Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`


    if (direct_hemisphere_sample)
        return estimate_direct_lighting_hemisphere(r, isect);

    return estimate_direct_lighting_importance(r, isect);


}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
    Vector3D L_out(0);

    // 一次反射
    L_out += one_bounce_radiance(r, isect);

    // Russian Roulette
    if (r.depth == 0) return L_out;

    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D wo = w2o * (-r.d);

    Vector3D wi;
    double pdf;

    Vector3D f = isect.bsdf->sample_f(wo, &wi, &pdf);

    if (pdf < 1e-6) return L_out;

    // 世界坐标方向
    Vector3D wi_world = o2w * wi;

    Ray bounce_ray(isect.hit_p + isect.n * EPS_F, wi_world);
    bounce_ray.depth = r.depth - 1;

    // 递归下一次反弹
    Vector3D L_indirect =
        est_radiance_global_illumination(bounce_ray);

    L_out += f * L_indirect * fabs(wi.z) / pdf;

    return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
    Intersection isect;

    if (!bvh->intersect(r, &isect))
        return envLight ? envLight->sample_dir(r) : Vector3D(0);

    return zero_bounce_radiance(r, isect)
        + at_least_one_bounce_radiance(r, isect);
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
    // TODO (Part 1.2):
    // Make a loop that generates num_samples camera rays and traces them
    // through the scene. Return the average Vector3D.
    // You should call est_radiance_global_illumination in this function.

    // TODO (Part 5):
    // Modify your implementation to include adaptive sampling.
    // Use the command line parameters "samplesPerBatch" and "maxTolerance"

    int num_samples = ns_aa;               // 每个像素总采样数（命令行 -s 控制）
    Vector2D origin((double)x, (double)y); // 像素左下角

    Vector3D L_sum(0.0, 0.0, 0.0);

    for (int i = 0; i < num_samples; ++i) {
        // 在当前像素内做一次均匀随机采样（[0,1)×[0,1)）
        Vector2D sp = gridSampler->get_sample();
        Vector2D pixel_sample = origin + sp;

        // 映射到 [0,1]×[0,1] 的标准屏幕坐标
        double sx = pixel_sample.x / sampleBuffer.w;
        double sy = pixel_sample.y / sampleBuffer.h;

        // 生成一次相机光线
        Ray r = camera->generate_ray(sx, sy);
        r.depth = max_ray_depth;

        // 跟踪这条光线得到该采样的辐射度
        Vector3D Li = est_radiance_global_illumination(r);
        L_sum += Li;
    }

    // 取平均，写入 sampleBuffer
    Vector3D L = L_sum / (double)num_samples;
    sampleBuffer.update_pixel(L, x, y);

    // 记录这个像素实际用了多少样本（后面自适应采样会用到）
    sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
