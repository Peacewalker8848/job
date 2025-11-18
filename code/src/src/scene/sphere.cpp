#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
    Vector3D oc = r.o - o;

    double a = dot(r.d, r.d);
    double b = 2.0 * dot(oc, r.d);
    double c = dot(oc, oc) - r2;

    double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return false;

    double sqrt_disc = sqrt(disc);
    t1 = (-b - sqrt_disc) / (2.0 * a);
    t2 = (-b + sqrt_disc) / (2.0 * a);

    if (t1 > t2) std::swap(t1, t2);


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
    double t1, t2;
    if (!test(r, t1, t2)) return false;

    double t = t1;
    if (t < r.min_t) t = t2;

    if (t < r.min_t || t > r.max_t) return false;

    // update the nearest hit
    Ray& ray = const_cast<Ray&>(r);
    ray.max_t = t;


  return true;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
    double t1, t2;
    if (!test(r, t1, t2)) return false;

    double t = t1;
    if (t < r.min_t) t = t2;

    if (t < r.min_t || t > r.max_t) return false;

    // intersection found -> update ray.max_t
    Ray& ray = const_cast<Ray&>(r);
    ray.max_t = t;

    // only update intersection if this is the closest so far
    if (t < i->t) {

        i->t = t;

        Vector3D p = r.o + t * r.d;
        i->n = normal(p);

        i->primitive = this;
        i->bsdf = get_bsdf();
        i->hit_p = r.o + r.d * i->t;
    }


  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
