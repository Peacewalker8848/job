#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
     // 使用 slab 方法，对 x/y/z 三个轴分别计算交点区间，然后求交集
    double tmin = t0;
    double tmax = t1;

    // --- X 轴 ---
    if (r.d.x != 0.0) {
        double tx1 = (min.x - r.o.x) / r.d.x;
        double tx2 = (max.x - r.o.x) / r.d.x;
        if (tx1 > tx2) std::swap(tx1, tx2);
        tmin = std::max(tmin, tx1);
        tmax = std::min(tmax, tx2);
        if (tmin > tmax) return false;
    }
    else {
        // 射线方向在 x 轴为 0，如果原点在盒子外侧，永远不会相交
        if (r.o.x < min.x || r.o.x > max.x) return false;
    }

    // --- Y 轴 ---
    if (r.d.y != 0.0) {
        double ty1 = (min.y - r.o.y) / r.d.y;
        double ty2 = (max.y - r.o.y) / r.d.y;
        if (ty1 > ty2) std::swap(ty1, ty2);
        tmin = std::max(tmin, ty1);
        tmax = std::min(tmax, ty2);
        if (tmin > tmax) return false;
    }
    else {
        if (r.o.y < min.y || r.o.y > max.y) return false;
    }

    // --- Z 轴 ---
    if (r.d.z != 0.0) {
        double tz1 = (min.z - r.o.z) / r.d.z;
        double tz2 = (max.z - r.o.z) / r.d.z;
        if (tz1 > tz2) std::swap(tz1, tz2);
        tmin = std::max(tmin, tz1);
        tmax = std::min(tmax, tz2);
        if (tmin > tmax) return false;
    }
    else {
        if (r.o.z < min.z || r.o.z > max.z) return false;
    }

    // 更新 t0 和 t1 为与包围盒相交的区间
    t0 = tmin;
    t1 = tmax;
    return true;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
