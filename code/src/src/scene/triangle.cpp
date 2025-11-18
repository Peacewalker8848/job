#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
	Vector3D e1 = p2 - p1;
	Vector3D e2 = p3 - p1;

	Vector3D s1 = cross(r.d, e2);
	double divisor = dot(s1, e1);
	if (fabs(divisor) < EPS_D) return false;  // ray parallel to triangle

	double inv_div = 1.0 / divisor;

	Vector3D s = r.o - p1;
	double b1 = dot(s1, s) * inv_div;  // barycentric coord for v2
	if (b1 < 0.0 || b1 > 1.0) return false;

	Vector3D s2 = cross(s, e1);
	double b2 = dot(s2, r.d) * inv_div;  // barycentric coord for v3
	if (b2 < 0.0 || b1 + b2 > 1.0) return false;

	double t = dot(s2, e2) * inv_div;
	if (t < r.min_t || t > r.max_t) return false;

	// 根据作业说明，需要把 max_t 更新为目前为止最近的交点
	Ray& ray = const_cast<Ray&>(r);
	ray.max_t = t;


  return true;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
	Vector3D e1 = p2 - p1;
	Vector3D e2 = p3 - p1;

	Vector3D s1 = cross(r.d, e2);
	double divisor = dot(s1, e1);
	if (fabs(divisor) < EPS_D) return false;  // 平行

	double inv_div = 1.0 / divisor;

	Vector3D s = r.o - p1;
	double b1 = dot(s1, s) * inv_div;
	if (b1 < 0.0 || b1 > 1.0) return false;

	Vector3D s2 = cross(s, e1);
	double b2 = dot(s2, r.d) * inv_div;
	if (b2 < 0.0 || b1 + b2 > 1.0) return false;

	double t = dot(s2, e2) * inv_div;
	if (t < r.min_t || t > r.max_t) return false;

	// 更新 ray 的 max_t，保证之后的求交只会找更近的
	Ray& ray = const_cast<Ray&>(r);
	ray.max_t = t;

	// 计算重心坐标
	double b0 = 1.0 - b1 - b2;

	// 用重心坐标插值法线并归一化
	Vector3D n = (b0 * n1 + b1 * n2 + b2 * n3).unit();

	// 填写 Intersection 结果
	isect->t = t;
	isect->n = n;
	isect->primitive = this;
	isect->bsdf = bsdf;
	isect->hit_p = r.o + r.d * isect->t;

  return true;


}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
