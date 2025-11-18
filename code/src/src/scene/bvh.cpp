#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


    BBox bbox;
    for (auto p = start; p != end; ++p) {
        bbox.expand((*p)->get_bbox());
    }

    BVHNode* node = new BVHNode(bbox);

    size_t nPrims = end - start;

    // 2. 叶子节点条件：图元数量 <= max_leaf_size
    if (nPrims <= max_leaf_size) {
        node->start = start;
        node->end = end;
        node->l = node->r = nullptr;
        return node;
    }

    // 3. 计算质心包围盒（用于选择划分轴）
    BBox centroid_bbox;
    for (auto p = start; p != end; ++p) {
        centroid_bbox.expand((*p)->get_bbox().centroid());
    }

    Vector3D diag = centroid_bbox.max - centroid_bbox.min;

    // 如果质心都挤在一起，没法再分，就直接做叶子
    if (diag.x <= 0 && diag.y <= 0 && diag.z <= 0) {
        node->start = start;
        node->end = end;
        node->l = node->r = nullptr;
        return node;
    }

    // 4. 选择最长轴作为划分轴
    int axis = 0;
    if (diag.y > diag.x && diag.y >= diag.z) {
        axis = 1;
    }
    else if (diag.z > diag.x && diag.z >= diag.y) {
        axis = 2;
    }

    // 5. 取该轴方向上质心包围盒的中点作为划分平面
    double split_pos;
    if (axis == 0) {
        split_pos = 0.5 * (centroid_bbox.min.x + centroid_bbox.max.x);
    }
    else if (axis == 1) {
        split_pos = 0.5 * (centroid_bbox.min.y + centroid_bbox.max.y);
    }
    else {
        split_pos = 0.5 * (centroid_bbox.min.z + centroid_bbox.max.z);
    }

    // 6. 按质心在划分平面左/右两侧做一次 partition
    auto mid = std::partition(start, end, [&](Primitive* p) {
        Vector3D c = p->get_bbox().centroid();
        if (axis == 0) return c.x < split_pos;
        if (axis == 1) return c.y < split_pos;
        return c.z < split_pos;
        });

    // 7. 处理退化情况：所有图元都跑到同一边
    if (mid == start || mid == end) {
        mid = start + nPrims / 2;  // 简单按数量二分
    }

    // 8. 递归构建左右子树
    node->l = construct_bvh(start, mid, max_leaf_size);
    node->r = construct_bvh(mid, end, max_leaf_size);

    node->start = start;
    node->end = end;

    return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
    if (!node) return false;

    double t0 = ray.min_t;
    double t1 = ray.max_t;

    // 先和当前节点的 bbox 做一次快速剔除
    if (!node->bb.intersect(ray, t0, t1)) {
        return false;
    }

    // 叶子节点：直接遍历其中的所有 primitive
    if (node->isLeaf()) {
        bool hit = false;
        for (auto p = node->start; p != node->end; ++p) {
            if ((*p)->has_intersection(ray)) {
                hit = true;
            }
        }
        return hit;
    }

    // 内部节点：递归检查左右子树
    bool hit_left = has_intersection(ray, node->l);
    bool hit_right = has_intersection(ray, node->r);
    return hit_left || hit_right;
}

bool BVHAccel::has_intersection(const Ray& ray) const {
    if (!root) return false;
    return has_intersection(ray, root);

}

bool BVHAccel::intersect(const Ray& ray, Intersection* isect, BVHNode* node) const {

    // 1. BVH node bounding box test
    double t0 = ray.min_t, t1 = ray.max_t;
    if (!node->bb.intersect(ray, t0, t1)) {
        return false;
    }

    bool hit = false;

    // 2. If leaf → test all primitives in this leaf
    if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
            if ((*p)->intersect(ray, isect)) {
                hit = true;
            }
        }
    }
    else {
        // 3. Internal node → recursively test children
        bool hit_left = intersect(ray, isect, node->l);
        bool hit_right = intersect(ray, isect, node->r);
        hit = hit_left || hit_right;
    }

    return hit;
}

} // namespace SceneObjects
} // namespace CGL
