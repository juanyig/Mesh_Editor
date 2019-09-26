#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 {
namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, vector<size_t>& v) : mesh(mesh), v(v) {}
Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3)
    : mesh(mesh), v1(v1), v2(v2), v3(v3) {}

BBox Triangle::get_bbox() const {
  // TODO (PathTracer):
  // compute the bounding box of the triangle
  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  double min_x, min_y, min_z, max_x, max_y, max_z;
  min_x = (p0.x < p1.x) ? ((p0.x < p2.x) ? p0.x : p2.x) : ((p1.x < p2.x) ? p1.x : p2.x);
  min_y = (p0.y < p1.y) ? ((p0.y < p2.y) ? p0.y : p2.y) : ((p1.y < p2.y) ? p1.y : p2.y);
  min_z = (p0.z < p1.z) ? ((p0.z < p2.z) ? p0.z : p2.z) : ((p1.z < p2.z) ? p1.z : p2.z);

  max_x = (p0.x > p1.x) ? ((p0.x > p2.x) ? p0.x : p2.x) : ((p1.x > p2.x) ? p1.x : p2.x);
  max_y = (p0.y > p1.y) ? ((p0.y > p2.y) ? p0.y : p2.y) : ((p1.y > p2.y) ? p1.y : p2.y);
  max_z = (p0.z > p1.z) ? ((p0.z > p2.z) ? p0.z : p2.z) : ((p1.z > p2.z) ? p1.z : p2.z);

  return BBox(Vector3D(min_x, min_y, min_z), Vector3D(max_x, max_y, max_z));
}

bool Triangle::intersect(const Ray& r) const {
  // TODO (PathTracer): implement ray-triangle intersection

  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  Vector3D e1 = p1 - p0;
  Vector3D e2 = p2 - p0;
  Vector3D s = r.o - p0;

  Vector3D e1_d = cross(e1, r.d);
  if (dot(e1_d, e2) == 0) return false;

  double coeff = 1.0 / dot(e1_d, e2);
  double u = coeff * dot(r.d, cross(e2, s));
  double v = coeff * dot(s, e1_d);
  double t = coeff * dot(e1, cross(e2, s));

  if (u < 0 || u > 1) return false;
  if (v < 0 || v > 1) return false;
  if ((u + v) > 1) return false;
  if (t < r.min_t || t > r.max_t) return false;
  r.max_t = t;

  return true;
}

bool Triangle::intersect(const Ray& r, Intersection* isect) const {
  // TODO (PathTracer):
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  Vector3D e1 = p1 - p0;
  Vector3D e2 = p2 - p0;
  Vector3D s = r.o - p0;

  Vector3D e1_d = cross(e1, r.d);
  if (dot(e1_d, e2) == 0) return false;

  double coeff = 1.0 / dot(e1_d, e2);
  double u = coeff * dot(r.d, cross(e2, s));
  double v = coeff * dot(s, e1_d);
  double t = coeff * dot(e1, cross(e2, s));

  if (u < 0 || u > 1) return false;
  if (v < 0 || v > 1) return false;
  if ((u + v) > 1) return false;
  if (t < r.min_t || t > r.max_t) return false;
  r.max_t = t;

  Vector3D n0 = mesh->normals[v1];
  Vector3D n1 = mesh->normals[v2];
  Vector3D n2 = mesh->normals[v3];
  Vector3D n = (1 - u - v) * n0 + u * n1 + v * n2;

  isect->t = t;
  isect->primitive = this;
  isect->n = (dot(n,r.d) < 0)? n : -n;
  isect->bsdf = mesh->get_bsdf();
  return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

}  // namespace StaticScene
}  // namespace CMU462
