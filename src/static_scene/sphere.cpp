#include "sphere.h"

#include <cmath>

#include "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 {
namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double a = dot(r.d, r.d);
  Vector3D v = r.o - this->o;
  double b = 2 * dot(r.d, v);
  double c = dot(v, v) - this->r * this->r;
  double delta = b * b - 4 * a * c;
  if (delta < 0) return false;

  double sqrt_delta = sqrt(delta);
  t1 = (-b - sqrt_delta) / (2 * a);
  t2 = (-b + sqrt_delta) / (2 * a);
  return true;
}

bool Sphere::intersect(const Ray& r) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  if(!test(r, t1, t2)) return false;
  if (t1 > r.max_t) return false;

  if (t1 < r.min_t)
  {
    if (t2 < r.min_t || t2 > r.max_t) return false;
    r.max_t = t2;
  }
  else{
    r.max_t = t1;
  }

  return true;
}

bool Sphere::intersect(const Ray& r, Intersection* isect) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2;
  if(!test(r, t1, t2)) return false;
  if (t1 > r.max_t) return false;

  if (t1 < r.min_t)
  {
    if (t2 < r.min_t || t2 > r.max_t) return false;
    r.max_t = t2;
  }
  else{
    r.max_t = t1;
  }

  isect->t = r.max_t;
  Vector3D normal = (r.o + r.max_t * r.d) - this->o;
  isect->n = normal / this->r;
  isect->primitive = this;
  isect->bsdf = get_bsdf();
  return true;
}

void Sphere::draw(const Color& c) const { Misc::draw_sphere_opengl(o, r, c); }

void Sphere::drawOutline(const Color& c) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

}  // namespace StaticScene
}  // namespace CMU462
