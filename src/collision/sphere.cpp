#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
	Vector3D line = pm.position - origin;
	Vector3D tangent = origin + line.unit() * radius;
	Vector3D correction = (tangent - pm.last_position);
	if (radius >= line.norm()){
		pm.position = pm.last_position + (correction * (1 - double(friction)));
	}
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  Misc::draw_sphere(shader, origin, radius * 0.92);
}
