namespace Intersect {

using namespace Eigen;

inline bool plane(Vector3f rayOrigin, Vector3f rayPoint,
           Vector3f planeNormal, Vector3f planePoint,
           Vector3f &intersect) {

  planeNormal = planeNormal.normalized();
  Vector3f rayDirection = (rayPoint - rayOrigin).normalized();

  float d = planePoint.dot(-planeNormal);

  float t = -(d + planeNormal.dot(rayOrigin));
  float dd = rayDirection.dot(planeNormal);

  if (dd == 0) {
    return false;
  } else {
    intersect = rayOrigin + (t / dd) * rayDirection;
    return true;
  }
}

inline bool triangle(Vector3f rayOrigin, Vector3f rayPoint,
                     Vector3f a, Vector3f b, Vector3f c,
                     Vector3f &intersect) {

  Vector3f u = b - a;
  Vector3f v = c - a;

  Vector3f planeNormal = u.cross(v).normalized();

  if (plane(rayOrigin, rayPoint, planeNormal, a, intersect)) {
    // check if the intersect is inside the triangle
    float uu, uv, vv, wu, wv, D;
    uu = u.dot(u);
    uv = u.dot(v);
    vv = v.dot(v);
    Vector3f w = intersect - a;
    wu = w.dot(u);
    wv = w.dot(v);
    D = uv * uv - uu * vv;

    // test parametric coordinates
    float x, y;
    x = (uv * wv - vv * wu) / D;
    if (x < 0.0 || x > 1.0)
        return false;
    y = (uv * wu - uu * wv) / D;
    if (y < 0.0 || (x + y) > 1.0)
        return false;

    return true;                   // I is in T
  }
  else {
    return false;
  }
}

inline bool Box(const Vector3f &origin,const Vector3f &direction, Vector3f Bmin, Vector3f Bmax) {
	
	float xmin = (Bmin.x() - origin.x()) / direction.x();
	float xmax = (Bmax.x() - origin.x()) / direction.x();

	if (xmin > xmax) {
		swap(xmin, xmax);
	}

	float ymin = (Bmin.y() - origin.y()) / direction.y();
	float ymax = (Bmax.y() - origin.y()) / direction.y();

	if (ymin > ymax) {
		swap(ymin, ymax);
	}

	if ((xmin > ymax) || (ymin > xmax)) {
		return false;
	}

	if (ymin > xmin) {
		xmin = ymin;
	}	

	if (ymax < xmax) {
		xmax = ymax;
	}		

	float zmin = (Bmin.z() - origin.z()) / direction.z();
	float zmax = (Bmax.z() - origin.z()) / direction.z();

	if (zmin > zmax) {
		swap(zmin, zmax);
	}

	if ((xmin > zmax) || (zmin > xmax)) {
		return false;
	}

	if (zmin > xmin){
		xmin = zmin;
	}

	if (zmax < xmax) {
		xmax = zmax;
	}

	return true;	

}
} // namespace Intersect
