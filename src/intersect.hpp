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

} // namespace Intersect
