#pragma once

namespace Intersect {

bool plane(Eigen::Vector3f rayOrigin, Eigen::Vector3f rayDirection,
           Eigen::Vector3f planeNormal, Eigen::Vector3f planePoint,
           Eigen::Vector3f &intersect) {
  planeNormal = planeNormal.normalized();
  float d = planeNormal.dot(planePoint);

  float t = (d - planeNormal.dot(rayOrigin));
  float dd = rayDirection.dot(planeNormal);

  if (dd == 0) {
    return false;
  } else {
    intersect = rayOrigin + (t / dd) * rayDirection;
    return true;
  }
}

bool triangle(Eigen::Vector3f rayOrigin, Eigen::Vector3f rayDirection,
              Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c) {
  Eigen::Vector3f ab = a - b;
  Eigen::Vector3f ac = a - c;

  Eigen::Vector3f planeNormal = ab.cross(ac);

  Eigen::Vector3f trianglePlaneIntersect;
  if (plane(rayOrigin, rayDirection, planeNormal, a, trianglePlaneIntersect)) {
    // check if the trianglePlaneIntersect is inside the triangle

    Eigen::MatrixXf m(3, 2);
    m << ab.x(), ac.x(), ab.y(), ac.y(), ab.z(), ac.z();

    Eigen::Vector2f barycentric = m.colPivHouseholderQr().solve(trianglePlaneIntersect - a);

    float alpha = barycentric.x();
    float beta = barycentric.y();

    if (alpha < 0)
      return false;
    else if (beta < 0)
      return false;
    else if (alpha + beta > 1)
      return false;
    else
      return true;
  }
  else {
    return false;
  }
}

} // namespace Intersect
