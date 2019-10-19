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

bool triangle(Eigen::Vector3f point, Eigen::Vector3f vertice1,
              Eigen::Vector3f vertice2, Eigen::Vector3f vertice3) {
  Eigen::Vector3f dirOne = vertice2 - vertice1;
  Eigen::Vector3f dirTwo = vertice3 - vertice1;

  Eigen::MatrixXf m(3, 2);
  m << dirOne.x(), dirTwo.x(), dirOne.y(), dirTwo.y(), dirOne.z(), dirTwo.z();

  Eigen::Vector2f result = m.colPivHouseholderQr().solve((point - vertice1));

  return (result.x() >= 0 && result.x() <= 1 && result.y() >= 0 &&
          result.y() <= 1 && (result.x() + result.y()) <= 1);
}

} // namespace Intersect
