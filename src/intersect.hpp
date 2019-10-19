#pragma once

namespace Intersect {

bool plane(Eigen::Vector3f &origin, Eigen::Vector3f dir, Eigen::Vector3f norm,
           Eigen::Vector3f point, Eigen::Vector3f &intersect) {
  Eigen::Vector3f normalizedNorm = norm.normalized();
  float D = normalizedNorm.dot(point);

  float t = (D - normalizedNorm.dot(origin));
  float dd = dir.dot(normalizedNorm);

  if (dd == 0) {
    return false;
  } else {
    intersect = origin + (t / dd) * dir;
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
