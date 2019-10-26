#pragma once

inline Eigen::Vector3f Flyscene::min(Eigen::Vector3f a, Eigen::Vector3f b) {
	double x = std::min(a.x(), b.x());
	double y = std::min(a.y(), b.y());
	double z = std::min(a.z(), b.z());

	return Eigen::Vector3f(x, y, z);
}

inline float clamp(float v, float min, float max) {
  if (v < min)
    return min;
  if (v > max)
    return max;
  return v;
}

// Reflect ray according to normal
inline Eigen::Vector3f reflect(const Eigen::Vector3f &v, const Eigen::Vector3f &n) {
  return v - 2 * v.dot(n) * n;
}

Eigen::Vector2f calculateBarycentric(const Eigen::Vector3f &a,
                                     const Eigen::Vector3f &b,
                                     const Eigen::Vector3f &c,
                                     const Eigen::Vector3f &p) {
  using namespace Eigen;

  Vector3f u = b - a;
  Vector3f v = c - a;
  Vector3f w = p - a;

  float uu, uv, vv, wu, wv, D;
  uu = u.dot(u);
  uv = u.dot(v);
  vv = v.dot(v);

  wu = w.dot(u);
  wv = w.dot(v);

  D = uv * uv - uu * vv;

  float x, y;
  x = (uv * wv - vv * wu) / D;
  y = (uv * wu - uu * wv) / D;

  return Vector2f(x, y);
}

Eigen::Vector3f interpolate(const Eigen::Vector3f &nx,
                            const Eigen::Vector3f &ny,
                            const Eigen::Vector3f &nz,
                            const Eigen::Vector2f &barycentric) {
  float x, y;
  x = barycentric(0);
  y = barycentric(1);

  return x * ny + y * nz + (1.f - x - y) * nx;
}

