#pragma once

#include <limits>

namespace Intersect {

	using namespace Eigen;

	inline bool plane(Vector3f rayOrigin, Vector3f rayPoint,
		Vector3f planeNormal, Vector3f planePoint,
		Vector3f& intersect) {

		planeNormal = planeNormal.normalized();
		Vector3f rayDirection = (rayPoint - rayOrigin).normalized();

		float d = planePoint.dot(-planeNormal);

		float t = -(d + planeNormal.dot(rayOrigin));
		float dd = rayDirection.dot(planeNormal);

		if (dd == 0) {
			return false;
		}
		else {
			intersect = rayOrigin + (t / dd) * rayDirection;
			return true;
		}
	}

	inline bool triangle(Vector3f rayOrigin, Vector3f rayPoint,
		Vector3f a, Vector3f b, Vector3f c,
		Vector3f& intersect) {

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

			return true;
		}
		else {
			return false;
		}
	}

	inline bool box(const Eigen::Vector3f& origin, const Eigen::Vector3f& point,
		Vector3f bmin, Vector3f bmax) {
		using namespace Eigen;

		Vector3f direction = point - origin;

		Vector3f invD =
			Vector3f(1.f / direction.x(), 1.f / direction.y(), 1.f / direction.z());

		Vector3f t0s = (bmin - origin).cwiseProduct(invD);
		Vector3f t1s = (bmax - origin).cwiseProduct(invD);

		Vector3f tsmaller =
			Vector3f(std::min(t0s.x(), t1s.x()), std::min(t0s.y(), t1s.y()),
				std::min(t0s.z(), t1s.z()));
		Vector3f tbigger =
			Vector3f(std::max(t0s.x(), t1s.x()), std::max(t0s.y(), t1s.y()),
				std::max(t0s.z(), t1s.z()));

		float tmin = std::numeric_limits<float>::min();
		float tmax = std::numeric_limits<float>::max();
		tmin = std::max(tmin,
			std::max(tsmaller.x(), std::max(tsmaller.y(), tsmaller.z())));
		tmax =
			std::min(tmax, std::min(tbigger.x(), std::min(tbigger.y(), tbigger.z())));

		return (tmin < tmax);
	}
} // namespace Intersect