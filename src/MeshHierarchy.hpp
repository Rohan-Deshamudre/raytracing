#pragma once

#include <limits>

#include "intersect.hpp"

class MeshHierarchy {
private:
	static const int MAX_DEPTH = 12;
	static const int MIN_TRIANGLES = 20;

	int facesChecked = 0;

	typedef struct BoundingBox {
		int level;

		Eigen::Vector3f min;
		Eigen::Vector3f max;

		std::vector<int>* faces;

		BoundingBox* less;
		BoundingBox* more;
	} BoundingBox;

	BoundingBox* box;

	void printBoundingBox(BoundingBox* box) {
		using namespace std;

		if (box->faces == nullptr || box->faces->empty())
			return;

		cout << "### BoundingBox " << endl;
		cout << "Level: " << box->level << endl;
		cout << box->min << box->max << endl;
		cout << box->faces->size() << endl << endl;

		if (box->less != nullptr)
			printBoundingBox(box->less);
		if (box->more != nullptr)
			printBoundingBox(box->more);
	}

	Tucano::Mesh mesh;

public:
	MeshHierarchy() {}
	MeshHierarchy(const Tucano::Mesh& mesh) {
		this->mesh = mesh;

		int num_faces = this->mesh.getNumberOfFaces();
		std::vector<int>* faces = new std::vector<int>();
		for (int i = 0; i < num_faces; i++) {
			faces->push_back(i);
		}

		box = new BoundingBox();
		buildUp(box, 0, faces);
		/* printBoundingBox(box); */
	}

	bool intersect(const Eigen::Vector3f& origin,
		const Eigen::Vector3f& point,
		Tucano::Face** outFace,
		Eigen::Vector3f** outIntersect)
	{
		const Eigen::Affine3f shapeMatrix = this->mesh.getShapeMatrix();

		if (!Intersect::box(origin, point, shapeMatrix * this->box->min, shapeMatrix * this->box->max))
			return false;

		return intersectBoundingBox(box, origin, point, outFace, outIntersect);
	}

	int getfacesChecked() {
		return facesChecked;
	}
	void setfacesChecked(int pFacesChecked) {
		facesChecked = pFacesChecked;
	}

protected:
	bool intersectBoundingBox(BoundingBox* box,
		const Eigen::Vector3f& origin, const Eigen::Vector3f& point,
		Tucano::Face** outFace, Eigen::Vector3f** outIntersect)
	{
		const Eigen::Affine3f shapeMatrix = this->mesh.getShapeMatrix();

		if (box->less != nullptr && Intersect::box(origin, point, shapeMatrix * this->box->less->min, shapeMatrix * this->box->less->max)) {
			return intersectBoundingBox(box->less, origin, point, outFace, outIntersect);
		}
		if (box->more != nullptr && Intersect::box(origin, point, shapeMatrix * this->box->more->min, shapeMatrix * this->box->more->max)) {
			return intersectBoundingBox(box->more, origin, point, outFace, outIntersect);
		}

		return intersectGeometry(this->box->faces, origin, point, outFace, outIntersect);
	}

	bool intersectGeometry(std::vector<int>* faces,
		const Eigen::Vector3f& origin,
		const Eigen::Vector3f& point,
		Tucano::Face** outFace,
		Eigen::Vector3f** outIntersect)
	{
		if (faces == nullptr || faces->empty())
			return false;

		const Eigen::Affine3f shapeMatrix = this->mesh.getShapeMatrix();

		Tucano::Face closestFace;
		Eigen::Vector3f closestIntersect;
		float minDist = std::numeric_limits<float>::max();

		facesChecked += faces->size();

		// Loop over all faces
		for (int i : *faces) {
			Eigen::Vector3f intersect;

			Tucano::Face face = mesh.getFace(i);
			// Assume a triangle
			Eigen::Vector4f vert1 = shapeMatrix * mesh.getVertex(face.vertex_ids[0]);
			Eigen::Vector4f vert2 = shapeMatrix * mesh.getVertex(face.vertex_ids[1]);
			Eigen::Vector4f vert3 = shapeMatrix * mesh.getVertex(face.vertex_ids[2]);

			// Intersect + set calculate distance
			if (Intersect::triangle(origin, point, vert1.head<3>() / vert1.w(),
				vert2.head<3>() / vert2.w(),
				vert3.head<3>() / vert3.w(), intersect)) {
				Eigen::Vector3f rayVector = intersect - origin;
				float dist = rayVector.norm();
				if (dist < minDist && rayVector.dot(point - origin) > 0.f) {
					minDist = dist;
					closestFace = face;
					closestIntersect = intersect;
				}
			}
		}

		if (minDist < std::numeric_limits<float>::max()) {
			*outFace = new Tucano::Face(closestFace);
			*outIntersect = new Eigen::Vector3f(closestIntersect);
			return true;
		}
		return false;
	}

	void buildUp(BoundingBox* box, int level, std::vector<int>* faces) {
		if (level >= MAX_DEPTH)
			return;

		box->level = level;
		box->faces = faces;

		constexpr float minf = std::numeric_limits<float>::min();
		constexpr float maxf = std::numeric_limits<float>::max();
		box->min = Eigen::Vector3f(maxf, maxf, maxf);
		box->max = Eigen::Vector3f(minf, minf, minf);

		// get box bounds
		for (int i : *faces) {
			Tucano::Face face = mesh.getFace(i);
			// Assume a triangle
			Eigen::Vector4f vert1 = mesh.getVertex(face.vertex_ids[0]);
			Eigen::Vector4f vert2 = mesh.getVertex(face.vertex_ids[1]);
			Eigen::Vector4f vert3 = mesh.getVertex(face.vertex_ids[2]);

			updateMinMax(&(box->min), &(box->max), &vert1);
			updateMinMax(&(box->min), &(box->max), &vert2);
			updateMinMax(&(box->min), &(box->max), &vert3);
		}

		// Don't split if reached min limit
		if (box->faces->size() <= MIN_TRIANGLES)
			return;

		// get split axis
		float xspan = fabs(box->min.x() - box->max.x());
		float yspan = fabs(box->min.y() - box->max.y());
		float zspan = fabs(box->min.z() - box->max.z());
		float span = std::max(xspan, std::max(yspan, zspan));

		Eigen::Vector3f mid = (box->max + box->min) * 0.5f;

		std::vector<int>* less = new std::vector<int>();
		std::vector<int>* more = new std::vector<int>();

		if (xspan == span) {
			splitByAxis(faces, mid, Eigen::Vector3f(1.0, 0.0, 0.0), less, more);
		}
		else if (yspan == span) {
			splitByAxis(faces, mid, Eigen::Vector3f(0.0, 1.0, 0.0), less, more);
		}
		else if (zspan == span) {
			splitByAxis(faces, mid, Eigen::Vector3f(0.0, 0.0, 1.0), less, more);
		}

		// recurse if has geometry
		if (!less->empty()) {
			box->less = new BoundingBox();
			buildUp(box->less, level + 1, less);
		}
		if (!more->empty()) {
			box->more = new BoundingBox();
			buildUp(box->more, level + 1, more);
		}
	}

private:
	void splitByAxis(std::vector<int>* faces, const Eigen::Vector3f& mid,
		const Eigen::Vector3f& normal, std::vector<int>* less, std::vector<int>* more) {
		using namespace Eigen;

		for (int i : *faces) {
			Tucano::Face face = mesh.getFace(i);
			// Assume a triangle
			Vector4f vert1 = mesh.getVertex(face.vertex_ids[0]);
			Vector4f vert2 = mesh.getVertex(face.vertex_ids[1]);
			Vector4f vert3 = mesh.getVertex(face.vertex_ids[2]);

			Vector3f m1 = vert1.head<3>() - mid;
			Vector3f m2 = vert2.head<3>() - mid;
			Vector3f m3 = vert3.head<3>() - mid;

			float dot1 = m1.dot(normal);
			float dot2 = m2.dot(normal);
			float dot3 = m3.dot(normal);

			if (dot1 < 0.f && dot2 < 0.f && dot3 < 0.f) {
				less->push_back(i);
			}
			else if (dot1 > 0.f && dot2 > 0.f && dot3 > 0.f) {
				more->push_back(i);
			}
			else {
				less->push_back(i);
				more->push_back(i);
			}
		}
	}

	void updateMinMax(Eigen::Vector3f* min, Eigen::Vector3f* max,
		Eigen::Vector4f* vertex) {
		if (vertex->x() > max->x())
			(*max)(0) = vertex->x();
		if (vertex->y() > max->y())
			(*max)(1) = vertex->y();
		if (vertex->z() > max->z())
			(*max)(2) = vertex->z();

		if (vertex->x() < min->x())
			(*min)(0) = vertex->x();
		if (vertex->y() < min->y())
			(*min)(1) = vertex->y();
		if (vertex->z() < min->z())
			(*min)(2) = vertex->z();
	}
};

