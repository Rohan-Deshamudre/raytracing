#pragma once

#include <limits>

#include "intersect.hpp"

class MeshHierarchy {
public:
  MeshHierarchy() {}
  MeshHierarchy(const Tucano::Mesh &mesh) {
    this->mesh = mesh;
    buildUp();
  }

  bool intersect(const Eigen::Vector3f &origin,
                 const Eigen::Vector3f &point,
                 Tucano::Face **outFace,
                 Eigen::Vector3f **outIntersect) {

    Eigen::Affine3f shapeMatrix = this->mesh.getShapeMatrix();

    if (!Intersect::box(origin, point, shapeMatrix * this->box->min, shapeMatrix * this->box->max))
      return false;

    Tucano::Face closestFace;
    Eigen::Vector3f closestIntersect;
    float minDist = std::numeric_limits<float>::max();

    // Loop over all faces
    int num_faces = mesh.getNumberOfFaces();
    for (int i = 0; i < num_faces; ++i) {
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

protected:
  void buildUp() {
    box = new BoundingBox();

    constexpr float minf = std::numeric_limits<float>::min();
    constexpr float maxf = std::numeric_limits<float>::max();
    box->min = Eigen::Vector3f(maxf, maxf, maxf);
    box->max = Eigen::Vector3f(minf, minf, minf);

    int num_faces = mesh.getNumberOfFaces();
    for (int i = 0; i < num_faces; ++i) {
      Tucano::Face face = mesh.getFace(i);
      // Assume a triangle
      Eigen::Vector4f vert1 = mesh.getVertex(face.vertex_ids[0]);
      Eigen::Vector4f vert2 = mesh.getVertex(face.vertex_ids[1]);
      Eigen::Vector4f vert3 = mesh.getVertex(face.vertex_ids[2]);

      updateMinMax(&(box->min), &(box->max), &vert1);
      updateMinMax(&(box->min), &(box->max), &vert2);
      updateMinMax(&(box->min), &(box->max), &vert3);
    }
  }

private:
  void updateMinMax(Eigen::Vector3f *min, Eigen::Vector3f *max, Eigen::Vector4f *vertex) {
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

  Tucano::Mesh mesh;

  typedef struct BoundingBox {
    Eigen::Vector3f min;
    Eigen::Vector3f max;

    std::vector<int> faces;

    BoundingBox *left;
    BoundingBox *right;
  } BoundingBox;
  BoundingBox* box;
};
