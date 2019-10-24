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
    if (!Intersect::box(origin, point, this->box.min, this->box.max))
      return false;

    Eigen::Affine3f shapeMatrix = mesh.getShapeModelMatrix();

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
    Eigen::Affine3f shapeMatrix = this->mesh.getShapeMatrix();

    float minf = std::numeric_limits<float>::min();
    float maxf = std::numeric_limits<float>::max();
    Eigen::Vector3f min = Eigen::Vector3f(maxf, maxf, maxf);
    Eigen::Vector3f max = Eigen::Vector3f(minf, minf, minf);

    int vertexCount = mesh.getNumberOfVertices();
    for (int i = 0; i < vertexCount; i++) {
      Eigen::Vector4f vertex = shapeMatrix * mesh.getVertex(i);
      vertex /= vertex.w();

      if (vertex(0) > max(0))
        max(0) = vertex(0);
      if (vertex(1) > max(1))
        max(1) = vertex(1);
      if (vertex(2) > max(2))
        max(2) = vertex(2);

      if (vertex(0) < min(0))
        min(0) = vertex(0);
      if (vertex(1) < min(1))
        min(1) = vertex(1);
      if (vertex(2) < min(2))
        min(2) = vertex(2);
    }

    this->box.min = min;
    this->box.max = max;
  }

private:
  Tucano::Mesh mesh;

  typedef struct BoundingBox {
    Eigen::Vector3f min;
    Eigen::Vector3f max;

    std::vector<int> faceIndices;

    BoundingBox *left;
    BoundingBox *right;
  } BoundingBox;
  BoundingBox box;
};
