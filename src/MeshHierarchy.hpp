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
                 const Eigen::Vector3f &direction) {
    return Intersect::box(origin, direction, this->box.min, this->box.max);
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
