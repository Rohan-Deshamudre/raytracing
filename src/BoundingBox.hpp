#ifndef _BOUNDINGBOX_
#define _BOUNDINGBOX_


#include <tucano/mesh.hpp>
#include <vector>
#include <GLFW/glfw3.h>
#include "flyscene.hpp"
#include "Intersect.hpp"

////////////////

#define NUM_OF_FACES 500

class BoundingBox {

private:

	std::vector<int> faces;  // The faces in the box
	std::vector<BoundingBox> boxes;
	Eigen::Vector3f maxV;   // The max vertex
	Eigen::Vector3f minV;	// The min vertex
	Eigen::Matrix4f shapeModelMatrix;  // The shape model matrix
	Tucano::Mesh* mesh; // The mesh

public:

	BoundingBox(void) {	}

	BoundingBox(Tucano::Mesh pointerToMesh, Eigen::Matrix4f shapeMatrix, std::vector<int> _faces, Eigen::Vector3f _min, Eigen::Vector3f _max) {
		shapeModelMatrix = shapeMatrix;
		mesh = &pointerToMesh;

		maxV = _max;
		minV = _min;

		faces = _faces;

		if (faces.size() > NUM_OF_FACES) {
			

			Eigen::Vector3f maxLeft;
			Eigen::Vector3f minRight;


			completeBoxValues(maxLeft, minRight);

			std::vector<int> facesOfRightBox;
			std::vector<int> facesOfLeftBox;

			

			split(&facesOfLeftBox, &facesOfRightBox, minV, maxLeft, minRight, maxV);



			BoundingBox rightBox = BoundingBox(*mesh, shapeModelMatrix, facesOfRightBox, minRight, maxV);
			BoundingBox leftBox = BoundingBox(*mesh, shapeModelMatrix, facesOfLeftBox, minV, maxLeft);
			
			
			boxes.push_back(rightBox);
			boxes.push_back(leftBox);

		}
	}

	void completeBoxValues(Eigen::Vector3f maxLeft, Eigen::Vector3f minRight) {
	
		float xDif = maxV[0] - minV[0];
		float yDif = maxV[1] - minV[1];
		float zDif = maxV[2] - minV[2];
		if (xDif == std::max(xDif, yDif) && xDif == std::max(xDif, zDif)) {

			//x is the longest axis.
			minRight[0] = minV[0] + ((maxV[0] - minV[0]) / 2);
			minRight[1] = minV[1];
			minRight[2] = minV[2];


			maxLeft[0] = minV[0] + ((maxV[0] - minV[0]) / 2);
			maxLeft[1] = maxV[1];
			maxLeft[2] = maxV[2];


		}
		else if (yDif == std::max(xDif, yDif) && yDif == std::max(yDif, zDif)) {

			//y is the longest axis.
			minRight[0] = minV[0];
			minRight[1] = minV[1] + ((maxV[1] - minV[1]) / 2);
			minRight[2] = minV[2];


			maxLeft[0] = maxV[0];
			maxLeft[1] = minV[1] + ((maxV[1] - minV[1]) / 2);
			maxLeft[2] = maxV[2];


		}
		else if (zDif == std::max(xDif, zDif) && zDif == std::max(yDif, zDif)) {

			//z is the longest axis.
			minRight[0] = minV[0];
			minRight[1] = minV[1];
			minRight[2] = minV[2] + ((maxV[2] - minV[2]) / 2);


			maxLeft[0] = maxV[0];
			maxLeft[1] = maxV[1];
			maxLeft[2] = minV[2] + ((maxV[2] - minV[2]) / 2);


		}

	}
	void split(std::vector<int>* leftFaces, std::vector<int>* rightFaces, Eigen::Vector3f leftMin, Eigen::Vector3f maxLeft, Eigen::Vector3f minRight, Eigen::Vector3f rightMax) {
		std::vector<int> facesOfBothBoxes;

		
		//Place each triangle in the left, right or both boxes
		for (const int& face : faces) {

			bool isInRight = hasVertexInBox(face, minRight, rightMax);
			bool isInLeft = hasVertexInBox(face, leftMin, maxLeft);

			if (isInLeft && isInRight ) {
				facesOfBothBoxes.push_back(face);
			}else if (isInRight) {
				rightFaces->push_back(face);
			}else if (isInLeft) {
				leftFaces->push_back(face);
			}
			
		}
		faces = facesOfBothBoxes;

	}
	

	bool hasVertexInBox(int triangleIndex, Eigen::Vector3f _boxMin, Eigen::Vector3f _boxMax) {
		Tucano::Face face = mesh->getFace(triangleIndex);
		for (int i = 0; i < 3; i++) {

			if (isVertexInBox(face.vertex_ids[i], _boxMin, _boxMax)) {
				return true;
			}
		}
		return false;
	}

	bool isVertexInBox(int vertexID, Eigen::Vector3f _boxMin, Eigen::Vector3f _boxMax) {

		Eigen::Vector4f v = mesh->getShapeModelMatrix() * mesh->getVertex(vertexID);

		return v[0] >= _boxMin[0] && v[0] <= _boxMax[0] &&
			v[1] >= _boxMin[1] && v[1] <= _boxMax[1] &&
			v[2] >= _boxMin[2] && v[2] <= _boxMax[2];
	}


	std::vector<int> getIntersectedTriangles(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction) {
		std::vector<int> IntersectedTriangles;

		if (Intersect::box(origin, direction, minV, maxV)) {
			if (faces.size() > 0) {
				IntersectedTriangles.insert(IntersectedTriangles.end(), faces.begin(), faces.end());
			}

			if (boxes.size() > 0) {
				
				if (Intersect::box(origin, direction, boxes[0].minV, boxes[0].maxV)) {
					std::vector<int> _facesFromBox = boxes[0].getIntersectedTriangles(origin, direction);
					IntersectedTriangles.insert(IntersectedTriangles.end(), _facesFromBox.begin(), _facesFromBox.end());
				}
				

				if (Intersect::box(origin, direction, boxes[1].minV, boxes[1].maxV)) {
					std::vector<int> _facesFromBox = boxes[1].getIntersectedTriangles(origin, direction);
					IntersectedTriangles.insert(IntersectedTriangles.end(), _facesFromBox.begin(), _facesFromBox.end());
				}
			}
		}

		return IntersectedTriangles;
	}

};

#endif // BoundingBox_hpp 
