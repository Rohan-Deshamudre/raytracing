#include "flyscene.hpp"
#include <GLFW/glfw3.h>

#include <algorithm>
#include <limits>
#include <thread>

#include "intersect.hpp"

float clamp(float v, float min, float max) {
  if (v < min)
    return min;
  if (v > max)
    return max;
  return v;
}

// Reflect ray according to normal
Eigen::Vector3f reflect(const Eigen::Vector3f &v, const Eigen::Vector3f &n) {
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

void Flyscene::initialize(int width, int height) {
  // initiliaze the Phong Shading effect for the Opengl Previewer
  phong.initialize();

  // set the camera's projection matrix
  flycamera.setPerspectiveMatrix(60.0, width / (float)height, 0.1f, 100.0f);
  flycamera.setViewport(Eigen::Vector2f((float)width, (float)height));

  // load the OBJ file and materials
  Tucano::MeshImporter::loadObjFile(mesh, materials,
                                    "resources/models/toy.obj");

  // normalize the model (scale to unit cube and center at origin)
  mesh.normalizeModelMatrix();

  // pass all the materials to the Phong Shader
  for (int i = 0; i < materials.size(); ++i)
    phong.addMaterial(materials[i]);

  // set the color and size of the sphere to represent the light sources
  // same sphere is used for all sources
  lightrep.setColor(Eigen::Vector4f(1.0, 1.0, 0.0, 1.0));
  lightrep.setSize(0.15);

  // create a first ray-tracing light source at some random position
  lights.push_back(Eigen::Vector3f(-1.0, 1.0, 1.0));

  // scale the camera representation (frustum) for the ray debug
  camerarep.shapeMatrix()->scale(0.2);

  // craete a first debug ray pointing at the center of the screen
  createDebugRay(Eigen::Vector2f(width / 2.0, height / 2.0));

  glEnable(GL_DEPTH_TEST);

  

  Flyscene::initTheBoundingBox(mesh);


  /* for (int i = 0; i<mesh.getNumberOfFaces(); ++i){ */
  /*   Tucano::Face face = mesh.getFace(i); */
  /*   for (int j =0; j<face.vertex_ids.size(); ++j){ */
  /*     std::cout<<"vid "<<j<<" "<<face.vertex_ids[j]<<std::endl; */
  /*     std::cout<<"vertex"<<mesh.getVertex(face.vertex_ids[j]).transpose()<<std::endl;
   */
  /*     std::cout<<"normal"<<mesh.getNormal(face.vertex_ids[j]).transpose()<<std::endl;
   */
  /*   } */
  /*   std::cout<<"mat id "<<face.material_id<<std::endl<<std::endl; */
  /*   std::cout<<"face normal "<<face.normal.transpose() << std::endl << */
  /*   std::endl; */
  /* } */
}

void Flyscene::initTheBoundingBox(Tucano::Mesh mesh) {
	Eigen::Vector4f static min = mesh.getVertex(0);
	Eigen::Vector4f static max = mesh.getVertex(0);

	for (int i = 0; i < mesh.getNumberOfVertices(); ++i) {
		Eigen::Vector4f current = mesh.getVertex(i);
		for (int j = 0; j < 3; j++) {
			if (current[j] < min[j]) {
				min[j] = current[j];

			}
		}

		for (int j = 0; j < 3; j++) {
			if (current[j] > max[j]) {
				max[j] = current[j];

			}
		}
	}

	minPoint = min;
	maxPoint = max;

	std::cout << "This is the first bounding box: " << std::endl;
	std::cout << "The min value: " << std::endl << minPoint << std::endl;
	std::cout << "The max value: " << std::endl << maxPoint << std::endl;
	//to here we have the main big box 


	Eigen::Vector4f maxLeft;
	Eigen::Vector4f minRight;

	//Split according the longest axis.
	float xDif = maxPoint[0] - minPoint[0];
	float yDif = maxPoint[1] - minPoint[1];
	float zDif = maxPoint[2] - minPoint[2];
	if (xDif == std::max(xDif, yDif) && xDif == std::max(xDif, zDif)){

		//x is the longest axis.
		minRight[0] = minPoint[0] + ((maxPoint[0] - minPoint[0]) / 2);
		minRight[1] = minPoint[1];
		minRight[2] = minPoint[2];


		maxLeft[0] = minPoint[0] + ((maxPoint[0] - minPoint[0]) / 2);
		maxLeft[1] = maxPoint[1];
		maxLeft[2] = maxPoint[2];


	}
	else if (yDif == std::max(xDif, yDif) && yDif == std::max(yDif, zDif)) {

		//y is the longest axis.
		minRight[0] = minPoint[0];
		minRight[1] = minPoint[1] + ((maxPoint[1] - minPoint[1]) / 2);
		minRight[2] = minPoint[2];


		maxLeft[0] = maxPoint[0];
		maxLeft[1] = minPoint[1] + ((maxPoint[1] - minPoint[1]) / 2);
		maxLeft[2] = maxPoint[2];


	}
	else if (zDif == std::max(xDif, zDif) && zDif == std::max(yDif, zDif)) {
		
		//z is the longest axis.
		minRight[0] = minPoint[0];
		minRight[1] = minPoint[1];
		minRight[2] = minPoint[2] + ((maxPoint[2] - minPoint[2]) / 2);


		maxLeft[0] = maxPoint[0];
		maxLeft[1] = maxPoint[1];
		maxLeft[2] = minPoint[2] + ((maxPoint[2] - minPoint[2]) / 2);


	}
	//to here we have 2 boxes

	int num_faces = mesh.getNumberOfFaces();
	for (int i = 0; i < num_faces; ++i) {
		Tucano::Face face = mesh.getFace(i);
	
		if (hasVertexInBox(i, minPoint, maxLeft) && hasVertexInBox(i, minRight, maxPoint)) {//in both boxes
			
		}
		else if (hasVertexInBox(i, minPoint, maxLeft)) {//in left box
			
		}
		else if (hasVertexInBox(i, minRight, maxPoint)) {//in right box
			
		}

		
	}
}

bool Flyscene::hasVertexInBox(int triangleIndex, Eigen::Vector3f _boxMin, Eigen::Vector3f _boxMax) {
	Tucano::Face face = mesh.getFace(triangleIndex);
	for (int i = 0; i < 3; i++) {
		if (isVertexInBox(mesh.getVertex[face.vertex_ids.at(i)], _boxMin, _boxMax)) {
			return true;
		}
	}
	return false;
}

bool Flyscene::isVertexInBox(int triangleIndex, Eigen::Vector3f _boxMin, Eigen::Vector3f _boxMax) {
	return mesh.getVertex(triangleIndex)[0] >= _boxMin[0] && mesh.getVertex(triangleIndex)[0] <= _boxMax[0] &&
		mesh.getVertex(triangleIndex)[1] >= _boxMin[1] && mesh.getVertex(triangleIndex)[1] <= _boxMax[1] &&
		mesh.getVertex(triangleIndex)[2] >= _boxMin[2] && mesh.getVertex(triangleIndex)[2] <= _boxMax[2];
}


void Flyscene::paintGL(void) {

  // update the camera view matrix with the last mouse interactions
  flycamera.updateViewMatrix();
  Eigen::Vector4f viewport = flycamera.getViewport();

  // clear the screen and set background color
  glClearColor(0.9, 0.9, 0.9, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // position the scene light at the last ray-tracing light source
  scene_light.resetViewMatrix();
  scene_light.viewMatrix()->translate(-lights.back());

  // render the scene using OpenGL and one light source
  phong.render(mesh, flycamera, scene_light);

  // render the ray and camera representation for ray debug
  for (std::vector<Tucano::Shapes::Cylinder>::iterator it = debugRays.begin();
       it != debugRays.end(); ++it) {
    Tucano::Shapes::Cylinder temp = *it;
    temp.render(flycamera, scene_light);
  }

  camerarep.render(flycamera, scene_light);

  // render ray tracing light sources as yellow spheres
  for (int i = 0; i < lights.size(); ++i) {
    lightrep.resetModelMatrix();
    lightrep.modelMatrix()->translate(lights[i]);
    lightrep.render(flycamera, scene_light);
  }

  // render coordinate system at lower right corner
  flycamera.renderAtCorner();
}

void Flyscene::simulate(GLFWwindow *window) {
  // Update the camera.
  // NOTE(mickvangelderen): GLFW 3.2 has a problem on ubuntu where some key
  // events are repeated: https://github.com/glfw/glfw/issues/747. Sucks.
  float dx = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ? 1.0 : 0.0) -
             (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ? 1.0 : 0.0);
  float dy = (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS
                  ? 1.0
                  : 0.0) -
             (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS
                  ? 1.0
                  : 0.0);
  float dz = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ? 1.0 : 0.0) -
             (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ? 1.0 : 0.0);
  flycamera.translate(dx, dy, dz);
}

void Flyscene::traceDebugRay(Eigen::Vector3f from, Eigen::Vector3f to,
                             int maxReflections) {

  std::cout << "Reflection: " << maxReflections << std::endl;

  Eigen::Affine3f shapeMatrix = mesh.getShapeModelMatrix();
  Eigen::MatrixXf normalMatrix = shapeMatrix.linear().inverse().transpose();
  Eigen::Vector3f rayDirection = (to - from).normalized();

  Tucano::Face closestFace;
  Eigen::Vector3f closestIntersect;
  float minDist = std::numeric_limits<float>::max();

  Eigen::Vector3f intersect;

  // Loop over all faces
  int num_faces = mesh.getNumberOfFaces();
  for (int i = 0; i < num_faces; ++i) {
    Tucano::Face face = mesh.getFace(i);

    // Assume a triangle
    Eigen::Vector4f vert1 = shapeMatrix * mesh.getVertex(face.vertex_ids[0]);
    Eigen::Vector4f vert2 = shapeMatrix * mesh.getVertex(face.vertex_ids[1]);
    Eigen::Vector4f vert3 = shapeMatrix * mesh.getVertex(face.vertex_ids[2]);

    // Intersect + set calculate distance
    if (Intersect::triangle(from, to, vert1.head<3>() / vert1.w(),
                            vert2.head<3>() / vert2.w(),
                            vert3.head<3>() / vert3.w(), intersect)) {
      Eigen::Vector3f distVector = intersect - from;
      float dist = distVector.norm();
      if (dist < minDist && distVector.dot(to - from) > 0.f) {
        minDist = dist;
        closestFace = face;
        closestIntersect = intersect;
      }
    }
  }

  if (minDist < std::numeric_limits<float>::max()) {
    // intersection

    // calculating reflection
    Eigen::Vector3f reflectDir = reflect(rayDirection, closestFace.normal);
    float length = 3; // should be minDist
    Tucano::Shapes::Cylinder ray =
        Tucano::Shapes::Cylinder(0.01, minDist, 16, 64);

    std::cout << minDist << std::endl;
    ray.resetModelMatrix();
    ray.setOriginOrientation(from, rayDirection);
    debugRays.push_back(ray);
    if (maxReflections > 1) {
      traceDebugRay(closestIntersect + reflectDir * 0.001f,
                    closestIntersect + reflectDir, maxReflections - 1);
    }
  } else {
    // no intersection
    Tucano::Shapes::Cylinder ray = Tucano::Shapes::Cylinder(0.01, 42, 16, 64);
    ray.resetModelMatrix();
    ray.setOriginOrientation(from, rayDirection);
    debugRays.push_back(ray);
  }
}

void Flyscene::createDebugRay(const Eigen::Vector2f &mouse_pos) {
  debugRays.clear();

  // from pixel position to world coordinates
  Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouse_pos);

  // direction from camera center to click position
  Eigen::Vector3f dir = (screen_pos - flycamera.getCenter()).normalized();

  // position and orient the cylinder representing the ray
  traceDebugRay(flycamera.getCenter(), flycamera.getCenter() + dir, 2);

  // place the camera representation (frustum) on current camera location,
  camerarep.resetModelMatrix();
  camerarep.setModelMatrix(flycamera.getViewMatrix().inverse());
}

void Flyscene::raytraceScene(int width, int height) {
  std::cout << "ray tracing ..." << std::endl;

  // if no width or height passed, use dimensions of current viewport
  Eigen::Vector2i image_size(width, height);
  if (width == 0 || height == 0) {
    image_size = flycamera.getViewportSize();
  }

  // create 2d vector to hold pixel colors and resize to match image size
  vector<vector<Eigen::Vector3f>> pixel_data;
  pixel_data.resize(image_size[1]);
  for (int i = 0; i < image_size[1]; ++i)
    pixel_data[i].resize(image_size[0]);

  // check number of supported concurrent threads
  unsigned int threads = std::thread::hardware_concurrency();
  std::cout << threads << " concurrent threads are supported.\n";

  if (threads == 0) {
    // threading not supported
    std::cout << "Using single thread." << std::endl;
    raytracePartScene(pixel_data, image_size[1], image_size[0], 0,
                      image_size[1]);
  } else {
    // multithread for maximal power levels (over 9000!)
    std::cout << "Using " << threads << " threads." << std::endl;

    // split over threads
    std::vector<std::thread> workers(threads);
    for (int i = 0; i < threads; i++) {
      int x_start = i * (image_size[1] / threads);
      int x_end = x_start + (image_size[1] / threads);

      std::cout << "Starting thread " << i << " of " << threads << std::endl;
      workers[i] =
          std::thread(&Flyscene::raytracePartScene, this, std::ref(pixel_data),
                      image_size[1], image_size[0], x_start, x_end);
    }

    // wait for threads to finish
    for (auto &t : workers) {
      t.join();
      std::cout << "Thread finished." << std::endl;
    }
  }

  // write the ray tracing result to a PPM image
  Tucano::ImageImporter::writePPMImage("result.ppm", pixel_data);
  std::cout << "ray tracing done! " << std::endl;
}

void Flyscene::raytracePartScene(vector<vector<Eigen::Vector3f>> &pixel_data,
                                 int width, int height, int x_start,
                                 int x_end) {
  // origin of the ray is always the camera center
  Eigen::Vector3f origin = flycamera.getCenter();
  Eigen::Vector3f screen_coords;

  // for every pixel shoot a ray from the origin through the pixel coords
  for (int j = x_start; j < x_end; ++j) {
    for (int i = 0; i < height; ++i) {
      // create a ray from the camera passing through the pixel (i,j)
      screen_coords = flycamera.screenToWorld(Eigen::Vector2f(i, j));
      // launch raytracing for the given ray and write result to pixel data
      Eigen::Vector3f raw = traceRay(origin, screen_coords);

      // gamma 2 correction
      pixel_data[i][j] = Eigen::Vector3f(sqrt(clamp(raw(0), 0.f, 1.f)),
                                         sqrt(clamp(raw(1), 0.f, 1.f)),
                                         sqrt(clamp(raw(2), 0.f, 1.f)));
    }
  }
}

Eigen::Vector3f Flyscene::traceRay(Eigen::Vector3f &origin,
                                   Eigen::Vector3f &dest) {
  Eigen::Affine3f shapeMatrix = mesh.getShapeModelMatrix();
  Eigen::MatrixXf normalMatrix = shapeMatrix.linear().inverse().transpose();
  Eigen::Vector3f rayDirection = (dest - origin).normalized();

  Tucano::Face closestFace;
  Eigen::Vector3f closestIntersect;
  float minDist = std::numeric_limits<float>::max();

  Eigen::Vector3f intersect;

  // Loop over all faces
  int num_faces = mesh.getNumberOfFaces();
  for (int i = 0; i < num_faces; ++i) {
    Tucano::Face face = mesh.getFace(i);

    // Assume a triangle
    Eigen::Vector4f vert1 = shapeMatrix * mesh.getVertex(face.vertex_ids[0]);
    Eigen::Vector4f vert2 = shapeMatrix * mesh.getVertex(face.vertex_ids[1]);
    Eigen::Vector4f vert3 = shapeMatrix * mesh.getVertex(face.vertex_ids[2]);

    // Intersect + set calculate distance
    if (Intersect::triangle(origin, dest, vert1.head<3>() / vert1.w(),
                            vert2.head<3>() / vert2.w(),
                            vert3.head<3>() / vert3.w(), intersect)) {
      Eigen::Vector3f distVector = intersect - origin;
      float dist = distVector.norm();
      if (dist < minDist && distVector.dot(dest - origin) > 0.f) {
        minDist = dist;
        closestFace = face;
        closestIntersect = intersect;
      }
    }
  }

  if (minDist < std::numeric_limits<float>::max()) {
    auto material = materials[closestFace.material_id];
    Eigen::Vector3f kd = material.getDiffuse();
    Eigen::Vector3f ks = material.getSpecular();
    float shininess = material.getShininess();

    // Interpolate normal
    Eigen::Vector4f vert1 =
        shapeMatrix * mesh.getVertex(closestFace.vertex_ids[0]);
    Eigen::Vector4f vert2 =
        shapeMatrix * mesh.getVertex(closestFace.vertex_ids[1]);
    Eigen::Vector4f vert3 =
        shapeMatrix * mesh.getVertex(closestFace.vertex_ids[2]);
    Eigen::Vector2f barycentric = calculateBarycentric(
        vert1.head<3>() / vert1.w(), vert2.head<3>() / vert2.w(),
        vert3.head<3>() / vert3.w(), closestIntersect);

    Eigen::Vector3f surfaceNormal = interpolate(
        mesh.getNormal(closestFace.vertex_ids[0]).normalized(),
        mesh.getNormal(closestFace.vertex_ids[1]).normalized(),
        mesh.getNormal(closestFace.vertex_ids[2]).normalized(), barycentric);
    surfaceNormal = (normalMatrix * surfaceNormal.normalized()).normalized();

    // calculate diffuse + specular illumination
    Eigen::Vector3f diffuse = Eigen::Vector3f(0.0, 0.0, 0.0);
    Eigen::Vector3f specular = Eigen::Vector3f(0.0, 0.0, 0.0);

    for (auto light : lights) {
      Eigen::Vector3f toLight = light - closestIntersect;
      Eigen::Vector3f toLightUnit = toLight.normalized();
      float lightDistance = toLight.norm();
      Eigen::Vector3f reflectedLight = reflect(-toLightUnit, surfaceNormal);

      // check hard shadow
      minDist = std::numeric_limits<float>::max();
      Eigen::Vector3f impactPoint = intersect;

      for (int i = 0; i < num_faces; ++i) {
        Tucano::Face face = mesh.getFace(i);

        // Assume a triangle
        Eigen::Vector4f vert1 =
            shapeMatrix * mesh.getVertex(face.vertex_ids[0]);
        Eigen::Vector4f vert2 =
            shapeMatrix * mesh.getVertex(face.vertex_ids[1]);
        Eigen::Vector4f vert3 =
            shapeMatrix * mesh.getVertex(face.vertex_ids[2]);

        // Intersect + set calculate distance
        if (Intersect::triangle(impactPoint, impactPoint + toLightUnit,
                                vert1.head<3>() / vert1.w(),
                                vert2.head<3>() / vert2.w(),
                                vert3.head<3>() / vert3.w(), intersect)) {
          Eigen::Vector3f distVector = intersect - impactPoint;
          float dist = distVector.norm();
          // check if closer and in the correct halfspace
          if (dist < minDist &&
              distVector.dot(impactPoint + toLightUnit) > 0.f) {
            minDist = dist;

            // Only need to know that such an intersect exists
            /* return Eigen::Vector3f(1.0, 0.0, 0.0); */
            break;
          }
        }
      }

      // if no hit on ray back to light -> illuminated
      if (minDist >= std::numeric_limits<float>::max()) {
        diffuse +=
            kd * std::max(0.f, surfaceNormal.dot(toLightUnit)) / lightDistance;
        specular +=
            ks * pow(max(rayDirection.dot(-reflectedLight), 0.f), shininess);
      }
    }

    /* return Eigen::Vector3f(0.0, 0.0, 1.0); */
    return diffuse + specular;
  }

  // no intersection
  return Eigen::Vector3f(0.9f, 0.9f, 0.9f);
}
