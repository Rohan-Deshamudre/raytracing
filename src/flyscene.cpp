#include "flyscene.hpp"
#include <GLFW/glfw3.h>

#include <algorithm>
#include <limits>
#include <thread>

#include "intersect.hpp"
#include "helper.hpp"

#include "MeshHierarchy.hpp"

void Flyscene::initialize(int width, int height) {
  // initiliaze the Phong Shading effect for the Opengl Previewer
  phong.initialize();

  // set the camera's projection matrix
  flycamera.setPerspectiveMatrix(60.0, width / (float)height, 0.1f, 100.0f);
  flycamera.setViewport(Eigen::Vector2f((float)width, (float)height));

  // load the OBJ file and materials
  Tucano::MeshImporter::loadObjFile(mesh, materials,
                                    "resources/models/torus2.obj");
  // normalize the model (scale to unit cube and center at origin)
  mesh.normalizeModelMatrix();
  // create mesh hierarchy
  this->meshHierarchy = MeshHierarchy(mesh);

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
}

void Flyscene::paintGL(void) {

  // update the camera view matrix with the last mouse interactions
  flycamera.updateViewMatrix();
  Eigen::Vector4f viewport = flycamera.getViewport();

  // clear the screen and set background color
  glClearColor(0.95, 0.95, 0.95, 0.0);
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
  traceDebugRay(flycamera.getCenter(), flycamera.getCenter() + dir, 10);

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

  Tucano::Face *closestFace;
  Eigen::Vector3f *closestIntersect;

  // intersect with bounding box
  if (!meshHierarchy.intersect(origin, dest, &closestFace, &closestIntersect))
    return Eigen::Vector3f(0.95f, 0.95f, 0.95f);

  // Interpolate normal
  Eigen::Vector4f vert1 =
      shapeMatrix * mesh.getVertex(closestFace->vertex_ids[0]);
  Eigen::Vector4f vert2 =
      shapeMatrix * mesh.getVertex(closestFace->vertex_ids[1]);
  Eigen::Vector4f vert3 =
      shapeMatrix * mesh.getVertex(closestFace->vertex_ids[2]);
  Eigen::Vector2f barycentric = calculateBarycentric(
      vert1.head<3>() / vert1.w(), vert2.head<3>() / vert2.w(),
      vert3.head<3>() / vert3.w(), *closestIntersect);
  Eigen::Vector3f surfaceNormal = interpolate(
      mesh.getNormal(closestFace->vertex_ids[0]).normalized(),
      mesh.getNormal(closestFace->vertex_ids[1]).normalized(),
      mesh.getNormal(closestFace->vertex_ids[2]).normalized(), barycentric);
  surfaceNormal = (normalMatrix * surfaceNormal.normalized()).normalized();

  // Material properties
  auto material = materials[closestFace->material_id];
  Eigen::Vector3f kd = material.getDiffuse();
  Eigen::Vector3f ks = material.getSpecular();
  float shininess = material.getShininess();

  // calculate diffuse + specular illumination
  Eigen::Vector3f diffuse = Eigen::Vector3f(0.0, 0.0, 0.0);
  Eigen::Vector3f specular = Eigen::Vector3f(0.0, 0.0, 0.0);
  for (auto light : lights) {
    Eigen::Vector3f lightColour = Eigen::Vector3f(1.0, 1.0, 1.0);

    Eigen::Vector3f rayVector = *closestIntersect - origin;

    // check if in shadow
    if (!lightBlocked(*closestFace, *closestIntersect - 0.001f * rayVector,
                      light)) {
      Eigen::Vector3f toLight = light - *closestIntersect;
      Eigen::Vector3f toLightUnit = toLight.normalized();
      float lightDistance = toLight.norm();
      Eigen::Vector3f reflectedLight = reflect(-toLightUnit, surfaceNormal);

      // if no hit on ray back to light -> illuminated
      diffuse += kd.cwiseProduct(lightColour) *
                 std::max(0.f, surfaceNormal.dot(toLightUnit)) / lightDistance;
      specular += ks.cwiseProduct(lightColour) *
                  pow(max(rayDirection.dot(-reflectedLight), 0.f), shininess);
    }
  }
  return diffuse + specular;
}

bool Flyscene::lightBlocked(const Tucano::Face &originFace,
                            Eigen::Vector3f origin, Eigen::Vector3f lightPos) {
  Eigen::Vector3f intersect;

  Eigen::Affine3f shapeMatrix = mesh.getShapeModelMatrix();

  int num_faces = mesh.getNumberOfFaces();
  for (int i = 0; i < num_faces; ++i) {
    Tucano::Face face = mesh.getFace(i);

    // a surface cannot cast shadow on itself
    if ((face.vertex_ids[0] == originFace.vertex_ids[0]) &&
        (face.vertex_ids[1] == originFace.vertex_ids[1]) &&
        (face.vertex_ids[2] == originFace.vertex_ids[2]))
      continue;

    // Assume a triangle
    Eigen::Vector4f vert1 = shapeMatrix * mesh.getVertex(face.vertex_ids[0]);
    Eigen::Vector4f vert2 = shapeMatrix * mesh.getVertex(face.vertex_ids[1]);
    Eigen::Vector4f vert3 = shapeMatrix * mesh.getVertex(face.vertex_ids[2]);

    // Intersect
    if (Intersect::triangle(origin, lightPos, vert1.head<3>() / vert1.w(),
                            vert2.head<3>() / vert2.w(),
                            vert3.head<3>() / vert3.w(), intersect)) {
      Eigen::Vector3f rayVector = intersect - origin;
      if (rayVector.dot(lightPos - origin) > 0.f)
        return true;
    }
  }

  return false;
}
