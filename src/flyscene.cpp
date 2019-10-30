#include "flyscene.hpp"
#include <GLFW/glfw3.h>

#include <algorithm>
#include <limits>
#include <thread>
#include <random>

#include "intersect.hpp"
#include "helper.hpp"

#include "MeshHierarchy.hpp"


void Flyscene::modifyDebugReflection(int change) {
	if (change > 0 || maxDebugReflections > 1) {
		maxDebugReflections += change;
    std::cout << "max Debug ray reflections: " << maxDebugReflections-1 << std::endl;
	}
}

void Flyscene::initialize(int width, int height) {
	// initiliaze the Phong Shading effect for the Opengl Previewer
	phong.initialize();

	//set Max debug ray reflections
	maxDebugReflections = 3;

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

	/* // create a first ray-tracing light source at some random position */
	/* lights.push_back(Eigen::Vector3f(0.0, 1.0, 0.0)); */

	// scale the camera representation (frustum) for the ray debug
	camerarep.shapeMatrix()->scale(0.2);

	// craete a first debug ray pointing at the center of the screen
	createDebugRay(Eigen::Vector2f(width / 2.0, height / 2.0));

	glEnable(GL_DEPTH_TEST);

	/********** from now on initialize the GUI sliders, labels, and buttons ************/

	gui.setViewportSize(width, height);

	string assets_path = "resources/assets/";

	menu_button.setPosition(10, 10);
	menu_button.onClick([&]() {groupbox.toggleDisplay(); });
	menu_button.setTexture(assets_path + "menu_button.pam");
	menu_button.setDimensionsFromHeight(30);
	gui.add(&menu_button);


	groupbox.setPosition(1, 50);
	groupbox.setDimensions(100, 210);
	groupbox.setTexture(assets_path + "groupbox_long.pam");
	gui.add(&groupbox);

	shadow_button.setPosition(10, 60);
	shadow_button.onClick([&]() {toggleSoftShadows(); });
	shadow_button.setTexture(assets_path + "shadowmap_button.pam");
	shadow_button.setDimensionsFromHeight(30);
	groupbox.add(&shadow_button);

	aa_button.setPosition(10, 110);
	aa_button.onClick([&]() {toggleAntiAliasing(); });
	aa_button.setTexture(assets_path + "reload_button.pam");
	aa_button.setDimensionsFromHeight(30);
	groupbox.add(&aa_button);

	increment_reflections.setPosition(40, 160);
	increment_reflections.onClick([&]() {incrementReflections(); });
	increment_reflections.setTexture(assets_path + "reload_button.pam");
	increment_reflections.setDimensionsFromHeight(30);
	groupbox.add(&increment_reflections);

	decrement_reflections.setPosition(10, 160);
	decrement_reflections.onClick([&]() {decrementReflections(); });
	decrement_reflections.setTexture(assets_path + "reload_button.pam");
	decrement_reflections.setDimensionsFromHeight(30);
	groupbox.add(&decrement_reflections);

	increment_smoothing.setPosition(40, 210);
	increment_smoothing.onClick([&]() {incrementSmoothing(); });
	increment_smoothing.setTexture(assets_path + "reload_button.pam");
	increment_smoothing.setDimensionsFromHeight(30);
	groupbox.add(&increment_smoothing);

	decrement_smoothing.setPosition(10, 210);
	decrement_smoothing.onClick([&]() {decrementSmoothing(); });
	decrement_smoothing.setTexture(assets_path + "reload_button.pam");
	decrement_smoothing.setDimensionsFromHeight(30);
	groupbox.add(&decrement_smoothing);


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
  if (!lights.empty())
    scene_light.viewMatrix()->translate(-lights.back());

  // render the scene using OpenGL and one light source
  phong.render(mesh, flycamera, scene_light);

  gui.render();

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
  float dx = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ? 0.2 : 0.0) -
             (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ? 0.2 : 0.0);
  float dy = (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS
                  ? 0.2
                  : 0.0) -
             (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS
                  ? 0.2
                  : 0.0);
  float dz = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ? 0.2 : 0.0) -
             (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ? 0.2 : 0.0);

  flycamera.translate(dx, dy, dz);
}

void Flyscene::traceDebugRay(Eigen::Vector3f from, Eigen::Vector3f to,
                             int maxReflections, float refrIndex) {

  Eigen::Affine3f shapeMatrix = mesh.getShapeModelMatrix();
  Eigen::MatrixXf normalMatrix = shapeMatrix.linear().inverse().transpose();
  Eigen::Vector3f rayDirection = (to - from).normalized();

  Tucano::Face *closestFace;
  Eigen::Vector3f *closestIntersect;
  if (meshHierarchy.intersect(from, to, &closestFace, &closestIntersect, true)) {
    Eigen::Vector3f rayVector = *closestIntersect - from;
    float minDist = rayVector.norm();

    // calculating reflection
    Eigen::Vector3f reflectDir = reflect(rayDirection, closestFace->normal);
    Tucano::Shapes::Cylinder ray =
        Tucano::Shapes::Cylinder(0.01, minDist, 16, 64);

    ray.resetModelMatrix();
    ray.setOriginOrientation(from, rayDirection);
    debugRays.push_back(ray);
    if (maxReflections > 1) {
      traceDebugRay(*closestIntersect + reflectDir * 0.001f,
                    *closestIntersect + reflectDir, maxReflections - 1, refrIndex);

	  
	  Eigen::Vector3f light = Eigen::Vector3f(1.f, 1.f, 1.f);
	  float refrMatIndex = materials[closestFace -> material_id].getOpticalDensity();

	  //std::cout << refrMatIndex << "\n";
	  /*if (refrMatIndex == refrIndex) {
		  refrMatIndex = 1.f;
	  }*/
	  //Eigen::Vector3f realDir = (origin + rayDirection).normalized();

	  float cosRefr = rayDirection.dot(closestFace -> normal);
	  Eigen::Vector3f realNorm = closestFace->normal;
	  if (cosRefr < 0) {
		  cosRefr = -cosRefr;
	  }
	  else {
		  realNorm = -realNorm;
		  //float spom = refrMatIndex;
		  refrMatIndex = 1.f;
	  }

	  //if(cosRefr > 1.f)   std::cout << "cosRefr is " << cosRefr << "\n";

	  float isRefr = 1.f - pow(refrIndex, 2) * (1.f - pow(cosRefr, 2)) / pow(refrMatIndex, 2);

	  if (isRefr > 0) {
		  Eigen::Vector3f t = ((refrIndex / refrMatIndex) * (rayDirection + cosRefr * realNorm) - realNorm * sqrt(isRefr)).normalized();
		  traceDebugRay(*closestIntersect + 0.001f * t, *closestIntersect + t, maxReflections - 1, refrMatIndex);
	  }

	  /*if (isRefr < 0.f) {
		  //std::cout << "here\n";
		  return diffuse + specular + ks.cwiseProduct(traceRay(intersect + 0.001f * surfaceNormal,
			  intersect + reflectedVector, levels - 1, true, refrIndex));
	  }*/
    }
  }
  else {
    Tucano::Shapes::Cylinder ray = Tucano::Shapes::Cylinder(0.01, 42, 16, 64);
    ray.resetModelMatrix();
    ray.setOriginOrientation(from, rayDirection);
    debugRays.push_back(ray);
  }
}

void Flyscene::createDebugRay(const Eigen::Vector2f &mouse_pos) {
  debugRays.clear();

  // from pixel position to ra coordinates
  Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouse_pos);

  // direction from camera center to click position
  Eigen::Vector3f dir = (screen_pos - flycamera.getCenter()).normalized();

  // position and orient the cylinder representing the ray

  traceDebugRay(flycamera.getCenter(), flycamera.getCenter() + dir, maxDebugReflections, 1.0);

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
      int x_start = i * ((image_size[1]) / threads);
      int x_end = x_start + ((image_size[1]) / threads);

      //std::cout << "Starting thread " << i << " of " << threads << std::endl;
      workers[i] =
          std::thread(&Flyscene::raytracePartScene, this, std::ref(pixel_data),
                      image_size[1], image_size[0], x_start, x_end);
    }

    // wait for threads to finish
    for (auto &t : workers) {
      t.join();
      //std::cout << "\nThread finished." << std::endl;
    }
	printProgress = 0.f;
	pixelProcessed = 0;
	long long noFaces = mesh.getNumberOfFaces();
	noFaces *= image_size[0] * image_size[1];
	std::cout << meshHierarchy.getfacesChecked() << " Faces checked (not including reflections)" << std::endl;
	std::cout << noFaces*SOFTSHADOW_POINTS*SSAA_X << " Faces to check w/o Acc structure (not including reflections)" << std::endl;
	std::cout << ((float)meshHierarchy.getfacesChecked())/(noFaces*SOFTSHADOW_POINTS*SSAA_X) * 100 << "% Faces checked in comparison to no Acceleration structure" << std::endl;

	meshHierarchy.setfacesChecked(0);
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
  
  Eigen::Vector3f temp = flycamera.screenToWorld(Eigen::Vector2f(0, x_start)) - flycamera.screenToWorld(Eigen::Vector2f(1, x_start));
  //distance between Pixels
  float unitDistance = temp.norm() / 2;


  // for every pixel shoot a ray from the origin through the pixel coords
  for (int j = x_start; j < x_end; ++j) {

	  float progress = (float)pixelProcessed / ((float)(width * height));
		  if(progress > printProgress) {
			  printProgress += 0.01;
			  std::cout << progress*100 << "% raytraced                       \r" << std::flush;
		   }

    for (int i = 0; i < height; ++i) {
		++pixelProcessed;

      // create a ray from the camera passing through the pixel (i,j)
      screen_coords = flycamera.screenToWorld(Eigen::Vector2f(i, j));
      // launch raytracing for the given ray and write result to pixel data
	 
	  //creating 4 random Points in 1 Pixel
	  std::vector<Eigen::Vector3f> pixelPoints = create_points(unitDistance, SSAA_X, screen_coords, (screen_coords - origin));
	  Eigen::Vector3f temp;
	  temp.fill(0);
	  for (Eigen::Vector3f v : pixelPoints) {
		  temp += traceRay(origin, v, MAX_REFLECTIONS, false, 1.0);
	  }
	  //screen_coords get 4 unit points around it
	  Eigen::Vector3f raw = temp / SSAA_X;

      // gamma 2 correction
      pixel_data[i][j] = Eigen::Vector3f(sqrt(clamp(raw(0), 0.f, 1.f)),
                                         sqrt(clamp(raw(1), 0.f, 1.f)),
                                         sqrt(clamp(raw(2), 0.f, 1.f)));
    }
  }
}


Eigen::Vector3f Flyscene::traceRay(const Eigen::Vector3f &origin,
                                   const Eigen::Vector3f &dest,
                                   int levels, bool isReflected, float refrIndex) {
  using namespace Eigen;

  const Vector3f background = Vector3f(0.95f, 0.95f, 0.95f);
  const Vector3f outOfReflections = Vector3f(0.f, 0.f, 0.f);

	if (levels <= 0) {
		return outOfReflections;
	}

  Eigen::Affine3f shapeMatrix = mesh.getShapeModelMatrix();
  Eigen::MatrixXf normalMatrix = shapeMatrix.linear().inverse().transpose();

  Tucano::Face *closestFace;
  Eigen::Vector3f *closestIntersect;
  if (!meshHierarchy.intersect(origin, dest, &closestFace, &closestIntersect, isReflected))
    return isReflected ? Vector3f(0.f, 0.f, 0.f) : background;

  // Interpolate normal
  Eigen::Vector4f vert1 = shapeMatrix * mesh.getVertex(closestFace->vertex_ids[0]);
  Eigen::Vector4f vert2 = shapeMatrix * mesh.getVertex(closestFace->vertex_ids[1]);
  Eigen::Vector4f vert3 = shapeMatrix * mesh.getVertex(closestFace->vertex_ids[2]);
  Eigen::Vector2f barycentric = calculateBarycentric(
      vert1.head<3>() / vert1.w(), vert2.head<3>() / vert2.w(),
      vert3.head<3>() / vert3.w(), *closestIntersect);
  Eigen::Vector3f surfaceNormal = interpolate(
      mesh.getNormal(closestFace->vertex_ids[0]).normalized(),
      mesh.getNormal(closestFace->vertex_ids[1]).normalized(),
      mesh.getNormal(closestFace->vertex_ids[2]).normalized(), barycentric);
  surfaceNormal = (normalMatrix * surfaceNormal.normalized()).normalized();

  Eigen::Vector3f rayDirection = (dest - origin).normalized();
  return calculateShading(*closestFace, *closestIntersect, surfaceNormal,
      origin, rayDirection, levels, isReflected, refrIndex);
}

bool button_press = false;

Eigen::Vector3f Flyscene::calculateShading(const Tucano::Face& face,
    const Eigen::Vector3f& intersect, const Eigen::Vector3f& surfaceNormal,
    const Eigen::Vector3f& origin, const Eigen::Vector3f& rayDirection,
    int levels, bool isReflected, float refrIndex)
{
  // Material properties
  auto material = materials[face.material_id];
  Eigen::Vector3f kd = material.getDiffuse();
  Eigen::Vector3f ks = material.getSpecular();
  float shininess = material.getShininess();
  int illumination = material.getIlluminationModel();

  // calculate diffuse + specular illumination
  Eigen::Vector3f diffuse = Eigen::Vector3f(0.0, 0.0, 0.0);
  Eigen::Vector3f specular = Eigen::Vector3f(0.0, 0.0, 0.0);
  for (auto light : lights) {
    Eigen::Vector3f lightColour = Eigen::Vector3f(1.0, 1.0, 1.0);

    Eigen::Vector3f rayVector = intersect - origin;

    // check if in hard 
	if (softShadowsEnabled) {
		float ratio = lightRatio(0.02, SOFTSHADOW_POINTS, light, face, intersect + 0.001f * surfaceNormal);
		lightColour *= ratio;

		Eigen::Vector3f toLight = light - intersect;
		Eigen::Vector3f toLightUnit = toLight.normalized();
		float lightDistance = toLight.norm();
		Eigen::Vector3f reflectedLight = reflect(-toLightUnit, surfaceNormal);

		// if no hit on ray back to light -> illuminated
		diffuse += kd.cwiseProduct(lightColour) *
			std::max(0.f, surfaceNormal.dot(toLightUnit)) / lightDistance;
		specular += ks.cwiseProduct(lightColour) *
			pow(max(rayDirection.dot(-reflectedLight), 0.f), shininess);
	}
	else if (lightBlocked(face, intersect + 0.001f * surfaceNormal, light)) {
		Eigen::Vector3f toLight = light - intersect;
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

	// Compute recursive ray tracing.
	Eigen::Vector3f reflectedVector = reflect(rayDirection, surfaceNormal).normalized();

  switch (illumination) {
  case 0:
    return kd;

  case 1:
    return diffuse;

  case 2:
	return diffuse + specular;
	  
  case 3:
    return diffuse + ks.cwiseProduct(
      traceRay(intersect + 0.001f * surfaceNormal,
          intersect + 0.001f * surfaceNormal + reflectedVector,
          levels - 1, true, refrIndex).array().pow(shininess).matrix());

  case 4:
	  return diffuse + ks.cwiseProduct(
		  traceRay(intersect + 0.001f * surfaceNormal,
			  intersect + 0.001f * surfaceNormal + reflectedVector,
			  levels - 1, true, refrIndex).array().pow(shininess).matrix());

  case 6: {
	  Eigen::Vector3f light = Eigen::Vector3f(1.f, 1.f, 1.f);
	  float refrMatIndex = material.getOpticalDensity();

	  float cosRefr = rayDirection.dot(surfaceNormal.normalized());
	  Eigen::Vector3f realNorm = surfaceNormal;
	  if (cosRefr < 0) {
		  cosRefr = -cosRefr;
	  }
	  else {
		  realNorm = -realNorm;
		  refrMatIndex = 1.f;
	  }


	  float isRefr = 1.f - pow(refrIndex, 2) * (1.f - pow(cosRefr, 2)) / pow(refrMatIndex, 2);



	  if (isRefr < 0.f) {
		  return diffuse + specular + ks.cwiseProduct(traceRay(intersect + 0.001f * surfaceNormal,
			  intersect + reflectedVector, levels - 1, true, refrIndex));
	  }

	  Eigen::Vector3f t = ((refrIndex / refrMatIndex) * (rayDirection + cosRefr * realNorm) - realNorm * sqrt(isRefr)).normalized();
	  
	  return diffuse + specular + ks.cwiseProduct(traceRay(intersect + 0.001f * surfaceNormal,
		  intersect + reflectedVector, levels - 1, true, refrIndex)) + (light - ks).cwiseProduct(traceRay(intersect + 0.001f * surfaceNormal,
			  intersect + t, levels - 1, true, refrMatIndex));
  }

  default:
    return Eigen::Vector3f(0.0, 0.0, 0.0);
  }
}

bool Flyscene::lightBlocked(const Tucano::Face &originFace,
                            Eigen::Vector3f origin, Eigen::Vector3f lightPos) {
  Tucano::Face *closestFace;
  Eigen::Vector3f *closestIntersect;
  if (!meshHierarchy.intersect(origin, lightPos, &closestFace, &closestIntersect, false))
    return false;

  // a surface cannot cast shadow on itself
  if ((originFace.vertex_ids[0] == closestFace->vertex_ids[0]) &&
      (originFace.vertex_ids[1] == closestFace->vertex_ids[1]) &&
      (originFace.vertex_ids[2] == closestFace->vertex_ids[2]))
    return false;

  return true;
}

void Flyscene::toggleSoftShadows() {
	softShadowsEnabled = !softShadowsEnabled;
	std::cout <<"soft shadows enabled: " << (softShadowsEnabled ? "true" : "false") << std::endl;
}
void Flyscene::toggleAntiAliasing() {
	SSAA_X = SSAA_X == 4 ? 1 : 4;
	std::cout << "anti aliasing on: " << (SSAA_X == 4 ? "true" : "false") << std::endl;
}
float Flyscene::lightRatio(float radius, int times, Eigen::Vector3f lightpos, const Tucano::Face& originFace, Eigen::Vector3f origin) {
	vector<Eigen::Vector3f> points = create_points(radius, times, lightpos, lightpos-origin);
	float count = times;
	for (Eigen::Vector3f point : points) {
		count -= lightBlocked(originFace, origin, point);
	}
	return count / times;
}
vector<Eigen::Vector3f> Flyscene::create_points(float radius, int times, Eigen::Vector3f pos, Eigen::Vector3f dir) {
	float a, b;
	vector<Eigen::Vector3f> ret;
	Eigen::Vector3f e1 = (dir.unitOrthogonal() + pos).normalized();
	Eigen::Vector3f e2 = (e1.cross(dir) + pos).normalized();
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-radius, radius);
	for (int i = 0; i < times; i++) {
		do {
			a = dis(gen);
			b = dis(gen);
		} while (sqrt(pow(a, 2) + pow(b, 2)) < radius && sqrt(pow(a, 2) + pow(b, 2)) > 0.5 * radius);


		ret.push_back(pos + a*e1 + b*e2);
	}
	return ret;
}
void Flyscene::incrementReflections() {
	MAX_REFLECTIONS += 1;
	std::cout << "max reflections: " << MAX_REFLECTIONS << std::endl;
}
void Flyscene::decrementReflections() {
	MAX_REFLECTIONS -= 1;
	std::cout << "max reflections: " << MAX_REFLECTIONS << std::endl;
}

void Flyscene::incrementSmoothing() {
	SOFTSHADOW_POINTS += 1;
	std::cout << "max shadowPoints: " << SOFTSHADOW_POINTS << std::endl;
}
void Flyscene::decrementSmoothing() {
	SOFTSHADOW_POINTS -= 1;
	std::cout << "max shadowPoints: " << SOFTSHADOW_POINTS << std::endl;
}

