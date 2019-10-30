#ifndef __FLYSCENE__
#define __FLYSCENE__

// Must be included before glfw.
#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <tucano/effects/phongmaterialshader.hpp>
#include <tucano/mesh.hpp>
#include <tucano/shapes/camerarep.hpp>
#include <tucano/shapes/cylinder.hpp>
#include <tucano/shapes/sphere.hpp>
#include <tucano/utils/flycamera.hpp>
#include <tucano/utils/imageIO.hpp>
#include <tucano/utils/mtlIO.hpp>
#include <tucano/utils/objimporter.hpp>
#include "tucano/gui/base.hpp"


#include "MeshHierarchy.hpp"

class Flyscene {

public:

	float printProgress = 0.f;
	int pixelProcessed = 0;

Flyscene(void) {}

  /**
   * @brief Initializes the shader effect
   * @param width Window width in pixels
   * @param height Window height in pixels
   */
  void initialize(int width, int height);

  /**
   * Repaints screen buffer.
   **/
  virtual void paintGL();

  /**
   * Perform a single simulation step.
   **/
  virtual void simulate(GLFWwindow *window);

  /**
   * Returns the pointer to the flycamera instance
   * @return pointer to flycamera
   **/
  Tucano::Flycamera *getCamera(void) { return &flycamera; }

  /**
	 * @brief Returns pointer to GUI
	 * @return pointer to GUI
	 */
  Tucano::GUI::Base* getGUI(void) { return &gui; }



  /**
   * @brief Add a new light source
   */
  void addLight(void) { lights.push_back(flycamera.getCenter()); }

  /**
   * @brief Create a debug ray at the current camera location and passing
   * through pixel that mouse is over
   * @param mouse_pos Mouse cursor position in pixels
   */
  void createDebugRay(const Eigen::Vector2f &mouse_pos);

  void traceDebugRay(Eigen::Vector3f from, Eigen::Vector3f to,
                     int maxReflections);

  void modifyDebugReflection(int change);

  /**
   * @brief raytrace your scene from current camera position
   * @see   raytracePartScene
   */
  void raytraceScene(int width = 0, int height = 0);

  /**
   * @brief raytrace part of your scene from current camera position
   */
  void raytracePartScene(vector<vector<Eigen::Vector3f>> &pixel_data,
                         int width = 0, int height = 0, int x_start = 0,
                         int x_end = 0);

  Eigen::Vector3f calculateShading(const Tucano::Face& face,
    const Eigen::Vector3f& point, const Eigen::Vector3f& surfaceNormal,
    const Eigen::Vector3f& origin, const Eigen::Vector3f& rayDirection,
    int levels, bool isReflected);

  

  /**
   * @brief trace a single ray from the camera passing through dest
   * @param origin Ray origin
   * @param dest Other point on the ray, usually screen coordinates
   * @return a RGB color
   */
  Eigen::Vector3f traceRay(const Eigen::Vector3f &origin,
      const Eigen::Vector3f &dest, int levels, bool isReflected);


  Eigen::Vector3f min(Eigen::Vector3f a, Eigen::Vector3f b);

  static bool planeIntersection(Eigen::Vector3f& origin, Eigen::Vector3f dir, Eigen::Vector3f norm, Eigen::Vector3f point, Eigen::Vector3f& intersect);

  static bool isInTriangle(Eigen::Vector3f point, Eigen::Vector3f vertice1, Eigen::Vector3f vertice2, Eigen::Vector3f vertice3);




private:

	int maxDebugReflections;

  // A simple phong shader for rendering meshes
  Tucano::Effects::PhongMaterial phong;

  // A fly through camera
  Tucano::Flycamera flycamera;

  // the size of the image generated by ray tracing
  Eigen::Vector2i raytracing_image_size;

  // A camera representation for animating path (false means that we do not
  // render front face)
  Tucano::Shapes::CameraRep camerarep = Tucano::Shapes::CameraRep(false);

  // a frustum to represent the camera in the scene
  Tucano::Shapes::Sphere lightrep;

  // light sources for ray tracing
  vector<Eigen::Vector3f> lights;

  // Scene light represented as a camera
  Tucano::Camera scene_light;

  /// A vector of Cylinders for debug rays
  std::vector<Tucano::Shapes::Cylinder> debugRays;

  // Scene meshes
  Tucano::Mesh mesh;
  MeshHierarchy meshHierarchy;

  /// MTL materials
  vector<Tucano::Material::Mtl> materials;

  bool lightBlocked(const Tucano::Face &originFace, Eigen::Vector3f origin,
                    Eigen::Vector3f lightPos);
  float lightRatio(float radius, int times, Eigen::Vector3f lightpos, const Tucano::Face& originFace, Eigen::Vector3f origin);

  vector<Eigen::Vector3f> create_points(float radius, int times, Eigen::Vector3f pos, Eigen::Vector3f dir);

  /// GUI holder
  Tucano::GUI::Base gui;

  /// Box to group all gui elements
  Tucano::GUI::GroupBox groupbox;

  /// Shadow button
  Tucano::GUI::Button shadow_button;

  /// Menu show/hide button
  Tucano::GUI::SelectButton menu_button;

  /// Reflection button
  Tucano::GUI::Button reflection_button;
     
};

#endif // FLYSCENE
