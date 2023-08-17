#ifndef WAVEMAP_RVIZ_PLUGIN_UTILS_LISTENERS_H_
#define WAVEMAP_RVIZ_PLUGIN_UTILS_LISTENERS_H_

#include <utility>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>

namespace wavemap::rviz_plugin {
class ViewportCameraChangedListener : public Ogre::Viewport::Listener {
 public:
  using CallbackType = std::function<void(Ogre::Viewport*)>;

  explicit ViewportCameraChangedListener(CallbackType callback)
      : callback_(std::move(callback)) {}

  void viewportCameraChanged(Ogre::Viewport* viewport) override {
    std::invoke(callback_, viewport);
  }

 private:
  CallbackType callback_;
};

class CameraPrerenderListener : public Ogre::Camera::Listener {
 public:
  using CallbackType = std::function<void(Ogre::Camera*)>;

  explicit CameraPrerenderListener(CallbackType callback)
      : callback_(std::move(callback)) {}

  void cameraPreRenderScene(Ogre::Camera* cam) override {
    std::invoke(callback_, cam);
  }

 private:
  CallbackType callback_;
};

class ViewportPrerenderListener {
 public:
  using CallbackType = std::function<void(Ogre::Camera*)>;

  explicit ViewportPrerenderListener(Ogre::Viewport* viewport,
                                     CallbackType callback);

 private:
  ViewportCameraChangedListener viewport_camera_changed_listener_;
  CameraPrerenderListener camera_prerender_listener_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_UTILS_LISTENERS_H_
