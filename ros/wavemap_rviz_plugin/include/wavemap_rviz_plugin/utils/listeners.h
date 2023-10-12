#ifndef WAVEMAP_RVIZ_PLUGIN_UTILS_LISTENERS_H_
#define WAVEMAP_RVIZ_PLUGIN_UTILS_LISTENERS_H_

#include <memory>
#include <utility>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>

namespace wavemap::rviz_plugin {
class ViewportCameraChangedListener : public Ogre::Viewport::Listener {
 public:
  using CallbackType = std::function<void(Ogre::Viewport*)>;

  explicit ViewportCameraChangedListener(Ogre::Viewport* viewport,
                                         CallbackType callback);
  ~ViewportCameraChangedListener() override { viewport_->removeListener(this); }

  void viewportCameraChanged(Ogre::Viewport* viewport) override {
    std::invoke(callback_, viewport);
  }

 private:
  Ogre::Viewport* viewport_;
  CallbackType callback_;
};

class CameraPrerenderListener : public Ogre::Camera::Listener {
 public:
  using CallbackType = std::function<void(Ogre::Camera*)>;

  explicit CameraPrerenderListener(Ogre::Camera* camera, CallbackType callback);
  ~CameraPrerenderListener() override { camera_->removeListener(this); }

  void cameraPreRenderScene(Ogre::Camera* cam) override {
    std::invoke(callback_, cam);
  }

 private:
  Ogre::Camera* camera_;
  CallbackType callback_;
};

class ViewportPrerenderListener {
 public:
  explicit ViewportPrerenderListener(
      Ogre::Viewport* viewport, CameraPrerenderListener::CallbackType callback);

 private:
  std::unique_ptr<CameraPrerenderListener> camera_prerender_listener_;
  ViewportCameraChangedListener viewport_camera_changed_listener_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_UTILS_LISTENERS_H_
