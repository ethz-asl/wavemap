#include "wavemap_rviz_plugin/utils/listeners.h"

namespace wavemap::rviz_plugin {
ViewportPrerenderListener::ViewportPrerenderListener(
    Ogre::Viewport* viewport, ViewportPrerenderListener::CallbackType callback)
    : viewport_camera_changed_listener_([this](Ogre::Viewport* viewport) {
        if (Ogre::Camera* new_cam = viewport->getCamera(); new_cam) {
          new_cam->addListener(&camera_prerender_listener_);
        }
      }),
      camera_prerender_listener_(std::move(callback)) {
  viewport->addListener(&viewport_camera_changed_listener_);
}
}  // namespace wavemap::rviz_plugin
