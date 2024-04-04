#include "wavemap_rviz_plugin/utils/listeners.h"

#include <glog/logging.h>

namespace wavemap::rviz_plugin {
ViewportCameraChangedListener::ViewportCameraChangedListener(
    Ogre::Viewport* viewport,
    ViewportCameraChangedListener::CallbackType callback)
    : viewport_(CHECK_NOTNULL(viewport)), callback_(std::move(callback)) {
  viewport_->addListener(this);
}

CameraPrerenderListener::CameraPrerenderListener(
    Ogre::Camera* camera, CameraPrerenderListener::CallbackType callback)
    : camera_(CHECK_NOTNULL(camera)), callback_(std::move(callback)) {
  camera_->addListener(this);
}

ViewportPrerenderListener::ViewportPrerenderListener(
    Ogre::Viewport* viewport, CameraPrerenderListener::CallbackType callback)
    : viewport_camera_changed_listener_(
          CHECK_NOTNULL(viewport),
          [prerender_cb = callback, &prerender_ls = camera_prerender_listener_](
              Ogre::Viewport* viewport) {
            if (Ogre::Camera* new_camera = viewport->getCamera(); new_camera) {
              prerender_ls = std::make_unique<CameraPrerenderListener>(
                  new_camera, prerender_cb);
            } else {
              prerender_ls.reset();
            }
          }) {
  // Connect the prerender listener to the current camera, if available
  auto* initial_camera = viewport->getCamera();
  if (initial_camera) {
    camera_prerender_listener_ =
        std::make_unique<CameraPrerenderListener>(initial_camera, callback);
  }
}
}  // namespace wavemap::rviz_plugin
