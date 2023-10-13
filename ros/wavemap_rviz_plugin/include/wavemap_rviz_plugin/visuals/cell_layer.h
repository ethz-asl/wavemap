#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_CELL_LAYER_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_CELL_LAYER_H_

#include <memory>
#include <string>
#include <vector>

#include <OgreAxisAlignedBox.h>
#include <OgreColourValue.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterial.h>
#include <OgreMovableObject.h>
#include <OgreSimpleRenderable.h>
#include <OgreString.h>

namespace wavemap {
class CellLayerRenderable;
using CellLayerRenderablePtr = std::shared_ptr<CellLayerRenderable>;
using CellLayerRenderableVector = std::vector<CellLayerRenderablePtr>;

struct Cell {
  Ogre::Vector3 center;
  Ogre::ColourValue color;

  inline void setColor(float r, float g, float b, float a = 1.0) {
    color = Ogre::ColourValue(r, g, b, a);
  }
};

class CellLayer : public Ogre::MovableObject {
 public:
  explicit CellLayer(const Ogre::MaterialPtr& cell_material);
  ~CellLayer() override { clear(); }

  void clear();
  void setCells(const std::vector<Cell>& cells);

  void setCellDimensions(float width, float height, float depth);
  void setAlpha(float alpha);

  const Ogre::String& getMovableType() const override {
    return movable_type_name_;
  }
  const Ogre::AxisAlignedBox& getBoundingBox() const override {
    return bounding_box_;
  }
  float getBoundingRadius() const override { return bounding_radius_; }
  virtual void getWorldTransforms(Ogre::Matrix4* xform) const {
    *xform = _getParentNodeFullTransform();
  }
  virtual uint16_t getNumWorldTransforms() const { return 1; }
  void _updateRenderQueue(Ogre::RenderQueue* queue) override;
  void _notifyAttached(Ogre::Node* parent, bool isTagPoint = false) override {
    MovableObject::_notifyAttached(parent, isTagPoint);
  }
  void visitRenderables(Ogre::Renderable::Visitor* /*visitor*/,
                        bool /*debug_renderables*/) override {}

  virtual void setName(const std::string& name) { mName = name; }

 private:
  uint32_t getVerticesPerCell() const;
  CellLayerRenderablePtr createRenderable(size_t num_cells);
  void shrinkRenderables();

  Ogre::AxisAlignedBox bounding_box_;
  float bounding_radius_ = 0.f;

  float width_ = 0.1f;
  float height_ = 0.1f;
  float depth_ = 0.1f;

  Ogre::MaterialPtr cell_material_;
  float alpha_ = 1.f;

  CellLayerRenderableVector renderables_;

  bool current_mode_supports_geometry_shader_ = false;

  static const Ogre::String movable_type_name_;
};

class CellLayerRenderable : public Ogre::SimpleRenderable {
 public:
  CellLayerRenderable(CellLayer* parent, size_t num_cells, bool use_tex_coords);
  ~CellLayerRenderable() override;

  using Ogre::SimpleRenderable::getRenderOperation;

  Ogre::RenderOperation* getRenderOperation() { return &mRenderOp; }

  Ogre::HardwareVertexBufferSharedPtr getBuffer();

  Ogre::Real getBoundingRadius() const override { return bounding_radius_; }
  void setBoundingRadius(float bounding_radius) {
    bounding_radius_ = bounding_radius;
  }
  Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const override;
  uint16_t getNumWorldTransforms() const override { return 1; }
  void getWorldTransforms(Ogre::Matrix4* xform) const override {
    parent_->getWorldTransforms(xform);
  }
  const Ogre::LightList& getLights() const override {
    return parent_->queryLights();
  }

 private:
  Ogre::MaterialPtr material_;
  CellLayer* parent_;
  float bounding_radius_ = 0.f;
};
}  // namespace wavemap

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_CELL_LAYER_H_
