#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_LAYER_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_LAYER_H_

#include <string>
#include <vector>

#include <OgreAxisAlignedBox.h>
#include <OgreColourValue.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterial.h>
#include <OgreMovableObject.h>
#include <OgreRoot.h>
#include <OgreSharedPtr.h>
#include <OgreSimpleRenderable.h>
#include <OgreString.h>
#include <boost/shared_ptr.hpp>
#include <rviz/ogre_helpers/ogre_vector.h>

namespace wavemap {
class GridLayerRenderable;
using GridLayerRenderablePtr = boost::shared_ptr<GridLayerRenderable>;
using GridLayerRenderableVector = std::vector<GridLayerRenderablePtr>;

class GridLayer : public Ogre::MovableObject {
 public:
  explicit GridLayer(const Ogre::MaterialPtr& cell_material);
  ~GridLayer() override { clear(); }

  void clear();

  struct Cell {
    Ogre::Vector3 center;
    Ogre::ColourValue color;

    inline void setColor(float r, float g, float b, float a = 1.0) {
      color = Ogre::ColourValue(r, g, b, a);
    }
  };

  void addCells(Cell* cells, uint32_t num_cells);
  void popCells(uint32_t num_cells);

  void setCellDimensions(float width, float height, float depth);

  void setAlpha(float alpha, bool per_cell_alpha = false);

  const Ogre::String& getMovableType() const override { return sm_Type; }
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
  GridLayerRenderablePtr createRenderable(int num_cells);
  void regenerateAll();
  void shrinkRenderables();

  Ogre::AxisAlignedBox bounding_box_;
  float bounding_radius_ = 0.f;

  using CellVector = std::vector<Cell>;
  CellVector cells_;
  uint32_t cell_count_ = 0;

  float width_ = 0.1f;
  float height_ = 0.1f;
  float depth_ = 0.1f;

  Ogre::MaterialPtr cell_material_;
  float alpha_ = 1.f;

  GridLayerRenderableVector renderables_;

  bool current_mode_supports_geometry_shader_ = false;

  static const Ogre::String sm_Type;
};

class GridLayerRenderable : public Ogre::SimpleRenderable {
 public:
  GridLayerRenderable(GridLayer* parent, int num_cells, bool use_tex_coords);
  ~GridLayerRenderable() override;

  using Ogre::SimpleRenderable::getRenderOperation;

  Ogre::RenderOperation* getRenderOperation() { return &mRenderOp; }

  Ogre::HardwareVertexBufferSharedPtr getBuffer();

  Ogre::Real getBoundingRadius() const override;
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
  GridLayer* parent_;
};
}  // namespace wavemap

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_GRID_LAYER_H_
