#include "wavemap_rviz_plugin/visuals/grid_layer.h"

#include <sstream>

#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <qglobal.h>
#include <ros/assert.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/ogre_helpers/custom_parameter_indices.h>
#include <rviz/ogre_helpers/ogre_vector.h>

#define VERTEX_BUFFER_CAPACITY (36 * 1024 * 10)

namespace wavemap {
static constexpr float g_point_vertices[3] = {0.0f, 0.0f, 0.0f};

static constexpr float g_box_vertices[6 * 6 * 3] = {
    // clang-format off
    // front
    -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f,
    0.5f, -0.5f, -0.5f,

    // back
    -0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f,

    // right
    0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, -0.5, 0.5,

    // left
    -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5,

    // top
    -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5, -0.5, 0.5, 0.5,

    // bottom
    -0.5, -0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, 0.5,
};  // clang-format on

const Ogre::String GridLayer::sm_Type = "GridLayer";

GridLayer::GridLayer() {
  // Make sure the material is loaded
  box_material_ =
      Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBox");
  box_material_->load();

  // Initialize transparency rendering
  setAlpha(alpha_);

  // Detect geometry shader support
  if (Ogre::Technique* best = box_material_->getBestTechnique(); best) {
    if (box_material_->getBestTechnique()->getName() == "gp") {
      current_mode_supports_geometry_shader_ = true;
    } else {
      current_mode_supports_geometry_shader_ = false;
    }
  } else {
    ROS_ERROR("No techniques available for material [%s]",
              box_material_->getName().c_str());
    current_mode_supports_geometry_shader_ = false;
  }
}

void GridLayer::clear() {
  cell_count_ = 0;
  bounding_box_.setNull();
  bounding_radius_ = 0.0f;

  if (getParentSceneNode()) {
    for (const auto& renderable : renderables_) {
      getParentSceneNode()->detachObject(renderable.get());
    }
    getParentSceneNode()->needUpdate();
  }

  renderables_.clear();
}

void GridLayer::regenerateAll() {
  if (cell_count_ == 0) {
    return;
  }

  CellVector cells;
  cells.swap(cells_);
  uint32_t count = cell_count_;

  clear();

  addCells(&cells.front(), count);
}

void GridLayer::setCellDimensions(float width, float height, float depth) {
  width_ = width;
  height_ = height;
  depth_ = depth;

  const Ogre::Vector4 size(width_, height_, depth_, 0.0f);
  for (const auto& renderable : renderables_) {
    renderable->setCustomParameter(SIZE_PARAMETER, size);
  }
}

void setAlphaBlending(const Ogre::MaterialPtr& mat) {
  if (mat->getBestTechnique()) {
    mat->getBestTechnique()->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    mat->getBestTechnique()->setDepthWriteEnabled(false);
  }
}

void setReplace(const Ogre::MaterialPtr& mat) {
  if (mat->getBestTechnique()) {
    mat->getBestTechnique()->setSceneBlending(Ogre::SBT_REPLACE);
    mat->getBestTechnique()->setDepthWriteEnabled(true);
  }
}

void GridLayer::setAlpha(float alpha, bool per_cell_alpha) {
  alpha_ = alpha;

  if (alpha < 0.9998 || per_cell_alpha) {
    setAlphaBlending(box_material_);
  } else {
    setReplace(box_material_);
  }

  const Ogre::Vector4 alpha4(alpha_, alpha_, alpha_, alpha_);
  for (const auto& renderable : renderables_) {
    renderable->setCustomParameter(ALPHA_PARAMETER, alpha4);
  }
}

void GridLayer::addCells(Cell* cells, uint32_t num_cells) {
  if (num_cells == 0) {
    return;
  }
  Ogre::Root* root = Ogre::Root::getSingletonPtr();

  if (cells_.size() < cell_count_ + num_cells) {
    cells_.resize(cell_count_ + num_cells);
  }

  Cell* begin = &cells_.front() + cell_count_;
#if defined(__GNUC__) && (__GNUC__ >= 8)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
  memcpy(begin, cells, sizeof(Cell) * num_cells);
#if defined(__GNUC__) && (__GNUC__ >= 8)
#pragma GCC diagnostic pop
#endif

  uint32_t vpp = getVerticesPerCell();
  Ogre::RenderOperation::OperationType op_type;
  if (current_mode_supports_geometry_shader_) {
    op_type = Ogre::RenderOperation::OT_POINT_LIST;
  } else {
    op_type = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  }

  const float* vertices = nullptr;
  if (current_mode_supports_geometry_shader_) {
    vertices = g_point_vertices;
  } else {
    vertices = g_box_vertices;
  }

  GridLayerRenderablePtr rend;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  void* vdata = nullptr;
  Ogre::RenderOperation* op = nullptr;
  float* fptr = nullptr;

  Ogre::AxisAlignedBox aabb;
  aabb.setNull();
  uint32_t current_vertex_count = 0;
  bounding_radius_ = 0.0f;
  uint32_t vertex_size = 0;
  uint32_t buffer_size = 0;
  for (uint32_t current_cell = 0; current_cell < num_cells; ++current_cell) {
    // if we didn't create a renderable yet,
    // or we've reached the vertex limit for the current renderable,
    // create a new one.
    while (!rend || current_vertex_count >= buffer_size) {
      if (rend) {
        ROS_ASSERT(current_vertex_count == buffer_size);

        op->vertexData->vertexCount =
            rend->getBuffer()->getNumVertices() - op->vertexData->vertexStart;
        ROS_ASSERT(op->vertexData->vertexCount + op->vertexData->vertexStart <=
                   rend->getBuffer()->getNumVertices());
        vbuf->unlock();
        rend->setBoundingBox(aabb);
        bounding_box_.merge(aabb);
      }

      buffer_size = std::min<int>(VERTEX_BUFFER_CAPACITY,
                                  (num_cells - current_cell) * vpp);

      rend = createRenderable(buffer_size);
      vbuf = rend->getBuffer();
      vdata = vbuf->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE);

      op = rend->getRenderOperation();
      op->operationType = op_type;
      current_vertex_count = 0;

      vertex_size = op->vertexData->vertexDeclaration->getVertexSize(0);
      fptr = reinterpret_cast<float*>(vdata);

      aabb.setNull();
    }

    const Cell& p = cells[current_cell];

    uint32_t color;
    root->convertColourValue(p.color, &color);

    aabb.merge(p.center);
    bounding_radius_ = std::max(bounding_radius_, p.center.squaredLength());

    float x = p.center.x;
    float y = p.center.y;
    float z = p.center.z;

    for (uint32_t j = 0; j < vpp; ++j, ++current_vertex_count) {
      *fptr++ = x;
      *fptr++ = y;
      *fptr++ = z;

      if (!current_mode_supports_geometry_shader_) {
        *fptr++ = vertices[(j * 3)];
        *fptr++ = vertices[(j * 3) + 1];
        *fptr++ = vertices[(j * 3) + 2];
      }
      std::memcpy(fptr++, &color, sizeof(float));

      ROS_ASSERT((uint8_t*)fptr <=
                 (uint8_t*)vdata +
                     rend->getBuffer()->getNumVertices() * vertex_size);
      Q_UNUSED(vertex_size);
    }
  }

  op->vertexData->vertexCount =
      current_vertex_count - op->vertexData->vertexStart;
  rend->setBoundingBox(aabb);
  bounding_box_.merge(aabb);
  ROS_ASSERT(op->vertexData->vertexCount + op->vertexData->vertexStart <=
             rend->getBuffer()->getNumVertices());

  vbuf->unlock();

  cell_count_ += num_cells;

  shrinkRenderables();

  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

void GridLayer::popCells(uint32_t num_cells) {
  uint32_t vpp = getVerticesPerCell();

  ROS_ASSERT(num_cells <= cell_count_);
  cells_.erase(cells_.begin(), cells_.begin() + num_cells);

  cell_count_ -= num_cells;

  // Now clear out popped cells
  uint32_t popped_count = 0;
  while (popped_count < num_cells * vpp) {
    GridLayerRenderablePtr rend = renderables_.front();
    Ogre::RenderOperation* op = rend->getRenderOperation();

    uint32_t popped =
        std::min(static_cast<size_t>(num_cells * vpp - popped_count),
                 op->vertexData->vertexCount);
    op->vertexData->vertexStart += popped;
    op->vertexData->vertexCount -= popped;

    popped_count += popped;

    if (op->vertexData->vertexCount == 0) {
      renderables_.erase(renderables_.begin(), renderables_.begin() + 1);

      op->vertexData->vertexStart = 0;
      renderables_.push_back(rend);
    }
  }
  ROS_ASSERT(popped_count == num_cells * vpp);

  // reset bounds
  bounding_box_.setNull();
  bounding_radius_ = 0.0f;
  for (uint32_t i = 0; i < cell_count_; ++i) {
    Cell& p = cells_[i];
    bounding_box_.merge(p.center);
    bounding_radius_ = std::max(bounding_radius_, p.center.squaredLength());
  }

  shrinkRenderables();

  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

void GridLayer::shrinkRenderables() {
  while (!renderables_.empty()) {
    GridLayerRenderablePtr rend = renderables_.back();
    Ogre::RenderOperation* op = rend->getRenderOperation();
    if (op->vertexData->vertexCount == 0) {
      renderables_.pop_back();
    } else {
      break;
    }
  }
}

void GridLayer::_updateRenderQueue(Ogre::RenderQueue* queue) {
  for (const auto& renderable : renderables_) {
    queue->addRenderable(renderable.get());
  }
}

uint32_t GridLayer::getVerticesPerCell() const {
  if (current_mode_supports_geometry_shader_) {
    return 1;
  } else {
    return 36;
  }
}

GridLayerRenderablePtr GridLayer::createRenderable(int num_cells) {
  GridLayerRenderablePtr rend(new GridLayerRenderable(
      this, num_cells, !current_mode_supports_geometry_shader_));
  rviz::setMaterial(*rend, box_material_);
  const Ogre::Vector4 size(width_, height_, depth_, 0.0f);
  const Ogre::Vector4 alpha(alpha_, 0.0f, 0.0f, 0.0f);
  rend->setCustomParameter(SIZE_PARAMETER, size);
  rend->setCustomParameter(ALPHA_PARAMETER, alpha);
  rend->setCustomParameter(NORMAL_PARAMETER,
                           Ogre::Vector4(Ogre::Vector3::NEGATIVE_UNIT_Z));
  rend->setCustomParameter(UP_PARAMETER, Ogre::Vector4(Ogre::Vector3::UNIT_Y));
  if (getParentSceneNode()) {
    getParentSceneNode()->attachObject(rend.get());
  }
  renderables_.push_back(rend);

  return rend;
}

GridLayerRenderable::GridLayerRenderable(GridLayer* parent, int num_cells,
                                         bool use_tex_coords)
    : parent_(parent) {
  // Initialize render operation
  mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
  mRenderOp.useIndexes = false;
  mRenderOp.vertexData = new Ogre::VertexData;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = 0;

  Ogre::VertexDeclaration* decl = mRenderOp.vertexData->vertexDeclaration;
  size_t offset = 0;

  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (use_tex_coords) {
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES,
                     0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }

  decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
          mRenderOp.vertexData->vertexDeclaration->getVertexSize(0), num_cells,
          Ogre::HardwareBuffer::HBU_DYNAMIC);

  // Bind buffer
  mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);
}

GridLayerRenderable::~GridLayerRenderable() {
  delete mRenderOp.vertexData;
  delete mRenderOp.indexData;
}

Ogre::HardwareVertexBufferSharedPtr GridLayerRenderable::getBuffer() {
  return mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
}

Ogre::Real GridLayerRenderable::getBoundingRadius() const {
  return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(),
                                   mBox.getMinimum().squaredLength()));
}

Ogre::Real GridLayerRenderable::getSquaredViewDepth(
    const Ogre::Camera* cam) const {
  const Ogre::Vector3& min = mBox.getMinimum();
  const Ogre::Vector3& max = mBox.getMaximum();
  const Ogre::Vector3 mid = ((max - min) * 0.5f) + min;
  const Ogre::Vector3 offset = cam->getDerivedPosition() - mid;

  return offset.squaredLength();
}
}  // namespace wavemap
