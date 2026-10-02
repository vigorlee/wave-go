// Render the existing ROS marker text using Qt's Unicode fonts in Ogre.
// Leaders retain the marker's map coordinate; no labels are inferred.
#include <rviz/visualization_frame.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <OgreCamera.h>
#include <OgreRenderWindow.h>
#include <OgreViewport.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreBillboardSet.h>
#include <OgreBillboard.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <QImage>
#include <QPainter>
#include <QFontMetrics>
#include <QRectF>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <string>

struct LabelLayer {
  Ogre::SceneManager* scene;
  rviz::RenderPanel* panel;
  Ogre::ManualObject* leaders;
  std::string line_material;
  struct Label {
    Ogre::BillboardSet* set;
    std::string name;
    Ogre::Vector3 anchor;
    float aspect;
    Ogre::ColourValue color;
  };
  std::vector<Label> labels;
  unsigned serial = 0;
  void clear() {
    for (auto& label : labels) {
      scene->destroyBillboardSet(label.set);
      Ogre::MaterialManager::getSingleton().remove(label.name);
      Ogre::TextureManager::getSingleton().remove(label.name);
    }
    labels.clear();
    leaders->clear();
  }
  void update() {
    auto* window = panel->getRenderWindow();
    if (!window || !window->getNumViewports()) return;
    auto* viewport = window->getViewport(0);
    auto* camera = viewport->getCamera();
    const float width = viewport->getActualWidth(), height = viewport->getActualHeight();
    if (width < 10 || height < 10) return;
    const auto matrix = camera->getProjectionMatrix() * camera->getViewMatrix();
    const auto inverse = matrix.inverse();
    std::vector<QRectF> occupied;
    leaders->clear();
    if (labels.empty()) return;
    leaders->begin(line_material, Ogre::RenderOperation::OT_LINE_LIST);
    for (auto& label : labels) {
      const auto clip = matrix * Ogre::Vector4(label.anchor.x, label.anchor.y, label.anchor.z, 1);
      const bool visible = clip.w > 0 && std::abs(clip.x) < clip.w && std::abs(clip.y) < clip.w;
      label.set->setVisible(visible);
      if (!visible) continue;
      const float sx = (clip.x/clip.w + 1)*width/2, sy = (1 - clip.y/clip.w)*height/2;
      // Constant screen size keeps Chinese legible at the map overview scale.
      const float h = 32, w = h * label.aspect;
      QRectF best;
      float cost = std::numeric_limits<float>::max();
      for (int row = 0; row < 18; ++row) {
        const float dy = row == 0 ? -8 : ((row % 2) ? -1 : 1) * ((row+1)/2) * (h+8) - 8;
        for (float dx : {0.f, -w*.65f, w*.65f, -w*1.2f, w*1.2f}) {
          QRectF rect(std::clamp(sx+dx-w/2, 4.f, std::max(4.f,width-w-4)),
                      std::clamp(sy+dy-h, 4.f, std::max(4.f,height-h-4)), w,h);
          bool overlap = false;
          for (const auto& other : occupied)
            overlap |= rect.adjusted(-4,-4,4,4).intersects(other);
          float score = dx*dx + dy*dy + (overlap ? 1e8f : 0);
          if (score < cost) { cost=score; best=rect; }
        }
      }
      occupied.push_back(best);
      const auto unproject = [&](float px, float py) {
        auto point = inverse * Ogre::Vector4(2*px/width-1, 1-2*py/height, clip.z/clip.w, 1);
        return Ogre::Vector3(point.x,point.y,point.z)/point.w;
      };
      const auto bottom = unproject(best.center().x(),best.bottom());
      const float world_height = (bottom-unproject(best.center().x(),best.top())).length();
      label.set->setDefaultDimensions(world_height*label.aspect,world_height);
      label.set->getBillboard(0)->setPosition(bottom);
      leaders->position(label.anchor); leaders->colour(label.color);
      leaders->position(bottom); leaders->colour(label.color);
    }
    leaders->end();
  }
};
extern "C" {
void* wavego_labels_create(void* widget) {
  auto* frame = static_cast<rviz::VisualizationFrame*>(widget);
  auto* scene = frame->getManager()->getSceneManager();
  const std::string name = "wavego_label_leaders_" + std::to_string(reinterpret_cast<uintptr_t>(widget));
  auto material = Ogre::MaterialManager::getSingleton().create(name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto* pass = material->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false); pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
  pass->setDepthWriteEnabled(false); pass->setDepthCheckEnabled(false);
  auto* lines = scene->createManualObject(name);
  lines->setDynamic(true);
  lines->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY-2);
  scene->getRootSceneNode()->attachObject(lines);
  return new LabelLayer{scene,frame->getManager()->getRenderPanel(),lines,name};
}
void wavego_labels_update(void* layer) { static_cast<LabelLayer*>(layer)->update(); }
void wavego_labels_clear(void* layer) { static_cast<LabelLayer*>(layer)->clear(); }
void wavego_labels_destroy(void* layer) {
  auto* state = static_cast<LabelLayer*>(layer); state->clear();
  state->scene->destroyManualObject(state->leaders);
  Ogre::MaterialManager::getSingleton().remove(state->line_material);
  delete state;
}
void wavego_labels_add(void* layer, const char* utf8, float x, float y, float z,
                       float r, float g, float b, float height) {
  auto* state = static_cast<LabelLayer*>(layer);
  QFont font("Noto Sans CJK SC"); font.setPixelSize(34); font.setWeight(QFont::DemiBold);
  const QString text = QString::fromUtf8(utf8);
  const QFontMetrics metrics(font);
  const int width = metrics.horizontalAdvance(text) + 32, pixels = metrics.height() + 16;
  QImage image(width, pixels, QImage::Format_RGBA8888); image.fill(Qt::transparent);
  {
    QPainter p(&image); p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(QColor(12,22,35,220)); p.setPen(Qt::NoPen);
    p.drawRoundedRect(image.rect().adjusted(1,1,-1,-1),8,8);
    p.setPen(QPen(QColor::fromRgbF(r,g,b),3));
    p.drawRoundedRect(image.rect().adjusted(2,2,-2,-2),8,8);
    p.setFont(font); p.setPen(QColor(246,248,253));
    p.drawText(image.rect(),Qt::AlignCenter,text);
  }
  const std::string name="wavego_unicode_" + std::to_string(state->serial++);
  auto texture=Ogre::TextureManager::getSingleton().createManual(name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
      width,pixels,0,Ogre::PF_BYTE_RGBA,Ogre::TU_STATIC_WRITE_ONLY);
  texture->getBuffer()->blitFromMemory(Ogre::PixelBox(width,pixels,1,Ogre::PF_BYTE_RGBA,image.bits()));
  auto material=Ogre::MaterialManager::getSingleton().create(name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto* pass=material->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false); pass->setDepthWriteEnabled(false);
  pass->setDepthCheckEnabled(false); pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setCullingMode(Ogre::CULL_NONE); pass->createTextureUnitState(name);
  auto* set=state->scene->createBillboardSet(name,1);
  set->setMaterialName(name); set->setBillboardOrigin(Ogre::BBO_BOTTOM_CENTER);
  set->setDefaultDimensions(height*width/pixels,height);
  set->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY-1);
  set->createBillboard(Ogre::Vector3(x,y,z));
  state->scene->getRootSceneNode()->attachObject(set);
  state->labels.push_back({set,name,Ogre::Vector3(x,y,z),float(width)/pixels,Ogre::ColourValue(r,g,b)});
}
}
