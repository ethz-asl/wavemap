#ifndef WAVEMAP_RVIZ_PLUGIN_UTILS_BUTTON_PROPERTY_H_
#define WAVEMAP_RVIZ_PLUGIN_UTILS_BUTTON_PROPERTY_H_

#ifndef Q_MOC_RUN
#include <string>

#include <QPushButton>
#include <rviz/properties/string_property.h>
#endif

namespace wavemap::rviz_plugin {
class ButtonProperty : public rviz::Property {
  Q_OBJECT
 public:  // NOLINT
  ButtonProperty(const QString& name, const QString& default_value,
                 const QString& description = QString(),
                 Property* parent = nullptr,
                 const char* button_released_slot = nullptr,
                 QObject* receiver = nullptr)
      : rviz::Property(name, default_value, description, parent),
        value_original_(getValue().toString().toStdString()),
        button_released_slot_(button_released_slot),
        receiver_(receiver) {
    // Don't save the button state in the Rviz config
    setShouldBeSaved(false);
  }

  //! Create a clickable button. Called when the property comes in focus.
  QWidget* createEditor(QWidget* parent,
                        const QStyleOptionViewItem& /*option*/) override {
    in_focus_ = true;
    auto* button = new QPushButton(value_in_focus_.c_str(), parent);
    button->setEnabled(enabled_);
    QObject::connect(button, SIGNAL(released()), receiver_,
                     button_released_slot_);
    QObject::connect(button, &QPushButton::destroyed, this,
                     &ButtonProperty::buttonOutOfFocusCallback);
    return button;
  }

  // Set whether the button can be clicked
  void setEnabled(bool enabled = true) { enabled_ = enabled; }
  bool getEnabled() const { return enabled_; }

  // Set the label of the button that is shown when the property is in focus
  void setInFocusValue(const std::string& value) {
    value_in_focus_ = value;
    updateValue();
  }
  const std::string& getInFocusValue() const { return value_in_focus_; }
  void resetInFocusValue() { setInFocusValue(value_original_); }

  // Set the value that is shown when the property is out of focus
  void setAtRestValue(const std::string& value) {
    value_at_rest_ = value;
    updateValue();
  }
  const std::string& getAtRestValue() const { return value_at_rest_; }
  void resetAtRestValue() { setAtRestValue(value_at_rest_); }

  // Reset all button values (values shown when in focus and at rest)
  void resetAllValues() {
    value_in_focus_ = value_original_;
    value_at_rest_ = value_original_;
    updateValue();
  }

 private Q_SLOTS:  // NOLINT
  void buttonOutOfFocusCallback() {
    in_focus_ = false;
    updateValue();
  }

 private:
  bool enabled_ = true;
  bool in_focus_ = false;

  std::string value_original_;
  std::string value_in_focus_ = value_original_;
  std::string value_at_rest_ = value_original_;
  void updateValue() {
    if (in_focus_) {
      setValue(value_in_focus_.c_str());
    } else {
      setValue(value_at_rest_.c_str());
    }
  }

  const char* const button_released_slot_;
  QObject* const receiver_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_UTILS_BUTTON_PROPERTY_H_
