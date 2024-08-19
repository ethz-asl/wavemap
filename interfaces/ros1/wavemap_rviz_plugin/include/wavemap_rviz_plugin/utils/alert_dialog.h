#ifndef WAVEMAP_RVIZ_PLUGIN_UTILS_ALERT_DIALOG_H_
#define WAVEMAP_RVIZ_PLUGIN_UTILS_ALERT_DIALOG_H_

#include <string>

#include <qmessagebox.h>

namespace wavemap::rviz_plugin {
class AlertDialog : public QMessageBox {
 public:
  AlertDialog(const std::string& title, const std::string& description)
      : QMessageBox(QMessageBox::Icon::Warning, QString::fromStdString(title),
                    QString::fromStdString(description), QMessageBox::Ok) {}
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_UTILS_ALERT_DIALOG_H_
