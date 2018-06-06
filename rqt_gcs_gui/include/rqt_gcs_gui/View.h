#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <rqt_gcs_gui/ui_View.h>

#include <ros/ros.h>

#include <QWidget>
#include <QObject>

namespace rqt_gcs_gui {

std::string GetStdoutFromCommand(std::string cmd);

class View : public rqt_gui_cpp::Plugin {
Q_OBJECT

public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  View();

  /* ======================================================================== */
  /* Initialize/Shutdown                                                      */
  /* ======================================================================== */

  void initPlugin(qt_gui_cpp::PluginContext& context) override;

  void shutdownPlugin() override;

  /* ======================================================================== */
  /* Settings                                                                 */
  /* ======================================================================== */

  void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                    qt_gui_cpp::Settings& instance_settings) const override;

  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                       const qt_gui_cpp::Settings& instance_settings) override;

private:

  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  const std::string TAG = "View";

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::ViewWidget ui_;
  QWidget* widget_;

  ros::NodeHandle nh_;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */



  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */


  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */



protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */



signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */


private slots:

  void on_pushButton_cam_clicked();
  void on_pushButton_verifyWifi_clicked();
  void on_pushButton_takeoff_clicked();
  void on_pushButton_land_clicked();
  void on_pushButton_emergency_clicked();
  void on_pushButton_nodeletStart_clicked();
  void on_pushButton_nodeletTerminate_clicked();
  void on_pushButton_steeringReset_clicked();
  void on_slider_valueChanged();
  void on_keySequenceEdit_keySequenceChanged();
private slots:

  void on_lineEdit_keyInput_textChanged(const QString &arg1);
};

} // namespace
