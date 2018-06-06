#include "rqt_gcs_gui/View.h"

#include <pluginlib/class_list_macros.h>

#include <QDebug>
#include <QString>
#include <QAbstractSlider>

namespace rqt_gcs_gui {

bool resetPressed = false;

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

View::View()
    : rqt_gui_cpp::Plugin(),
      widget_(nullptr) {

  setObjectName("View");
}

/* ========================================================================== */
/* Initialize/Shutdown                                                        */
/* ========================================================================== */

void View::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() +
        " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  connect(ui_.pushButton_verifyWifi, SIGNAL(pressed()), this, SLOT(on_pushButton_verifyWifi_clicked()));
  connect(ui_.pushButton_cam, SIGNAL(pressed()), this, SLOT(on_pushButton_cam_clicked()));
  connect(ui_.pushButton_takeoff, SIGNAL(pressed()), this, SLOT(on_pushButton_takeoff_clicked()));
  connect(ui_.pushButton_land, SIGNAL(pressed()), this, SLOT(on_pushButton_land_clicked()));
  connect(ui_.pushButton_emergency, SIGNAL(pressed()), this, SLOT(on_pushButton_emergency_clicked()));
  connect(ui_.pushButton_nodeletStart, SIGNAL(pressed()), this, SLOT(on_pushButton_nodeletStart_clicked()));
  connect(ui_.pushButton_nodeletTerminate, SIGNAL(pressed()), this, SLOT(on_pushButton_nodeletTerminate_clicked()));
  connect(ui_.pushButton_steeringReset, SIGNAL(pressed()), this, SLOT(on_pushButton_steeringReset_clicked()));
  connect(ui_.slider_x, SIGNAL(valueChanged(int)), this, SLOT(on_slider_valueChanged()));
  connect(ui_.slider_y, SIGNAL(valueChanged(int)), this, SLOT(on_slider_valueChanged()));
  connect(ui_.slider_z, SIGNAL(valueChanged(int)), this, SLOT(on_slider_valueChanged()));
  connect(ui_.slider_r, SIGNAL(valueChanged(int)), this, SLOT(on_slider_valueChanged()));
  connect(ui_.lineEdit_keyInput, SIGNAL(textChanged(QString)), this, SLOT(on_lineEdit_keyInput_textChanged(QString)));

    //      ros::Subscriber(const std::string &topic, const NodeHandle &node_handle, const SubscriptionCallbackHelperPtr &helper)
  //ros::Publisher(const std::string &topic, const std::string &md5sum, const std::string &datatype, const NodeHandle, const Subscriber callback)
  //        ros::NodeHandle()
  //ros::Publisher()

  //ros::init()
}

void View::shutdownPlugin() {
}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void View::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const {
}

void View::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings) {
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */



/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */


void View::on_pushButton_cam_clicked()
{
    std::string pitch = std::to_string(ui_.spinBox_cameraPitch->value());
    std::string yaw   = std::to_string(ui_.spinBox_cameraYaw->value());
    std::string cmd = "rostopic pub --once /bebop/camera_control geometry_msgs/Twist '[0,0,0]' '[0," + pitch + "," + yaw + "]' &";
    system(cmd.c_str());
    cmd = "echo " + cmd;
    system(cmd.c_str());
}

void View::on_pushButton_verifyWifi_clicked()
{
    std::string cmd;
    std::string wifi_target = ui_.lineEdit_ssid->text().toUtf8().constData();
    std::string wifi_actual = GetStdoutFromCommand("nmcli -t -f active,ssid dev wifi | egrep '^yes'");
    //system(wifi_actual.c_str());
    if (wifi_actual.length() > 0) {
        wifi_actual = wifi_actual.substr(4,wifi_actual.length()-5);

        if (wifi_actual == wifi_target) {
            cmd = "echo wifi check success: " + wifi_actual;
            ui_.pushButton_nodeletStart->setEnabled(true);
            ui_.lineEdit_ssid->setStyleSheet("QLineEdit { background: rgb(0, 255, 0); }");
        } else {
            cmd = "echo wifi check fail: " + wifi_actual;
            ui_.pushButton_nodeletStart->setEnabled(true);
            ui_.lineEdit_ssid->setStyleSheet("QLineEdit { background: rgb(255, 0, 0); }");
        }
    } else {
        cmd = "echo wifi check fail: not connected";
        ui_.lineEdit_ssid->setStyleSheet("QLineEdit { background: rgb(255, 0, 0); }");
    }
    system(cmd.c_str());
}

std::string GetStdoutFromCommand(std::string cmd) {
    std::string echo = "echo " + cmd;
    system(echo.c_str());
    std::string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
        if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    return data;
}

void View::on_pushButton_takeoff_clicked()
{
    std::string cmd = "rostopic pub --once /bebop/takeoff std_msgs/Empty &";
    system(cmd.c_str());
    cmd = "echo " + cmd;
    system(cmd.c_str());
}

void View::on_pushButton_land_clicked()
{
    std::string cmd = "rostopic pub --once /bebop/land std_msgs/Empty &";
    system(cmd.c_str());
    cmd = "echo " + cmd;
    system(cmd.c_str());
}

void View::on_pushButton_emergency_clicked()
{
    std::string cmd = "rostopic pub --once /bebop/reset std_msgs/Empty &";
    system(cmd.c_str());
    cmd = "echo " + cmd;
    system(cmd.c_str());
}

void View::on_pushButton_nodeletStart_clicked()
{
    std::string cmd = "gnome-terminal -x sh -c \"roslaunch bebop_tools bebop_nodelet_iv.launch\"";
    system(cmd.c_str());
    cmd = "echo " + cmd;
    system(cmd.c_str());

    ui_.pushButton_nodeletStart->setEnabled(false);
    ui_.pushButton_nodeletTerminate->setEnabled(true);

    ui_.spinBox_cameraPitch->setEnabled(true);
    ui_.spinBox_cameraYaw->setEnabled(true);
    ui_.pushButton_cam->setEnabled(true);
    ui_.pushButton_takeoff->setEnabled(true);
    ui_.pushButton_land->setEnabled(true);
    ui_.pushButton_emergency->setEnabled(true);
    ui_.slider_x->setEnabled(true);
    ui_.slider_y->setEnabled(true);
    ui_.slider_z->setEnabled(true);
    ui_.slider_r->setEnabled(true);
    ui_.lineEdit_keyInput->setEnabled(true);
    ui_.pushButton_steeringReset->setEnabled(true);
}

void View::on_pushButton_nodeletTerminate_clicked()
{
    std::string cmd = "pkill -f \"roslaunch bebop_tools bebop_nodelet_iv.launch\"";
    system(cmd.c_str());
    cmd = "echo " + cmd;
    system(cmd.c_str());

    ui_.pushButton_nodeletStart->setEnabled(true);
    ui_.pushButton_nodeletTerminate->setEnabled(false);
}

void View::on_pushButton_steeringReset_clicked()
{
    std::string cmd = "echo reset speed";
    system(cmd.c_str());
    cmd = "rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist '[0,0,0]' '[0,0,0]' &";
    system(cmd.c_str());
    cmd = "echo " + cmd;
    system(cmd.c_str());
    resetPressed = true;
    ui_.slider_x->setValue(0);
    ui_.slider_y->setValue(0);
    ui_.slider_z->setValue(0);
    ui_.slider_r->setValue(0);
    resetPressed = false;
}

void View::on_slider_valueChanged()
{
    if (not resetPressed) {
        std::string x = std::to_string( +(float)ui_.slider_x->value()/10 );
        std::string y = std::to_string( -(float)ui_.slider_y->value()/10 );
        std::string z = std::to_string( +(float)ui_.slider_z->value()/10 );
        std::string r = std::to_string( -(float)ui_.slider_r->value()/10 );
        std::string cmd = "rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist '[" + x + "," + y + "," + z + "]' '[0,0," + r + "]' &";
        system(cmd.c_str());
        cmd = "echo " + cmd;
        system(cmd.c_str());
    }
}

void View::on_lineEdit_keyInput_textChanged(const QString &inqstr)
{
    if (not resetPressed) {
        resetPressed = true;
        QString inchar = inqstr.at(inqstr.length()-1);
        const char *small = inchar.toStdString().c_str();
        ui_.lineEdit_keyInput->setText(inchar.append(QString::fromStdString(", ...")));
        resetPressed = false;
        switch (tolower(*small)) {
            case 'w' : {
                ui_.slider_z->setValue(ui_.slider_z->value()+1);                  //ui_.label_2->setText(QString::fromStdString("D"));
                break;
            }
            case 'a' : {
                ui_.slider_r->setValue(ui_.slider_r->value()-1);
                break;
            }
            case 's' : {
                ui_.slider_z->setValue(ui_.slider_z->value()-1);
                break;
            }
            case 'd' : {
                ui_.slider_r->setValue(ui_.slider_r->value()+1);
                break;
            }
            case 'i' : {
                ui_.slider_x->setValue(ui_.slider_x->value()+1);
                break;
            }
            case 'j' : {
                ui_.slider_y->setValue(ui_.slider_y->value()-1);
                break;
            }
            case 'k' : {
                ui_.slider_x->setValue(ui_.slider_x->value()-1);
                break;
            }
            case 'l' : {
                ui_.slider_y->setValue(ui_.slider_y->value()+1);
                break;
            }
            case 'r' : {
                ui_.pushButton_steeringReset->click();
                break;
            }
            default: {

            }

        }


    }

}

/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */



/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */



/* ========================================================================== */
/* Signals                                                                    */
/* ========================================================================== */



} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_gcs_gui::View,
                       rqt_gui_cpp::Plugin)

