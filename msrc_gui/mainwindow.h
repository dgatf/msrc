#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define CONFIG_VERSION 2

#include <QDebug>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPainter>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include "shared.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

   public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void generateCircuit(QLabel *label);

   private:
    Ui::MainWindow *ui;
    QSerialPort *serial = nullptr;
    QByteArray data;
    bool isConnected = false;
    QStringList portsList;
    config_t config;
    bool isDebug = false;
    bool autoscroll = true;

    void requestSerialConfig();
    void getConfigFromUi();
    void setUiFromConfig();
    void openSerialPort();
    void closeSerialPort();
    void enableWidgets(QWidget *widget, bool enable);

   private slots:
    void buttonSerialPort();
    void buttonDebug();
    void buttonClearDebug();
    void readSerial();
    void readSerialConfig();
    QStringList fillPortsInfo();
    void checkPorts();
    void writeSerialConfig();
    void defaultConfig();
    void openConfig();
    void saveConfig();
    void showAbout();
    void exitApp();
    void on_cbReceiver_currentIndexChanged(const QString &arg1);
    void on_cbEsc_currentIndexChanged(const QString &arg1);
    void on_gbEsc_toggled(bool arg1);
    void on_cbEscModel_currentIndexChanged(const QString &arg1);
    void on_cbBarometerType_currentIndexChanged(const QString &arg1);
    void on_gbVoltage1_toggled(bool arg1);
    void on_cbTemperature1_toggled(bool checked);
    void on_gbAltitude_toggled(bool arg1);
    void on_gbCurrent_toggled(bool arg1);
    void on_cbBarometerType_currentTextChanged(const QString &arg1);
    void on_cbEsc_currentTextChanged(const QString &arg1);
    void on_cbReceiver_currentTextChanged(const QString &arg1);
    void on_btCircuit_clicked();
    void resizeEvent(QResizeEvent *event);
    void on_gbGps_toggled(bool arg1);
    void on_cbCurrentAutoOffset_toggled(bool checked);
    void on_cbCurrentSensorType_currentTextChanged(const QString &arg1);
    void on_cbClockStretch_toggled(bool checked);
    void on_cbEscAutoOffset_stateChanged(int arg1);
    void on_gbAirspeed_toggled(bool arg1);
    void on_gbFuelmeter_toggled(bool arg1);
    void on_btScroll_clicked();
    void on_gbFuelPressure_toggled(bool arg1);
    void on_cbGpsProtocol_currentTextChanged(const QString &arg1);
    void on_ckSbusBattery_toggled(bool checked);
    void on_ckSbusExtVolt_toggled(bool checked);
};
#endif  // MAINWINDOW_H
