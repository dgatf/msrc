#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define CONFIG_VERSION 1

#include <QMainWindow>
#include <QDebug>
#include <QPainter>
#include <QLabel>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QFileDialog>
#include <QJsonObject>
#include <QJsonDocument>
#include <QTimer>
#include <QMessageBox>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

typedef enum rx_protocol_t : uint8_t
{
    RX_SMARTPORT,
    RX_XBUS,
    RX_SRXL,
    RX_FRSKY_D,
    RX_IBUS,
    RX_SBUS,
    RX_MULTIPLEX,
    RX_JETIEX,
    RX_HITEC
} rx_protocol_t;

typedef enum esc_protocol_t : uint8_t
{
    ESC_NONE,
    ESC_HW3,
    ESC_HW4,
    ESC_PWM,
    ESC_CASTLE,
    ESC_KONTRONIK,
    ESC_APD_F,
    ESC_APD_HV
} esc_protocol_t;

typedef enum i2c_module_t : uint8_t
{
    I2C_NONE,
    I2C_BMP280,
    I2C_MS5611,
    I2C_BMP180
} i2c_module_t;

typedef enum analog_current_type_t : uint8_t
{
    CURRENT_TYPE_HALL,
    CURRENT_TYPE_SHUNT
} analog_current_type_t;

typedef struct config_t // 122 bytes
{
    uint16_t version;
    enum rx_protocol_t rx_protocol;
    enum esc_protocol_t esc_protocol;
    bool enable_gps;
    uint32_t gps_baudrate;
    bool enable_analog_voltage;
    bool enable_analog_current;
    bool enable_analog_ntc;
    bool enable_analog_airspeed;
    enum i2c_module_t i2c_module;
    uint8_t i2c_address;
    float alpha_rpm;
    float alpha_voltage;
    float alpha_current;
    float alpha_temperature;
    float alpha_vario;
    float alpha_airspeed;
    uint16_t refresh_rate_rpm;
    uint16_t refresh_rate_voltage;
    uint16_t refresh_rate_current;
    uint16_t refresh_rate_temperature;
    uint16_t refresh_rate_gps;
    uint16_t refresh_rate_consumption;
    uint16_t refresh_rate_vario;
    uint16_t refresh_rate_airspeed;
    uint16_t refresh_rate_default;
    float analog_voltage_multiplier;
    enum analog_current_type_t analog_current_type;
    uint16_t analog_current_sensitivity;
    float analog_current_quiescent_voltage;
    float analog_current_multiplier;
    float analog_current_offset;
    bool analog_current_autoffset;
    uint8_t pairOfPoles;
    uint8_t mainTeeth;
    uint8_t pinionTeeth;
    float rpm_multiplier;
    uint8_t bmp280_filter;
    bool enable_pwm_out;
    uint8_t smartport_sensor_id;
    uint16_t smartport_data_id;
    bool vario_auto_offset;
    bool xbus_clock_stretch;
    bool jeti_gps_speed_units_kmh;
    bool enable_esc_hw4_init_delay;
    uint16_t esc_hw4_init_delay_duration;
    uint8_t esc_hw4_current_thresold;
    uint16_t esc_hw4_current_max;
    float esc_hw4_divisor;
    float esc_hw4_ampgain;
    bool ibus_alternative_coordinates;
    uint8_t debug;
    uint32_t spare1;
    uint32_t spare2;
    uint32_t spare3;
    uint32_t spare4;
    uint32_t spare5;
} config_t;

class MainWindow : public QMainWindow
{
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
    config_t config;
    bool isDebug = false;

    void requestSerialConfig();
    void getConfigFromUi();
    void setUiFromConfig();
    void openSerialPort();
    void closeSerialPort();

private slots:
    void buttonSerialPort();
    void buttonDebug();
    void readSerial();
    void readSerialConfig();
    QStringList fillPortsInfo();
    void writeSerialConfig();
    void openConfig();
    void saveConfig();
    void exitApp();
    void on_cbReceiver_currentIndexChanged(const QString &arg1);
    void on_cbEsc_currentIndexChanged(const QString &arg1);
    void on_gbEsc_toggled(bool arg1);
    void on_cbEscModel_currentIndexChanged(const QString &arg1);
    void on_cbBarometerType_currentIndexChanged(const QString &arg1);
    void on_gbVoltage1_toggled(bool arg1);
    void on_cbTemperature1_toggled(bool checked);
    void on_gbAltitude_toggled(bool arg1);
    void on_cbAirspeed_toggled(bool checked);
    void on_gbCurrent_toggled(bool arg1);
    void on_cbBarometerType_currentTextChanged(const QString &arg1);
    void on_cbEsc_currentTextChanged(const QString &arg1);
    void on_cbReceiver_currentTextChanged(const QString &arg1);
    void on_btCircuit_clicked();
    void resizeEvent(QResizeEvent* event);
    void on_gbGps_toggled(bool arg1);
    void on_cbCurrentAutoOffset_toggled(bool checked);
    void on_cbCurrentSensorType_currentTextChanged(const QString &arg1);
    void on_cbClockStretch_toggled(bool checked);
};
#endif // MAINWINDOW_H
