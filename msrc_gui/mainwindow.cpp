#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //ui->cbBoard->addItems({"Arduino Pro Mini", "Pololu ATmega328PB", "ATmega2560 Pro Mini", "Teensy 2", "Teensy LC", "Teensy 3.2"});
    ui->cbEsc->addItems({"Hobbywing V3", "Hobbywing V4 LV", "Hobbywing V4 HV", "Hobbywing V5 LV", "Hobbywing V5 HV", "PWM", "Castle link", "Kontronic"});
    ui->cbGpsBaudrate->addItems({"115200", "57600", "38400", "19200", "14400", "9600","4800"});
    ui->cbGpsBaudrate->setCurrentIndex(5);
    ui->cbReceiver->addItems({"Frsky Smartport", "Frsky D", "Spektrum XBUS", "Spektrum SRXL", "Flysky IBUS", "Futaba SBUS2", "Multiplex Sensor Bus", "Jeti Ex Bus"});
    for (uint8_t i = 0; i < 127; i++) {
        ui->cbAddress->addItem(QString::number(i));
    }
    ui->cbAddress->setCurrentIndex(0x77);
    connect(ui->btGenerate, SIGNAL (released()),this, SLOT (generateConfig()));
    connect(ui->actionGenerate_config, SIGNAL(triggered()), this, SLOT(generateConfig()));
    connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(exitApp()));
}

void MainWindow::exitApp()
{
    QApplication::quit();
}

void MainWindow::generateConfig()
{
    QString configString = "";

    configString +=""
        "#ifndef CONFIG_H"
        "\n#define CONFIG_H"
        "\n#include \"constants.h\""
        "\n";

    // Receiver
    configString += ""
        "\n/* Receiver protocol */"
        "\n#define RX_PROTOCOL ";
    switch (ui->cbReceiver->currentIndex())
    {
    case 0:
        configString += "RX_SMARTPORT";
        break;
    case 1:
        configString += "RX_FRSKY";
        break;
    case 2:
        configString += "RX_XBUS";
        break;
    case 3:
        configString += "RX_SRXL";
        break;
    case 4:
        configString += "RX_IBUS";
        break;
    case 5:
        configString += "RX_SBUS";
        break;
    case 6:
        configString += "RX_MULTIPLEX";
        break;
    case 7:
        configString += "RX_JETIEX";
        break;
    }

    /* Sensors */

    // ESC
    configString += ""
        "\n"
        "\n/* Sensors */"
        "\n#define CONFIG_ESC_PROTOCOL ";
    if (ui->gbEsc->isChecked())
    {
        switch (ui->cbEsc->currentIndex())
        {
        case 0:
            configString += "PROTOCOL_HW_V3";
            break;
        case 1:
            configString += "PROTOCOL_HW_V4_LV";
            break;
        case 2:
            configString += "PROTOCOL_HW_V4_HV";
            break;
        case 3:
            configString += "PROTOCOL_HW_V5_LV";
            break;
        case 4:
            configString += "PROTOCOL_HW_V5_HV";
            break;
        case 5:
            configString += "PROTOCOL_PWM";
            break;
        case 6:
            configString += "PROTOCOL_CASTLE";
            break;
        case 7:
            configString += "PROTOCOL_KONTRONIK";
            break;
        }
    }
    else
        configString +="PROTOCOL_NONE";

    // GPS
    configString += "\n#define CONFIG_GPS ";
    if (ui->gbGps->isChecked())
        configString += "true";
    else
        configString += "false";
    configString += "\n#define GPS_BAUD_RATE ";
    configString += ui->cbGpsBaudrate->currentText();

    // Voltage 1
    configString += " \n#define CONFIG_VOLTAGE1 ";
    if (ui->cbVoltage1->isChecked()) configString += "true";
    else configString += "false";

    // Voltage 2
    configString += " \n#define CONFIG_VOLTAGE2 ";
    if (ui->cbVoltage2->isChecked()) configString += "true";
    else configString += "false";

    // Temperature 1
    configString += " \n#define CONFIG_NTC1 ";
    if (ui->cbTemperature1->isChecked()) configString += "true";
    else configString += "false";

    // Temperature 2
    configString += " \n#define CONFIG_NTC2 ";
    if (ui->cbTemperature2->isChecked()) configString += "true";
    else configString += "false";

    // Current
    configString += " \n#define CONFIG_CURRENT ";
    if (ui->cbCurrent->isChecked()) configString += "true";
    else configString += "false";

    // Airspeed
    configString += " \n#define CONFIG_AIRSPEED ";
    if (ui->cbAirspeed->isChecked()) configString += "true";
    else configString += "false";

    // Altitude
    configString += " \n#define CONFIG_I2C1_TYPE ";
    if (ui->gbAltitude->isChecked()) configString += "I2C_BMP280";
    else configString += "I2C_NONE";
    configString += " \n#define CONFIG_I2C1_ADDRESS ";
    configString += ui->cbAddress->currentText();

    //
    configString += ""
        "\n/* Refresh rate in 0.1s (1 = 100ms) */"
        "\n#define CONFIG_REFRESH_RPM 1"
        "\n#define CONFIG_REFRESH_VOLT 1"
        "\n#define CONFIG_REFRESH_CURR 1"
        "\n#define CONFIG_REFRESH_TEMP 1"
        "\n#define CONFIG_REFRESH_DEF 1"
        "\n/* Averaging elements (1 = no averaging) */"
        "\n#define CONFIG_AVERAGING_ELEMENTS_RPM 3"
        "\n#define CONFIG_AVERAGING_ELEMENTS_VOLT 3"
        "\n#define CONFIG_AVERAGING_ELEMENTS_CURR 3"
        "\n#define CONFIG_AVERAGING_ELEMENTS_TEMP 3"
        "\n#define CONFIG_AVERAGING_ELEMENTS_DEF 3"
        "\n/* Analog multipliers */"
        "\n#define VOLTAGE1_MULTIPLIER 1"
        "\n#define VOLTAGE2_MULTIPLIER 1"
        "\n#define CURRENT_MULTIPLIER 1"
        "\n"
        "\n/* Pwm out */"
        "\n#define CONFIG_PWMOUT false"
        "\n#define PWMOUT_DUTY 0.5 // 0.5 = 50%"
        "\n"
        "\n/* Only smartport and opentx */"
        "\n#define SENSOR_ID 10 // Sensor Id"
        "\n#define DATA_ID 0x5000 // DataId (sensor type)"
        "\n//#define ESC_SIGNATURE // HW V4 signature (only smartport). This outputs esc signature and raw current to sensors 5100, 5101 and 5102"
        "\n#define CONFIG_LUA // Comment if not using lua script for configuration (only smartport)"
        "\n"
        "\n/* XBus */"
        "\n//#define XBUS_CLOCK_STRECH_SWITCH"
        "\n"
        "\n/* Use library I2C_T3 for Teensy LC/3.X */"
        "\n#define I2C_T3_TEENSY"
        "\n"
        "\n/* Add init delay for FlyFun ESC. Uncomment if the ESC doesn't arm */"
        "\n//#define ESC_INIT_DELAY 10000"
        "\n"
        "\n/* Force eeprom write */"
        "\n//#define FORCE_EEPROM_WRITE // Uncomment to force write eeprom as defined in config.h. Useful when using lua and eeprom is messed up. Reflash againg with line commented or config will be reset at power up"
        "\n"
        "\n/* Debug"
        "\n   Disconnect Vcc from the RC model to the Arduino"
        "\n   Do not connect at the same time Vcc from the model and usb (TTL)"
        "\n   Telemetry may not work properly in debug mode"
        "\n   Connect arduino Rx to TTL Tx for flashing, then if applicabe connect arduino Rx to esc or gps"
        "\n*/"
        "\n"
        "\n//#define DEBUG"
        "\n//#define DEBUG_PACKET"
        "\n//#define DEBUG_EEPROM_WRITE"
        "\n//#define DEBUG_EEPROM_READ"
        "\n"
        "\n//#define DEBUG_HW3"
        "\n//#define DEBUG_HW4"
        "\n//#define DEBUG_ESC_KONTRONIK"
        "\n//#define DEBUG_GPS"
        "\n//#define DEBUG_PWM"
        "\n//#define DEBUG_CASTLE"
        "\n//#define DEBUG_CASTLE_RX"
        "\n"
        "\n//#define SIM_RX"
        "\n//#define SIM_SENSORS"
        "\n//#define SIM_LUA_SEND"
        "\n//#define SIM_LUA_RECEIVE"
        "\n"
        "\n#endif";

    ui->txConfig->setText(configString);
}

MainWindow::~MainWindow()
{
    delete ui;
}
