#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //ui->cbBoard->addItems({"Arduino Pro Mini", "Pololu ATmega328PB", "ATmega2560 Pro Mini", "Teensy 2", "Teensy LC", "Teensy 3.2"});
    ui->cbEsc->addItems({"Hobbywing V3", "Hobbywing V4", "PWM", "Castle link", "Kontronic"});
    ui->cbGpsBaudrate->addItems({"115200", "57600", "38400", "19200", "14400", "9600","4800"});
    ui->cbGpsBaudrate->setCurrentIndex(5);
    ui->cbReceiver->addItems({"Frsky Smartport", "Frsky D", "Spektrum XBUS", "Spektrum SRXL", "Flysky IBUS", "Futaba SBUS2", "Multiplex Sensor Bus", "Jeti Ex Bus", "Hitec"});
    QComboBox *cbEscModel = ui->gbEsc->findChild<QComboBox *>("cbEscModel");
    cbEscModel->addItems({"Custom"});
    for (uint8_t i = 0; i < 127; i++) {
        QString hex;
        hex.setNum(i,16);
        ui->cbAddress->addItem("0x" + hex);
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
    case 8:
        configString += "RX_HITEC";
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
            configString += "PROTOCOL_HW_V4";
            break;
        case 2:
            configString += "PROTOCOL_PWM";
            break;
        case 3:
            configString += "PROTOCOL_CASTLE";
            break;
        case 4:
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
    if (ui->gbVoltage1->isChecked()) configString += "true";
    else configString += "false";

    // Voltage 2
    configString += " \n#define CONFIG_VOLTAGE2 ";
    if (ui->gbVoltage2->isChecked()) configString += "true";
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
    if (ui->gbCurrent->isChecked()) configString += "true";
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

    // Refresh rate
    configString += ""
                    "\n"
                    "\n/* Refresh rate in 0.1s (1 = 100ms) */";
    QGroupBox *gbRate = ui->gbReceiver->findChild<QGroupBox *>("gbRate");
    QSpinBox *sbRpmRate = ui->gbRate->findChild<QSpinBox *>("sbRpmRate");
    QSpinBox *sbVoltageRate = ui->gbRate->findChild<QSpinBox *>("sbVoltageRate");
    QSpinBox *sbCurrentRate = ui->gbRate->findChild<QSpinBox *>("sbCurrentRate");
    QSpinBox *sbTemperatureRate = ui->gbRate->findChild<QSpinBox *>("sbTemperatureRate");
    configString += "\n#define CONFIG_REFRESH_RPM " + QString::number(sbRpmRate->value());
    configString += "\n#define CONFIG_REFRESH_VOLT " + QString::number(sbVoltageRate->value());
    configString += "\n#define CONFIG_REFRESH_CURR " + QString::number(sbCurrentRate->value());
    configString += "\n#define CONFIG_REFRESH_TEMP " + QString::number(sbTemperatureRate->value());

    // Averaging
    configString += ""
                    "\n"
                    "\n/* Averaging elements (1 = no averaging) */";
    QSpinBox *sbRpmAvg = ui->gbAverage->findChild<QSpinBox *>("sbRpmAvg");
    QSpinBox *sbVoltageAvg = ui->gbAverage->findChild<QSpinBox *>("sbVoltageAvg");
    QSpinBox *sbCurrentAvg = ui->gbAverage->findChild<QSpinBox *>("sbCurrentAvg");
    QSpinBox *sbTemperatureAvg = ui->gbAverage->findChild<QSpinBox *>("sbTemperatureAvg");
    configString += "\n#define CONFIG_AVERAGING_ELEMENTS_RPM " + QString::number(sbRpmAvg->value());
    configString += "\n#define CONFIG_AVERAGING_ELEMENTS_VOLT " + QString::number(sbVoltageAvg->value());
    configString += "\n#define CONFIG_AVERAGING_ELEMENTS_CURR " + QString::number(sbCurrentAvg->value());
    configString += "\n#define CONFIG_AVERAGING_ELEMENTS_TEMP " + QString::number(sbTemperatureAvg->value());
    configString += "\n#define CONFIG_AVERAGING_ELEMENTS_DEF 1";

    // Analog Multipliers
    configString += ""
                    "\n"
                    "\n/* Analog multipliers */";
    QGroupBox *gbVoltage1 = ui->gbSensors->findChild<QGroupBox *>("gbVoltage1");
    QGroupBox *gbVoltage2 = ui->gbSensors->findChild<QGroupBox *>("gbVoltage2");
    QGroupBox *gbCurrent = ui->gbSensors->findChild<QGroupBox *>("gbCurrent");
    QDoubleSpinBox *sbVoltage1Mult = gbVoltage1->findChild<QDoubleSpinBox *>("sbVoltage1Mult");
    QDoubleSpinBox *sbVoltage2Mult = gbVoltage2->findChild<QDoubleSpinBox *>("sbVoltage2Mult");
    QDoubleSpinBox *sbCurrentMult = gbCurrent->findChild<QDoubleSpinBox *>("sbCurrentMult");
    configString += "\n#define VOLTAGE1_MULTIPLIER " + QString::number(sbVoltage1Mult->value());
    configString += "\n#define VOLTAGE2_MULTIPLIER " + QString::number(sbVoltage2Mult->value());
    configString += "\n#define CURRENT_MULTIPLIER " + QString::number(sbCurrentMult->value());

    // RPM Multipliers
    configString += ""
                    "\n"
                    "\n/* RPM multipliers (optional, this may be done in transmitter*/";
    QSpinBox *sbPairOfPoles = ui->gbRpmMultipliers->findChild<QSpinBox *>("sbPairOfPoles");
    QSpinBox *sbMainTeeth = ui->gbRpmMultipliers->findChild<QSpinBox *>("sbMainTeeth");
    QSpinBox *sbPinionTeeth = ui->gbRpmMultipliers->findChild<QSpinBox *>("sbPinionTeeth");
    configString += "\n#define RPM_PAIR_OF_POLES " + QString::number(sbPairOfPoles->value());
    configString += "\n#define RPM_PINION_TEETH " + QString::number(sbMainTeeth->value());
    configString += "\n#define RPM_MAIN_TEETH " + QString::number(sbPinionTeeth->value());

    //
    configString += ""
                    "\n"
                    "\n/* Pwm out */"
                    "\n#define CONFIG_PWMOUT false"
                    "\n#define PWMOUT_DUTY 0.5 // 0.5 = 50%"
                    "\n"
                    "\n/* Only smartport and opentx */"
                    "\n#define SENSOR_ID 10 // Sensor Id"
                    "\n#define DATA_ID 0x5000 // DataId (sensor type)"
                    "\n//#define ESC_SIGNATURE // HW V4 signature (only smartport). This outputs esc signature and raw current to sensors 5100, 5101 and 5102";

    // Lua Config
    QCheckBox *cbLuaConfig = ui->cbReceiver->findChild<QCheckBox *>("cbLuaConfig");
    if (ui->cbLuaConfig->isChecked()) configString += "\n#define CONFIG_LUA";
    else configString += "\n//#define CONFIG_LUA";

    // XBUS Clock stretch
    configString += ""
                    "\n"
                    "\n/* XBus */";
    QCheckBox *cbClockStretch = ui->cbReceiver->findChild<QCheckBox *>("cbClockStretch");
    if (ui->cbLuaConfig->isChecked()) configString += "\n//#define XBUS_CLOCK_STRECH_SWITCH";
    else configString += "\n//#define XBUS_CLOCK_STRECH_SWITCH";

    configString += ""
                    "\n"
                    "\n/* Use library I2C_T3 for Teensy LC/3.X */"
                    "\n#define I2C_T3_TEENSY";

    // HW V4/V5 parameters
    configString += ""
                    "\n"
                    "\n/* Add init delay for FlyFun ESC. Uncomment if the ESC doesn't arm */";
    QGroupBox *gbEsc = ui->gbSensors->findChild<QGroupBox *>("gbEsc");
    QGroupBox *gbEscParameters = gbEsc->findChild<QGroupBox *>("gbEscParameters");
    QCheckBox *cbInitDelay = gbEscParameters->findChild<QCheckBox *>("cbInitDelay");
    if (cbInitDelay->isChecked()) configString += "\n#define ESC_INIT_DELAY 10000";
    else configString += "\n//#define ESC_INIT_DELAY 10000";
    configString += ""
                    "\n"
                    "\n/* HW V4/V5 parameters */";
    QSpinBox *sbCurrentThresold = gbEscParameters->findChild<QSpinBox *>("sbCurrentThresold");
    QDoubleSpinBox *sbVoltageDivisor = gbEscParameters->findChild<QDoubleSpinBox *>("sbVoltageDivisor");
    QDoubleSpinBox *sbCurrentMultiplier = gbEscParameters->findChild<QDoubleSpinBox *>("sbCurrentMultiplier");
    QSpinBox *sbCurrentMax = gbEscParameters->findChild<QSpinBox *>("sbCurrentMax");
    configString += "\n#define CURRENT_THRESHOLD " + QString::number(sbCurrentThresold->value());
    configString += "\n#define ESCHW4_DIVISOR " + QString::number(sbVoltageDivisor->value());
    configString += "\n#define ESCHW4_AMPGAIN " + QString::number(sbCurrentMultiplier->value());
    configString += "\n#define ESCHW4_CURRENT_MAX " + QString::number(sbCurrentMax->value());

    configString += ""
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
                    "\n//#define DEBUG_SBUS_MS"
                    "\n//#define DEBUG_EEPROM_WRITE"
                    "\n//#define DEBUG_EEPROM_READ"
                    "\n"
                    "\n//#define DEBUG_HW3"
                    "\n//#define DEBUG_HW4"
                    "\n//#define DEBUG_ESC_KONTRONIK"
                    "\n//#define DEBUG_APDF"
                    "\n//#define DEBUG_APDHV"
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

void MainWindow::on_cbReceiver_currentIndexChanged(const QString &arg1)
{
    QGroupBox *gbRate = ui->gbReceiver->findChild<QGroupBox *>("gbRate");
    if (arg1 == "Spektrum XBUS")
    {
        ui->cbClockStretch->setVisible(true);
        gbRate->setVisible(false);
    }
    else
    {
        ui->cbClockStretch->setVisible(false);
        gbRate->setVisible(true);
    }

    if (arg1 == "Frsky D" || arg1 == "Frsky Smartport")
    {
        ui->cbLuaConfig->setVisible(true);
        gbRate->setVisible(true);
    }
    else
    {
        ui->cbLuaConfig->setVisible(false);
        gbRate->setVisible(false);
    }

}


void MainWindow::on_cbEsc_currentIndexChanged(const QString &arg1)
{
    QGroupBox *gbEscParameters = ui->gbEsc->findChild<QGroupBox *>("gbEscParameters");
    if (arg1 == "Hobbywing V4")
        gbEscParameters->setVisible(true);
    else
        gbEscParameters->setVisible(false);

}

