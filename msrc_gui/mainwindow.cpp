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
    ui->cbEscModel->addItems({"Platinum PRO v4 25/40/60", "Platinum PRO v4 80A", "Platinum PRO v4 100A", "Platinum PRO v4 120A", "Platinum PRO v4 130A-HV", "Platinum PRO v4 150A", "Platinum PRO v4 200A-HV",
                             "FlyFun 30/40A", "FlyFun 60A", "FlyFun 80A", "FlyFun 120A", "FlyFun 110A-HV", "FlyFun 130A-HV",  "FlyFun 160A-HV"});
    ui->cbBarometerType->addItems({"BMP280", "MS5611"});
    ui->cbAltitudeFilter->addItems({"Low", "Medium", "High"});
    ui->cbAltitudeFilter->setCurrentIndex(2);
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
    switch (ui->cbReceiver->currentIndex()) {
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
    if (ui->gbEsc->isChecked()) {
        switch (ui->cbEsc->currentIndex()) {
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
    } else
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
    if (ui->gbAltitude->isChecked()) configString += ui->cbBarometerType->currentText();
    else configString += "I2C_NONE";
    configString += " \n#define CONFIG_I2C1_ADDRESS ";
    configString += ui->cbAddress->currentText();

    // Refresh rate
    configString += ""
                    "\n"
                    "\n/* Refresh rate in 0.1s (1 = 100ms) */";
    QSpinBox *sbRpmRate = ui->gbRate->findChild<QSpinBox *>("sbRpmRate");
    QSpinBox *sbVoltageRate = ui->gbRate->findChild<QSpinBox *>("sbVoltageRate");
    QSpinBox *sbCurrentRate = ui->gbRate->findChild<QSpinBox *>("sbCurrentRate");
    QSpinBox *sbTemperatureRate = ui->gbRate->findChild<QSpinBox *>("sbTemperatureRate");
    configString += "\n#define CONFIG_REFRESH_RPM " + QString::number(sbRpmRate->value() / 100);
    configString += "\n#define CONFIG_REFRESH_VOLT " + QString::number(sbVoltageRate->value() / 100);
    configString += "\n#define CONFIG_REFRESH_CURR " + QString::number(sbCurrentRate->value() / 100);
    configString += "\n#define CONFIG_REFRESH_TEMP " + QString::number(sbTemperatureRate->value() / 100);

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
    QDoubleSpinBox *sbVoltage1Mult = ui->gbVoltage1->findChild<QDoubleSpinBox *>("sbVoltage1Mult");
    QDoubleSpinBox *sbVoltage2Mult = ui->gbVoltage2->findChild<QDoubleSpinBox *>("sbVoltage2Mult");
    QDoubleSpinBox *sbCurrentMult = ui->gbCurrent->findChild<QDoubleSpinBox *>("sbCurrentMult");
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

    // Altitude filter
    configString += ""
                    "\n"
                    "\n/* BMP filter. Higher filter = lower noise: 1 - low, 2 - medium, 3 - high */"
                    "\n#define BMP280_FILTER " + QString::number(ui->cbAltitudeFilter->currentIndex() + 1);

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
    if (ui->cbLuaConfig->isChecked()) configString += "\n#define CONFIG_LUA";
    else configString += "\n//#define CONFIG_LUA";

    // XBUS Clock stretch
    configString += ""
                    "\n"
                    "\n/* XBus */";
    if (ui->cbClockStretch->isChecked()) configString += "\n#define XBUS_CLOCK_STRECH_SWITCH";
    else configString += "\n//#define XBUS_CLOCK_STRECH_SWITCH";

    configString += ""
                    "\n"
                    "\n/* Use library I2C_T3 for Teensy LC/3.X */"
                    "\n#define I2C_T3_TEENSY";

    // HW V4/V5 parameters
    configString += ""
                    "\n"
                    "\n/* Add init delay for FlyFun ESC. Uncomment if the ESC doesn't arm */";
    QCheckBox *cbInitDelay = ui->gbEscParameters->findChild<QCheckBox *>("cbInitDelay");
    if (cbInitDelay->isChecked()) configString += "\n#define ESC_INIT_DELAY 10000";
    else configString += "\n//#define ESC_INIT_DELAY 10000";
    configString += ""
                    "\n"
                    "\n/* HW V4/V5 parameters */";
    QSpinBox *sbCurrentThresold = ui->gbEscParameters->findChild<QSpinBox *>("sbCurrentThresold");
    QDoubleSpinBox *sbVoltageDivisor = ui->gbEscParameters->findChild<QDoubleSpinBox *>("sbVoltageDivisor");
    QDoubleSpinBox *sbCurrentMultiplier = ui->gbEscParameters->findChild<QDoubleSpinBox *>("sbCurrentMultiplier");
    QSpinBox *sbCurrentMax = ui->gbEscParameters->findChild<QSpinBox *>("sbCurrentMax");
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
    if (arg1 == "Spektrum XBUS") {
        ui->cbClockStretch->setVisible(true);
        gbRate->setVisible(false);
    } else {
        ui->cbClockStretch->setVisible(false);
        gbRate->setVisible(true);
    }

    if (arg1 == "Frsky D" || arg1 == "Frsky Smartport") {
        ui->cbLuaConfig->setVisible(true);
        gbRate->setVisible(true);
    } else {
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

void MainWindow::on_gbEsc_toggled(bool arg1)
{
    QGroupBox *gbRpmMultipliers = ui->gbEsc->findChild<QGroupBox *>("gbRpmMultipliers");
    if (arg1 ==  true)
        gbRpmMultipliers->setVisible(true);
    else
        gbRpmMultipliers->setVisible(false);

}

void MainWindow::on_btCopy_clicked()
{
    QClipboard *clipboard = QGuiApplication::clipboard();
    clipboard->setText(ui->txConfig->toPlainText());
}

void MainWindow::on_cbEscModel_currentIndexChanged(const QString &arg1)
{
    //QSpinBox *sbCurrentThresold = ui->gbEsc->findChild<QSpinBox *>("sbCurrentThresold");
    QDoubleSpinBox *sbVoltageDivisor = ui->gbEsc->findChild<QDoubleSpinBox *>("sbVoltageDivisor");
    QDoubleSpinBox *sbCurrentMultiplier = ui->gbEsc->findChild<QDoubleSpinBox *>("sbCurrentMultiplier");
    QSpinBox *sbCurrentMax = ui->gbEsc->findChild<QSpinBox *>("sbCurrentMax");
    QCheckBox *cbInitDelay = ui->gbEsc->findChild<QCheckBox *>("cbInitDelay");

    if (arg1 == "Platinum PRO v4 25/40/60") {
        sbVoltageDivisor->setValue(11);
        sbCurrentMultiplier->setValue(0);
        sbCurrentMax->setValue(0);
        cbInitDelay->setChecked(false);
    }
    else if (arg1 == "Platinum PRO v4 80A") {
        sbVoltageDivisor->setValue(11);
        sbCurrentMultiplier->setValue(8);
        sbCurrentMax->setValue(100);
        cbInitDelay->setChecked(false);
    }
    else if (arg1 == "Platinum PRO v4 100A") {
        sbVoltageDivisor->setValue(11);
        sbCurrentMultiplier->setValue(9);
        sbCurrentMax->setValue(120);
        cbInitDelay->setChecked(false);
    }
    else if (arg1 == "Platinum PRO v4 120A") {
        sbVoltageDivisor->setValue(11);
        sbCurrentMultiplier->setValue(10);
        sbCurrentMax->setValue(140);
        cbInitDelay->setChecked(false);
    }
    else if (arg1 == "Platinum PRO v4 150A") {
        sbVoltageDivisor->setValue(15.75);
        sbCurrentMultiplier->setValue(10);
        sbCurrentMax->setValue(170);
        cbInitDelay->setChecked(false);
    }
    else if (arg1 == "Platinum PRO v4 130A-HV") {
        sbVoltageDivisor->setValue(21);
        sbCurrentMultiplier->setValue(11.3);
        sbCurrentMax->setValue(150);
        cbInitDelay->setChecked(false);
    }
    else if (arg1 == "Platinum PRO v4 200A-HV") {
        sbVoltageDivisor->setValue(21);
        sbCurrentMultiplier->setValue(16.9);
        sbCurrentMax->setValue(220);
        cbInitDelay->setChecked(false);
    }
    else if (arg1 == "FlyFun 30/40A") {
        sbVoltageDivisor->setValue(11);
        sbCurrentMultiplier->setValue(0);
        sbCurrentMax->setValue(0);
        cbInitDelay->setChecked(true);
    }
    else if (arg1 == "FlyFun 60A") {
        sbVoltageDivisor->setValue(11);
        sbCurrentMultiplier->setValue(6);
        sbCurrentMax->setValue(80);
        cbInitDelay->setChecked(true);
    }
    else if (arg1 == "FlyFun 80A") {
        sbVoltageDivisor->setValue(15.75);
        sbCurrentMultiplier->setValue(12.4);
        sbCurrentMax->setValue(100);
        cbInitDelay->setChecked(true);
    }
    else if (arg1 == "FlyFun 120A") {
        sbVoltageDivisor->setValue(21);
        sbCurrentMultiplier->setValue(15);
        sbCurrentMax->setValue(140);
        cbInitDelay->setChecked(true);
    }
    else if (arg1 == "FlyFun 110A-HV") {
        sbVoltageDivisor->setValue(21);
        sbCurrentMultiplier->setValue(15);
        sbCurrentMax->setValue(130);
        cbInitDelay->setChecked(true);
    }
    else if (arg1 == "FlyFun 130A-HV") {
        sbVoltageDivisor->setValue(21);
        sbCurrentMultiplier->setValue(15);
        sbCurrentMax->setValue(150);
        cbInitDelay->setChecked(true);
    }
    else if (arg1 == "FlyFun 160A-HV") {
        sbVoltageDivisor->setValue(21);
        sbCurrentMultiplier->setValue(15);
        sbCurrentMax->setValue(180);
        cbInitDelay->setChecked(true);
    }
}

void MainWindow::on_cbBarometerType_currentIndexChanged(const QString &arg1)
{
    QLabel *lbAltitudeFilter = ui->gbAltitude->findChild<QLabel *>("lbAltitudeFilter");
    if (arg1 == "BMP280")
    {
        ui->cbAltitudeFilter->setVisible(true);
        lbAltitudeFilter->setVisible(true);
    }
    else
    {
        ui->cbAltitudeFilter->setVisible(false);
        lbAltitudeFilter->setVisible(false);
    }
}

