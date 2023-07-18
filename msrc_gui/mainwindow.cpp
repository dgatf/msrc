#include "mainwindow.h"
#include "circuitdialog.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), serial(new QSerialPort())
{
    ui->setupUi(this);
    ui->cbEsc->addItems({"Hobbywing V3", "Hobbywing V4", "PWM", "Castle Link",
                         "Kontronic", "APD F", "APD HV"});
    ui->cbGpsBaudrate->addItems(
                {"115200", "57600", "38400", "19200", "14400", "9600", "4800"});
    ui->cbGpsBaudrate->setCurrentIndex(5);
    ui->cbReceiver->addItems({"Frsky Smartport", "Frsky D", "Spektrum XBUS",
                              "Spektrum SRXL", "Flysky IBUS", "Futaba SBUS2",
                              "Multiplex Sensor Bus", "Jeti Ex Bus", "Hitec"});
    ui->cbEscModel->addItems({
                                 "Platinum PRO v4 25/40/60", "Platinum PRO v4 80A",
                                 "Platinum PRO v4 100A", "Platinum PRO v4 120A",
                                 "Platinum PRO v4 130A-HV", "Platinum PRO v4 150A",
                                 "Platinum PRO v4 200A-HV", "FlyFun 30/40A", "FlyFun 60A", "FlyFun 80A",
                                 "FlyFun 120A", "FlyFun 110A-HV", "FlyFun 130A-HV", "FlyFun 160A-HV"});
    ui->cbCurrentSensorType->addItems({"Hall effect", "Shunt resistor"});
    ui->cbCurrentAutoOffset->setChecked(true);
    ui->cbBarometerType->addItems({"BMP280", "MS5611", "BMP180"});
    ui->cbAltitudeFilter->addItems({"Low", "Medium", "High"});
    ui->cbAltitudeFilter->setCurrentIndex(2);
    ui->cbSpeedUnitsGps->addItems({"km/h", "kts"});
    ui->lbQuiescentVoltage->setText("Zero current output voltage, V<sub>IOUT</sub> (V)");
    for(uint8_t i = 0; i < 127; i++) {
        QString hex;
        hex.setNum(i, 16);
        ui->cbAddress->addItem("0x" + hex);
    }
    ui->cbAddress->setCurrentIndex(0x77);

    connect(ui->btConnect, SIGNAL(released()),  this, SLOT(buttonSerialPort()));
    connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(exitApp()));
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerialConfig);
    connect(ui->btUpdate, SIGNAL(released()), this, SLOT(writeSerialConfig()));
    connect(ui->actionUpdateConfig, SIGNAL(triggered()), this, SLOT(writeSerialConfig()));

    connect(ui->actionOpen, SIGNAL(triggered()), this, SLOT(openConfig()));
    connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(saveConfig()));

    ui->lbCircuit->resize(621, 400); //(ui->lbCircuit->parentWidget()->width(),
    // ui->lbCircuit->parentWidget()->height());
    generateCircuit(ui->lbCircuit);

    fillPortsInfo();

    statusBar()->showMessage("Not connected");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::generateCircuit(QLabel *label)
{
    QSize *size = new QSize(label->width(), label->height());
    QPixmap *pix = new QPixmap(*size);
    QPainter *paint = new QPainter(pix);
    QImage image;

    image.load(":/res/rp2040_zero.png");
    paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));

    if(ui->gbCurrent->isChecked()) {
        image.load(":/res/current_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
    }

    if(ui->gbVoltage1->isChecked()) {
        image.load(":/res/voltage_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
    }

    if(ui->cbTemperature1->isChecked()) {
        image.load(":/res/ntc_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0),
                         image.scaled(*size, Qt::IgnoreAspectRatio));
    }

    if(ui->cbAirspeed->isChecked()) {
        image.load(":/res/airspeed_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0),
                         image.scaled(*size, Qt::IgnoreAspectRatio));
    }

    if(ui->gbGps->isChecked()) {
        image.load(":/res/gps_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0),
                         image.scaled(*size, Qt::IgnoreAspectRatio));
    }

    if(ui->gbEsc->isChecked()) {
        if(ui->cbEsc->currentText() == "Hobbywing V3" ||
                ui->cbEsc->currentText() == "Hobbywing V4" ||
                ui->cbEsc->currentText() == "Kontronic" ||
                ui->cbEsc->currentText() == "APD F" ||
                ui->cbEsc->currentText() == "APD HV")
            image.load(":/res/esc_rp2040_zero.png");
        else if(ui->cbEsc->currentText() == "PWM")
            image.load(":/res/pwm_rp2040_zero.png");
        else if(ui->cbEsc->currentText() == "Castle Link")
            image.load(":/res/castle_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0),
                         image.scaled(*size, Qt::IgnoreAspectRatio));
    }

    if(ui->gbAltitude->isChecked()) {
        image.load(":/res/vario_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0),
                         image.scaled(*size, Qt::IgnoreAspectRatio));
    }

    if(ui->cbReceiver->currentText() == "Frsky D") {
        image.load(":/res/receiver_frsky_d_rp2040_zero.png");
    } else if(ui->cbReceiver->currentText() == "Spektrum XBUS") {
        image.load(":/res/receiver_xbus_rp2040_zero.png");
    } else if(ui->cbReceiver->currentText() == "Hitec") {
        image.load(":/res/receiver_hitec_rp2040_zero.png");
    } else {
        image.load(":/res/receiver_serial_rp2040_zero.png");
    }

    paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));

    if (ui->cbReceiver->currentText() == "Spektrum XBUS" && ui->cbClockStretch->isChecked() == true) {
        image.load(":/res/clock_stretch_xbus_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
    }

    label->setPixmap(*pix);
}

void MainWindow::openConfig()
{
    QString filename = QFileDialog::getOpenFileName(this, "Open Config", "", "Config Files (*.cfg)");
    QFile file(filename);

    if(file.open(QIODevice::ReadOnly)) {
        file.read((char*)&config, sizeof(config_t));
        file.close();
        setUiFromConfig();
    }
}

void MainWindow::saveConfig()
{
    QString filename = QFileDialog::getSaveFileName(this, "Save Config", "", "Config Files (*.cfg)");

    /*QJsonObject json_obj;
    json_obj["version"] = config.version;
    json_obj["rx_protocol"] = config.rx_protocol;
    QJsonDocument json_doc(json_obj);
    QString json_string = json_doc.toJson();*/
    QFile file(filename);
    if(file.open(QIODevice::WriteOnly)) {
        //file.write(json_string.toLocal8Bit());
        //file.close();

        getConfigFromUi();
        file.write((char*)&config, sizeof(config_t));

        //DataPacket pin{1000, 123, "dataString"};
        //QDataStream out(&file);
        //out << pin;

    }
}

void MainWindow::buttonSerialPort()
{
    if (isConnected)
        closeSerialPort();
    else
        openSerialPort();
}

void MainWindow::openSerialPort()
{
    QString portName = ui->cbPortList->currentText();
    serial->setPortName(portName);
    serial->setBaudRate(QSerialPort::BaudRate::Baud115200);
    serial->setDataBits(QSerialPort::DataBits::Data8);
    serial->setParity(QSerialPort::Parity::NoParity);
    serial->setStopBits(QSerialPort::StopBits::OneStop);
    serial->setFlowControl(QSerialPort::FlowControl::NoFlowControl);
    if(serial->open(QIODevice::ReadWrite)) {
        statusBar()->showMessage("Connected " + portName);
        isConnected = true;
        requestSerialConfig();
        serial->setDataTerminalReady(true);
        ui->btUpdate->setEnabled(true);
        ui->btConnect->setText("Disconnect");
        ui->actionUpdateConfig->setEnabled(true);
        ui->saConfig->setEnabled(true);
    } else {
        statusBar()->showMessage("Not connected: " + serial->errorString());
        isConnected = false;
        ui->btUpdate->setEnabled(false);
        ui->btConnect->setText("Connect");
        ui->actionUpdateConfig->setEnabled(false);
        ui->saConfig->setEnabled(false);
    }
}

void MainWindow::closeSerialPort()
{
    serial->close();
    statusBar()->showMessage("Not connected");
    isConnected = false;
    ui->btUpdate->setEnabled(false);
    ui->btConnect->setText("Connect");
    ui->actionUpdateConfig->setEnabled(false);
    ui->saConfig->setEnabled(false);
}

void MainWindow::readSerialConfig()
{
    if(serial->bytesAvailable() == sizeof(config_t) + 2) {
        data = serial->readAll();
        if(data.at(0) == 0x30 && data.at(1) == 0x32) {
            memcpy(&config, data.data() + 2, sizeof(config_t));
            setUiFromConfig();
        }
    }
}

void MainWindow::writeSerialConfig()
{
    if(!isConnected) return;
    getConfigFromUi();
    char header = 0x30;
    serial->write(&header, 1);
    char command = 0x30;
    serial->write(&command, 1);
    serial->write((char *)&config, sizeof(config_t));
}

void MainWindow::setUiFromConfig()
{
    /* Receiver protocol */

    ui->cbReceiver->setCurrentIndex(config.rx_protocol);

    /* Sensors */

    // ESC

    if(config.esc_protocol == esc_protocol_t::ESC_NONE)
        ui->gbEsc->setChecked(false);
    else {
        ui->gbEsc->setChecked(true);
        ui->cbEsc->setCurrentIndex(config.esc_protocol - 1);
    }

    // GPS

    ui->gbGps->setChecked(config.enable_gps);
    ui->cbGpsBaudrate->currentText() = config.gps_baudrate;

    // Voltage 1

    ui->gbVoltage1->setChecked(config.enable_analog_voltage);

    // Voltage 2

    //ui->gbVoltage2->setChecked(config.enable_analog_voltage2);

    // Temperature 1

    ui->cbTemperature1->setChecked(config.enable_analog_ntc);

    // Temperature 2

    //ui->cbTemperature2->setChecked(config.enable_analog_ntc2);


    // Current

    ui->gbCurrent->setChecked(config.enable_analog_current);
    ui->cbCurrentSensorType->setCurrentIndex(config.analog_current_type);
    ui->cbCurrentAutoOffset->setChecked(config.analog_current_autoffset);
    ui->sbCurrentSens->setValue(config.analog_current_sensitivity);
    ui->sbQuiescentVoltage->setValue(config.analog_current_quiescent_voltage);
    ui->sbAnalogCurrentMultiplier->setValue(config.analog_current_multiplier);

    // Airspeed

    ui->cbAirspeed->setChecked(config.enable_analog_airspeed);

    // Altitude

    if(config.i2c_module == i2c_module_t::I2C_NONE)
        ui->gbAltitude->setChecked(false);
    else
        ui->gbAltitude->setChecked(true);
    ui->cbBarometerType->setCurrentIndex(config.i2c_module - 1);
    ui->cbAddress->setCurrentIndex(config.i2c_address);
    ui->cbAltitudeFilter->setCurrentIndex(config.bmp280_filter - 1);
    ui->cbVarioAutoOffset->setChecked(config.vario_auto_offset);

    // Refresh rate

    ui->sbRpmRate->setValue(config.refresh_rate_rpm);
    ui->sbVoltageRate->setValue(config.refresh_rate_voltage);
    ui->sbCurrentRate->setValue(config.refresh_rate_current);
    ui->sbTemperatureRate->setValue(config.refresh_rate_temperature);
    ui->sbGpsRate->setValue(config.refresh_rate_gps);
    ui->sbConsumptionRate->setValue(config.refresh_rate_consumption);
    ui->sbVarioRate->setValue(config.refresh_rate_vario);
    ui->sbAirspeedRate->setValue(config.refresh_rate_airspeed);
    //ui->sbDefRate->setValue(config.refresh_rate_def);

    // Averaging

    ui->sbRpmAvg->setValue(qRound(2 / config.alpha_rpm - 1));
    ui->sbVoltageAvg->setValue(qRound(2 / config.alpha_voltage - 1));
    ui->sbCurrentAvg->setValue(qRound(2 / config.alpha_current - 1));
    ui->sbTemperatureAvg->setValue(qRound(2 / config.alpha_temperature - 1));
    ui->sbVarioAvg->setValue(qRound(2 / config.alpha_vario - 1));
    ui->sbAirspeedAvg->setValue(qRound(2 / config.alpha_airspeed - 1));

    // Analog voltage multipliers

    ui->sbVoltage1Mult->setValue(config.analog_voltage_multiplier);
    //config.analog_voltage2_multiplier = ui->sbVoltage2Mult->value();

    // RPM Multipliers

    ui->sbPairOfPoles->setValue(config.pairOfPoles);
    ui->sbMainTeeth->setValue(config.mainTeeth);
    ui->sbPinionTeeth->setValue(config.pinionTeeth);

    // PWM out

    ui->cbPwmOut->setChecked(config.enable_pwm_out);

    // Smartport

    //config.smartport_data_id;
    //config.smartport_sensor_id;

    // XBUS Clock stretch

    ui->cbClockStretch->setChecked(config.xbus_clock_stretch);

    // Ibus

    ui->cbAlternativeCoordinates->setChecked(config.ibus_alternative_coordinates);

    // Jeti Ex

    if(config.jeti_gps_speed_units_kmh == true)
        ui->cbSpeedUnitsGps->setCurrentIndex(0);
    else
        ui->cbSpeedUnitsGps->setCurrentIndex(1);

    // HW V4/V5 parameters

    ui->cbInitDelay->setChecked(config.enable_esc_hw4_init_delay);
    //config.esc_hw4_init_delay_duration = 10000;
    ui->sbCurrentThresold->setValue(config.esc_hw4_current_thresold);
    ui->sbVoltageDivisor->setValue(config.esc_hw4_divisor);
    ui->sbCurrentMultiplier->setValue(config.esc_hw4_ampgain);
    ui->sbCurrentMax->setValue(config.esc_hw4_current_max);

    // Debug

    ui->cbDebug->setChecked(config.debug);

}

void MainWindow::getConfigFromUi()
{

    /* Config version  */

    config.version = CONFIG_VERSION;

    /* Receiver protocol */

    config.rx_protocol = (rx_protocol_t)ui->cbReceiver->currentIndex();

    /* Sensors */

    // ESC

    if(ui->gbEsc->isChecked())
        config.esc_protocol = (esc_protocol_t)(ui->cbEsc->currentIndex() + 1);
    else
        config.esc_protocol = esc_protocol_t::ESC_NONE;

    // GPS

    config.enable_gps = ui->gbGps->isChecked();
    config.gps_baudrate = ui->cbGpsBaudrate->currentText().toInt();

    // Voltage

    config.enable_analog_voltage = ui->gbVoltage1->isChecked();
    //config.enable_analog_voltage2 = ui->gbVoltage2->isChecked();

    // Temperature

    config.enable_analog_ntc = ui->cbTemperature1->isChecked();
    //config.enable_analog_ntc2 = ui->cbTemperature2->isChecked();

    // Current

    config.enable_analog_current = ui->gbCurrent->isChecked();
    config.analog_current_type = (analog_current_type_t)ui->cbCurrentSensorType->currentIndex();
    config.analog_current_autoffset = ui->cbCurrentAutoOffset->isChecked();
    config.analog_current_sensitivity = ui->sbCurrentSens->value();
    config.analog_current_quiescent_voltage = ui->sbQuiescentVoltage->value();
    config.analog_current_multiplier = ui->sbAnalogCurrentMultiplier->value();

    // Airspeed

    config.enable_analog_airspeed = ui->cbAirspeed->isChecked();

    // Altitude

    if(ui->gbAltitude->isChecked())
        config.i2c_module = (i2c_module_t)(ui->cbBarometerType->currentIndex() + 1);
    else
        config.i2c_module = i2c_module_t::I2C_NONE;
    config.i2c_address = ui->cbAddress->currentIndex();
    config.bmp280_filter = ui->cbAltitudeFilter->currentIndex() + 1;
    config.vario_auto_offset = ui->cbVarioAutoOffset->isChecked();

    // Refresh rate

    config.refresh_rate_rpm = ui->sbRpmRate->value();
    config.refresh_rate_voltage = ui->sbVoltageRate->value();
    config.refresh_rate_current = ui->sbCurrentRate->value();
    config.refresh_rate_temperature = ui->sbTemperatureRate->value();
    config.refresh_rate_gps = ui->sbGpsRate->value();
    config.refresh_rate_consumption = ui->sbConsumptionRate->value();
    config.refresh_rate_vario = ui->sbVarioRate->value();
    config.refresh_rate_airspeed = ui->sbAirspeedRate->value();

    // Averaging

    config.alpha_rpm = 2.0 / (ui->sbRpmAvg->value() + 1);
    config.alpha_voltage = 2.0 / (ui->sbVoltageAvg->value() + 1);
    config.alpha_current = 2.0 / (ui->sbCurrentAvg->value() + 1);
    config.alpha_temperature = 2.0 / (ui->sbTemperatureAvg->value() + 1);
    config.alpha_vario = 2.0 / (ui->sbVarioAvg->value() + 1);
    config.alpha_airspeed = 2.0 / (ui->sbAirspeedAvg->value() + 1);

    // Analog voltage multipliers

    config.analog_voltage_multiplier = ui->sbVoltage1Mult->value();
    //config.analog_voltage2_multiplier = ui->sbVoltage2Mult->value()

    // Analog current sensor

    if(ui->cbCurrentSensorType->currentIndex() == analog_current_type_t::CURRENT_TYPE_HALL) {
        if(ui->cbCurrentAutoOffset->isChecked()) {
            config.analog_current_autoffset = true;
            config.analog_current_offset = 0;
        } else {
            config.analog_current_autoffset = false;
            config.analog_current_offset = ui->sbQuiescentVoltage->value();
        }
        config.analog_current_multiplier = 1000 / ui->sbCurrentSens->value();
    } else if(ui->cbCurrentSensorType->currentIndex() == analog_current_type_t::CURRENT_TYPE_SHUNT) {
        config.analog_current_autoffset = false;
        config.analog_current_offset = 0;
        config.analog_current_multiplier = ui->sbAnalogCurrentMultiplier->value();
    }
    config.analog_current_type = (analog_current_type_t)ui->cbCurrentSensorType->currentIndex();

    // RPM Multipliers

    config.pairOfPoles = ui->sbPairOfPoles->value();
    config.mainTeeth = ui->sbMainTeeth->value();
    config.pinionTeeth = ui->sbPinionTeeth->value();
    config.rpm_multiplier = config.pinionTeeth / (config.mainTeeth * config.pairOfPoles);

    // PWM out

    config.enable_pwm_out = ui->cbPwmOut->isChecked();

    // Smartport

    //config.smartport_data_id = 0x5000;
    //config.smartport_sensor_id = 10;

    // XBUS Clock stretch

    config.xbus_clock_stretch = ui->cbClockStretch->isChecked();

    // Jeti Ex

    if(ui->cbSpeedUnitsGps->currentText() == "km/h")
        config.jeti_gps_speed_units_kmh = true;
    else
        config.jeti_gps_speed_units_kmh = false;

    // Ibus

    config.ibus_alternative_coordinates = ui->cbAlternativeCoordinates->isChecked();

    // HW V4/V5 parameters

    config.enable_esc_hw4_init_delay = ui->cbInitDelay->isChecked();
    //config.esc_hw4_init_delay_duration = 10000;

    config.esc_hw4_current_thresold = ui->sbCurrentThresold->value();
    config.esc_hw4_divisor = ui->sbVoltageDivisor->value();
    config.esc_hw4_ampgain = ui->sbCurrentMultiplier->value();
    config.esc_hw4_current_max = ui->sbCurrentMax->value();

    // Debug

    config.debug = ui->cbDebug->isChecked();
}

void MainWindow::requestSerialConfig()
{
    //serial->readAll();
    char header = 0x30;
    serial->write(&header, 1);
    char command = 0x31;
    serial->write(&command, 1);
}

QStringList MainWindow::fillPortsInfo()
{
    const QList<QSerialPortInfo> infos = QSerialPortInfo::availablePorts();
    QStringList list;
    for(const QSerialPortInfo& info : infos) {
        list.append(info.portName());
        ui->cbPortList->addItem(info.portName());
    }
    return list;
}

void MainWindow::exitApp()
{
    QApplication::quit();
}

void MainWindow::on_cbReceiver_currentIndexChanged(const QString &arg1)
{
    QGroupBox *gbRate = ui->gbReceiver->findChild<QGroupBox *>("gbRate");
    if(arg1 == "Spektrum XBUS") {
        ui->cbClockStretch->setVisible(true);
        gbRate->setVisible(false);
    } else {
        ui->cbClockStretch->setVisible(false);
        gbRate->setVisible(true);
    }

    if(arg1 == "Frsky Smartport" || arg1 == "Frsky D") {
        gbRate->setVisible(true);
    } else {
        gbRate->setVisible(false);
    }

    if(arg1 == "Flysky IBUS") {
        ui->cbAlternativeCoordinates->setVisible(true);
    } else {
        ui->cbAlternativeCoordinates->setVisible(false);
    }

    QLabel *lbSpeedUnitsGps = ui->gbGps->findChild<QLabel *>("lbSpeedUnitsGps");
    if(arg1 == "Jeti Ex Bus") {
        ui->cbSpeedUnitsGps->setVisible(true);
        lbSpeedUnitsGps->setVisible(true);
    } else {
        ui->cbSpeedUnitsGps->setVisible(false);
        lbSpeedUnitsGps->setVisible(false);
    }
}

void MainWindow::on_cbEsc_currentIndexChanged(const QString &arg1)
{
    QGroupBox *gbEscParameters =
            ui->gbEsc->findChild<QGroupBox *>("gbEscParameters");
    if(arg1 == "Hobbywing V4")
        gbEscParameters->setVisible(true);
    else
        gbEscParameters->setVisible(false);
}

void MainWindow::on_cbEscModel_currentIndexChanged(const QString &arg1)
{
    if(arg1 == "Platinum PRO v4 25/40/60") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(0);
        ui->sbCurrentMax->setValue(0);
        ui->cbInitDelay->setChecked(false);
    } else if(arg1 == "Platinum PRO v4 80A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(8);
        ui->sbCurrentMax->setValue(100);
        ui->cbInitDelay->setChecked(false);
    } else if(arg1 == "Platinum PRO v4 100A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(9);
        ui->sbCurrentMax->setValue(120);
        ui->cbInitDelay->setChecked(false);
    } else if(arg1 == "Platinum PRO v4 120A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(10);
        ui->sbCurrentMax->setValue(140);
        ui->cbInitDelay->setChecked(false);
    } else if(arg1 == "Platinum PRO v4 150A") {
        ui->sbVoltageDivisor->setValue(15.75);
        ui->sbCurrentMultiplier->setValue(10);
        ui->sbCurrentMax->setValue(170);
        ui->cbInitDelay->setChecked(false);
    } else if(arg1 == "Platinum PRO v4 130A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(11.3);
        ui->sbCurrentMax->setValue(150);
        ui->cbInitDelay->setChecked(false);
    } else if(arg1 == "Platinum PRO v4 200A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(16.9);
        ui->sbCurrentMax->setValue(220);
        ui->cbInitDelay->setChecked(false);
    } else if(arg1 == "FlyFun 30/40A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(0);
        ui->sbCurrentMax->setValue(0);
        ui->cbInitDelay->setChecked(true);
    } else if(arg1 == "FlyFun 60A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(6);
        ui->sbCurrentMax->setValue(80);
        ui->cbInitDelay->setChecked(true);
    } else if(arg1 == "FlyFun 80A") {
        ui->sbVoltageDivisor->setValue(15.75);
        ui->sbCurrentMultiplier->setValue(12.4);
        ui->sbCurrentMax->setValue(100);
        ui->cbInitDelay->setChecked(true);
    } else if(arg1 == "FlyFun 120A") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(15);
        ui->sbCurrentMax->setValue(140);
        ui->cbInitDelay->setChecked(true);
    } else if(arg1 == "FlyFun 110A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(15);
        ui->sbCurrentMax->setValue(130);
        ui->cbInitDelay->setChecked(true);
    } else if(arg1 == "FlyFun 130A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(15);
        ui->sbCurrentMax->setValue(150);
        ui->cbInitDelay->setChecked(true);
    } else if(arg1 == "FlyFun 160A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(15);
        ui->sbCurrentMax->setValue(180);
        ui->cbInitDelay->setChecked(true);
    }
}

void MainWindow::on_cbBarometerType_currentIndexChanged(const QString &arg1)
{
    if(arg1 == "BMP280") {
        ui->cbAltitudeFilter->setVisible(true);
        ui->lbAltitudeFilter->setVisible(true);
    } else {
        ui->cbAltitudeFilter->setVisible(false);
        ui->lbAltitudeFilter->setVisible(false);
    }
}

void MainWindow::on_gbEsc_toggled(bool arg1)
{
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbVoltage1_toggled(bool arg1)
{
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbTemperature1_toggled(bool checked)
{
    Q_UNUSED(checked);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbAltitude_toggled(bool arg1)
{
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbAirspeed_toggled(bool checked)
{
    Q_UNUSED(checked);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbCurrent_toggled(bool arg1)
{
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbBarometerType_currentTextChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbEsc_currentTextChanged(const QString &arg1)
{
    generateCircuit(ui->lbCircuit);
    if(arg1 == "Hobbywing V4")
        ui->cbPwmOut->setVisible(true);
    else
        ui->cbPwmOut->setVisible(false);
}

void MainWindow::on_cbReceiver_currentTextChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_btCircuit_clicked()
{
    CircuitDialog circuitDialog;
    circuitDialog.setModal(true);
    circuitDialog.mainWindow = this;
    generateCircuit(circuitDialog.lbCircuit);
    circuitDialog.exec();
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbGps_toggled(bool arg1)
{
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbCurrentAutoOffset_toggled(bool checked)
{
    if(checked) {
        ui->lbQuiescentVoltage->setVisible(false);
        ui->sbQuiescentVoltage->setVisible(false);
    } else {
        ui->lbQuiescentVoltage->setVisible(true);
        ui->sbQuiescentVoltage->setVisible(true);
    }
}

void MainWindow::on_cbCurrentSensorType_currentTextChanged(
        const QString &arg1)
{
    if(arg1 == "Hall effect") {
        ui->cbCurrentAutoOffset->setVisible(true);
        ui->lbCurrentSens->setVisible(true);
        ui->sbCurrentSens->setVisible(true);
        ui->lbAnalogCurrentMultiplier->setVisible(false);
        ui->sbAnalogCurrentMultiplier->setVisible(false);
        if(ui->cbCurrentAutoOffset->isChecked()) {
            ui->lbQuiescentVoltage->setVisible(false);
            ui->sbQuiescentVoltage->setVisible(false);
        } else {
            ui->lbQuiescentVoltage->setVisible(true);
            ui->sbQuiescentVoltage->setVisible(true);
        }
    }
    if(arg1 == "Shunt resistor") {
        ui->cbCurrentAutoOffset->setVisible(false);
        ui->lbCurrentSens->setVisible(false);
        ui->sbCurrentSens->setVisible(false);
        ui->lbQuiescentVoltage->setVisible(false);
        ui->sbQuiescentVoltage->setVisible(false);
        ui->lbAnalogCurrentMultiplier->setVisible(true);
        ui->sbAnalogCurrentMultiplier->setVisible(true);
    }
}

void MainWindow::on_cbClockStretch_toggled(bool checked)
{
    Q_UNUSED(checked);
    generateCircuit(ui->lbCircuit);
}

