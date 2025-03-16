#include "mainwindow.h"

#include "circuitdialog.h"
#include "qobject.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow), serial(new QSerialPort()) {
    ui->setupUi(this);
    ui->tbViews->setCurrentIndex(0);
    ui->ptDebug->ensureCursorVisible();
    ui->saConfig->setEnabled(false);
    ui->btDebug->setDisabled(true);
    ui->cbEsc->addItems({"Hobbywing V3", "Hobbywing V4/Flyfun (not VBAR firmware)", "PWM", "Castle Link", "Kontronic",
                         "Kiss", "APD HV", "HobbyWing V5"});

    ui->cbGpsBaudrate->addItems({"115200", "57600", "38400", "19200", "14400", "9600", "4800"});
    ui->cbGpsBaudrate->setCurrentIndex(5);
    ui->cbReceiver->addItem("Frsky Smartport", RX_SMARTPORT);
    ui->cbReceiver->addItem("Frsky D", RX_FRSKY_D);
    ui->cbReceiver->addItem("Spektrum XBUS", RX_XBUS);
    ui->cbReceiver->addItem("Spektrum SRXL", RX_SRXL);
    ui->cbReceiver->addItem("Spektrum SRXL2", RX_SRXL2);
    ui->cbReceiver->addItem("Flysky IBUS", RX_IBUS);
    ui->cbReceiver->addItem("Futaba SBUS2", RX_SBUS);
    ui->cbReceiver->addItem("Jeti Ex Bus", RX_JETIEX);
    ui->cbReceiver->addItem("Multiplex Sensor Bus", RX_MULTIPLEX);
    ui->cbReceiver->addItem("CRSF", RX_CRSF);
    ui->cbReceiver->addItem("Sanwa", RX_SANWA);
    ui->cbReceiver->addItem("HOTT", RX_HOTT);
    ui->cbReceiver->addItem("Hitec", RX_HITEC);
    ui->cbReceiver->addItem("JR Propo", RX_JR_PROPO);
    ui->cbReceiver->addItem("Serial Monitor", SERIAL_MONITOR);
    ui->cbEscModel->addItems({"", "Platinum PRO v4 25/40/60", "Platinum PRO v4 80A", "Platinum PRO v4 100A",
                              "Platinum PRO v4 120A", "Platinum PRO v4 130A-HV", "Platinum PRO v4 150A",
                              "Platinum PRO v4 200A-HV", "FlyFun 30/40A", "FlyFun 60A", "FlyFun 80A", "FlyFun 120A",
                              "FlyFun 110A-HV", "FlyFun 130A-HV", "FlyFun 160A-HV"});

    ui->sbEscOffset->setVisible(false);
    ui->cbCurrentSensorType->addItems({"Hall effect", "Shunt resistor"});
    ui->cbCurrentAutoOffset->setChecked(true);
    ui->cbBarometerType->addItems({"BMP280", "MS5611", "BMP180"});
    ui->cbAltitudeFilter->addItems({"Low", "Medium", "High"});
    ui->cbAltitudeFilter->setCurrentIndex(2);
    ui->cbSpeedUnitsGps->addItems({"km/h", "kts"});
    ui->lbQuiescentVoltage->setText("Zero current output voltage, V<sub>IOUT</sub> (V)");
    ui->cbVarioAutoOffset->setVisible(false);
    for (uint8_t i = 0; i < 127; i++) {
        QString hex;
        hex.setNum(i, 16);
        ui->cbAddress->addItem("0x" + hex);
    }
    ui->cbAddress->setCurrentIndex(0x77);

    ui->cbBaudrate->addItems({"115200", "57600", "38400", "19200", "9600", "4800"});
    ui->cbStopbits->addItems({"1", "2"});
    ui->cbParity->addItems({"None", "Odd", "Even"});

    ui->cbMaxPressure->addItems({"< 1 kPa (K = 8192)", "< 2 kPa (K = 4096)", "< 4 kPa (K = 2048)", "< 8 kPa (K = 1024)",
                                 "< 16 kPa (K = 512)", "< 32 kPa (K = 256)", "< 65 kPa (K = 128)", "< 130 kPa (K = 64)",
                                 "< 260 kPa (K = 32)", "< 500 kPa (K = 16)", "< 1000 kPa (K = 8)",
                                 "> 1000 kPa (K = 4)"});

    connect(ui->btConnect, SIGNAL(released()), this, SLOT(buttonSerialPort()));
    connect(ui->btDebug, SIGNAL(released()), this, SLOT(buttonDebug()));
    connect(ui->btClearDebug, SIGNAL(released()), this, SLOT(buttonClearDebug()));
    connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(exitApp()));
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerial);
    connect(ui->btUpdate, SIGNAL(released()), this, SLOT(writeSerialConfig()));
    connect(ui->actionUpdateConfig, SIGNAL(triggered()), this, SLOT(writeSerialConfig()));

    connect(ui->actionOpen, SIGNAL(triggered()), this, SLOT(openConfig()));
    connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(saveConfig()));
    connect(ui->actionAbout, SIGNAL(triggered()), this, SLOT(showAbout()));
    connect(ui->actionDefaultConfig, SIGNAL(triggered()), this, SLOT(defaultConfig()));

    ui->lbCircuit->resize(621, 400);  //(ui->lbCircuit->parentWidget()->width(),
    // ui->lbCircuit->parentWidget()->height());
    generateCircuit(ui->lbCircuit);

    portsList = fillPortsInfo();

    statusBar()->showMessage("Not connected");

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::checkPorts);
    timer->start(1000);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::generateCircuit(QLabel *label) {
    QSize *size = new QSize(label->width(), label->height());
    QPixmap *pix = new QPixmap(*size);
    QPainter *paint = new QPainter(pix);
    QImage image;

    image.load(":/res/rp2040_zero.png");
    paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));

    if (ui->cbReceiver->currentText() == "Serial Monitor") {
        image.load(":/res/esc_rp2040_zero.png");
        paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
    } else {
        if (ui->gbCurrent->isChecked()) {
            image.load(":/res/current_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbVoltage1->isChecked()) {
            image.load(":/res/voltage_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        }

        if (ui->cbTemperature1->isChecked()) {
            image.load(":/res/ntc_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbAirspeed->isChecked()) {
            image.load(":/res/airspeed_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbGps->isChecked()) {
            image.load(":/res/gps_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbEsc->isChecked()) {
            // ui->lbEsc->setEnabled(true);
            // ui->cbEsc->setEnabled(true);
            // ui->lbEscModel->setEnabled(true);
            // ui->lbEscModel->setEnabled(true);
            // ui->gbRpmMultipliers->setEnabled(true);
            if (ui->cbEsc->currentText() == "Hobbywing V3" ||
                ui->cbEsc->currentText() == "Hobbywing V4/Flyfun (not VBAR firmware)" ||
                ui->cbEsc->currentText() == "Kontronic" || ui->cbEsc->currentText() == "KIss" ||
                ui->cbEsc->currentText() == "APD HV" || ui->cbEsc->currentText() == "HobbyWing V5")
                image.load(":/res/esc_rp2040_zero.png");
            else if (ui->cbEsc->currentText() == "PWM")
                image.load(":/res/pwm_rp2040_zero.png");
            else if (ui->cbEsc->currentText() == "Castle Link")
                image.load(":/res/castle_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        } else {
        }

        if (ui->gbAltitude->isChecked()) {
            image.load(":/res/vario_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbFuelPressure->isChecked()) {
            image.load(":/res/vario_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        }

        if (ui->cbReceiver->currentText() == "Frsky D") {
            image.load(":/res/receiver_frsky_d_rp2040_zero.png");
        } else if (ui->cbReceiver->currentText() == "Spektrum XBUS") {
            image.load(":/res/receiver_xbus_rp2040_zero.png");
        } else if (ui->cbReceiver->currentText() == "Hitec") {
            image.load(":/res/receiver_hitec_rp2040_zero.png");
        } else {
            image.load(":/res/receiver_serial_rp2040_zero.png");
        }
        if (ui->cbReceiver->currentText() == "Spektrum XBUS" && ui->cbClockStretch->isChecked() == true) {
            image.load(":/res/clock_stretch_xbus_rp2040_zero.png");
            paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));
        }
    }
    paint->drawImage(QPoint(0, 0), image.scaled(*size, Qt::IgnoreAspectRatio));

    label->setPixmap(*pix);
}

void MainWindow::openConfig() {
    QString filename = QFileDialog::getOpenFileName(this, "Open Config", "", "Config Files (*.cfg)");
    QFile file(filename);

    if (file.open(QIODevice::ReadOnly)) {
        file.read((char *)&config, sizeof(config_t));
        file.close();
        if (config.version > CONFIG_VERSION) {
            QMessageBox::warning(this, tr("Information"),
                                 tr("Firmware config version is ") + QString::number(config.version) +
                                     ". mscr_gui config version is " + QString::number(CONFIG_VERSION) +
                                     ". Download latest msrc_gui. Please save the config in case the conversion fails.",
                                 QMessageBox::Close);
            saveConfig();
            closeSerialPort();
            return;
        }
        if (config.version < CONFIG_VERSION) {
            QMessageBox::warning(this, tr("Information"),
                                 tr("Firmware config version is ") + QString::number(config.version) +
                                     ". mscr_gui config version is " + QString::number(CONFIG_VERSION) +
                                     ". Converting config version to " + QString::number(CONFIG_VERSION) +
                                     "\nPlease press Update button to update MSRC config with new config "
                                     "version.\nAlso is needed to update MSRC firmware, if not already done.",
                                 QMessageBox::Close);
            config.version = CONFIG_VERSION;
        }
        setUiFromConfig();
    }
}

void MainWindow::saveConfig() {
    QFileDialog dialog(this, "Save Config", QString(), "Config Files (*.cfg)");
    dialog.setDefaultSuffix(".cfg");
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    if (dialog.exec()) {
        QString filename = dialog.selectedFiles().front();
        QFile file(filename);
        if (file.open(QIODevice::WriteOnly)) {
            getConfigFromUi();
            file.write((char *)&config, sizeof(config_t));
        }
    }
}

void MainWindow::showAbout() {
    QMessageBox::information(
        this, tr("About"),
        QString::asprintf("MSRC V%u.%u.%u\n\rDaniel Gorbea © 2020/2024", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH),
        QMessageBox::Close);
}

void MainWindow::buttonSerialPort() {
    if (isConnected)
        closeSerialPort();
    else
        openSerialPort();
}

void MainWindow::buttonDebug() {
    if (ui->btDebug->text() == "Enable Debug") {
        if (!isConnected) return;
        ui->btDebug->setText("Disable Debug");
        serial->readAll();
        char header = 0x30;
        serial->write(&header, 1);
        char command = 0x33;
        serial->write(&command, 1);
        isDebug = true;
    } else if (ui->btDebug->text() == "Disable Debug") {
        if (!isConnected) return;
        ui->btDebug->setText("Enable Debug");
        char header = 0x30;
        serial->write(&header, 1);
        char command = 0x34;
        serial->write(&command, 1);
        isDebug = false;
    }
}

void MainWindow::buttonClearDebug() { ui->ptDebug->clear(); }

void MainWindow::openSerialPort() {
    QString portName = ui->cbPortList->currentData().toString();
    serial->setPortName(portName);
    serial->setBaudRate(QSerialPort::BaudRate::Baud115200);
    serial->setDataBits(QSerialPort::DataBits::Data8);
    serial->setParity(QSerialPort::Parity::NoParity);
    serial->setStopBits(QSerialPort::StopBits::OneStop);
    serial->setFlowControl(QSerialPort::FlowControl::NoFlowControl);
    if (serial->open(QIODevice::ReadWrite)) {
        statusBar()->showMessage("Connected " + ui->cbPortList->currentText());
        isConnected = true;
        requestSerialConfig();
        serial->setDataTerminalReady(true);
        ui->btUpdate->setEnabled(true);
        ui->btConnect->setText("Disconnect");
        ui->actionUpdateConfig->setEnabled(true);
        ui->actionDefaultConfig->setEnabled(true);
        ui->saConfig->setEnabled(true);
        ui->cbPortList->setDisabled(true);
        ui->btDebug->setEnabled(true);
        ui->btDebug->setText("Enable Debug");
    } else {
        statusBar()->showMessage("Not connected: " + serial->errorString());
        isConnected = false;
        ui->btUpdate->setEnabled(false);
        ui->btConnect->setText("Connect");
        ui->actionUpdateConfig->setEnabled(false);
        ui->actionDefaultConfig->setEnabled(false);
        ui->saConfig->setEnabled(false);
        ui->btDebug->setEnabled(false);
        ui->btDebug->setText("Enable Debug");
    }
}

void MainWindow::closeSerialPort() {
    serial->close();
    statusBar()->showMessage("Not connected");
    isConnected = false;
    ui->btUpdate->setEnabled(false);
    ui->btConnect->setText("Connect");
    ui->actionUpdateConfig->setEnabled(false);
    ui->actionDefaultConfig->setEnabled(false);
    ui->saConfig->setEnabled(false);
    ui->cbPortList->setDisabled(false);
    ui->btDebug->setDisabled(true);
    ui->btDebug->setText("Enable Debug");
}

void MainWindow::readSerial() {
    /*
       header - 0x30
       command - 0x30 - msrc to read config from usb
                 0x31 - msrc to request to send config to usb
                 0x32 - msrc answer to send config
                 0x33 - debug on
                 0x34 - debug off
                 0x35 - save default config to rp2040 flash
    */

    if (isDebug == false) {
        if (serial->bytesAvailable() == sizeof(config_t) + 2) {
            data = serial->readAll();
            if (data.at(0) == 0x30 && data.at(1) == 0x32) {
                memcpy(&config, data.data() + 2, sizeof(config_t));

                if (config.version > CONFIG_VERSION) {
                    QMessageBox::warning(
                        this, tr("Information"),
                        tr("Firmware config version is ") + QString::number(config.version) +
                            ". mscr_gui config version is " + QString::number(CONFIG_VERSION) +
                            ". Download latest msrc_gui. Please save the config in case the conversion fails.",
                        QMessageBox::Close);
                    saveConfig();
                    closeSerialPort();
                    return;
                }
                if (config.version < CONFIG_VERSION) {
                    QMessageBox::warning(
                        this, tr("Information"),
                        tr("Firmware config version is ") + QString::number(config.version) +
                            ". mscr_gui config version is " + QString::number(CONFIG_VERSION) +
                            ". Converting config version to " + QString::number(CONFIG_VERSION) +
                            "\nIt is needed to update MSRC firmware first or you will lose your config to the default "
                            "values. Press Update button to update MSRC config with new config version only if MSRC "
                            "firmware is updated to latest version first.",
                        QMessageBox::Close);
                    config.version = CONFIG_VERSION;
                }

                setUiFromConfig();
            }
        }
    } else {
        data = serial->readAll();
        ui->ptDebug->insertPlainText(data);
        if (autoscroll) ui->ptDebug->ensureCursorVisible();
    }
}

void MainWindow::writeSerialConfig() {
    if (!isConnected) return;
    getConfigFromUi();
    char header = 0x30;
    serial->write(&header, 1);
    char command = 0x30;
    serial->write(&command, 1);
    serial->write((char *)&config, sizeof(config_t));
    /*QMessageBox msgBox;
    msgBox.setText("Reset RP2040 to apply settings.");
    msgBox.exec();*/
    QMessageBox::warning(this, tr("Information"), tr("Reset RP2040 to apply settings."), QMessageBox::Close);
}

void MainWindow::defaultConfig() {
    if (!isConnected) return;
    char header = 0x30;
    serial->write(&header, 1);
    char command = 0x35;
    serial->write(&command, 1);
    QMessageBox::warning(this, tr("Information"), tr("Reset RP2040 to apply settings."), QMessageBox::Close);
}

void MainWindow::setUiFromConfig() {
    /* Receiver protocol */

    int index = ui->cbReceiver->findData(config.rx_protocol);
    ui->cbReceiver->setCurrentIndex(index);

    /* Serial Monitor */

    int item = ui->cbBaudrate->findText(QString::number(config.serial_monitor_baudrate));
    if (item == -1)
        ui->cbBaudrate->setCurrentText(QString::number(config.serial_monitor_baudrate));
    else
        ui->cbBaudrate->setCurrentIndex(item);
    if (config.serial_monitor_parity > 2) config.serial_monitor_parity = 0;
    ui->cbParity->setCurrentIndex(config.serial_monitor_parity);
    if (config.serial_monitor_stop_bits > 2 || config.serial_monitor_stop_bits < 1) config.serial_monitor_stop_bits = 1;
    ui->cbStopbits->setCurrentIndex(config.serial_monitor_stop_bits - 1);
    if (config.serial_monitor_timeout_ms > 100) config.serial_monitor_timeout_ms = 100;
    ui->sbTimeout->setValue(config.serial_monitor_timeout_ms);
    ui->cbInverted->setChecked(config.serial_monitor_inverted);

    /* Sensors */

    // ESC

    if (config.esc_protocol == esc_protocol_t::ESC_NONE)
        ui->gbEsc->setChecked(false);
    else {
        ui->gbEsc->setChecked(true);
        ui->cbEsc->setCurrentIndex(config.esc_protocol - 1);
    }

    // GPS

    ui->gbGps->setChecked(config.enable_gps);
    switch (config.gps_baudrate) {
        case 115200:
            ui->cbGpsBaudrate->setCurrentIndex(0);
            break;
        case 57600:
            ui->cbGpsBaudrate->setCurrentIndex(1);
            break;
        case 38400:
            ui->cbGpsBaudrate->setCurrentIndex(2);
            break;
        case 19200:
            ui->cbGpsBaudrate->setCurrentIndex(3);
            break;
        case 14400:
            ui->cbGpsBaudrate->setCurrentIndex(4);
            break;
        case 9600:
            ui->cbGpsBaudrate->setCurrentIndex(5);
            break;
        case 4800:
            ui->cbGpsBaudrate->setCurrentIndex(6);
            break;
    }

    // Analog rate

    ui->sbAnalogRate->setValue(config.analog_rate);

    // Voltage

    ui->gbVoltage1->setChecked(config.enable_analog_voltage);

    // Temperature

    ui->cbTemperature1->setChecked(config.enable_analog_ntc);

    // Current

    ui->gbCurrent->setChecked(config.enable_analog_current);
    ui->cbCurrentSensorType->setCurrentIndex(config.analog_current_type);
    ui->cbCurrentAutoOffset->setChecked(config.analog_current_autoffset);
    ui->sbQuiescentVoltage->setValue(config.analog_current_quiescent_voltage);
    if (config.analog_current_type == analog_current_type_t::CURRENT_TYPE_HALL)
        ui->sbCurrentSens->setValue(1000 / config.analog_current_multiplier);
    else
        ui->sbAnalogCurrentMultiplier->setValue(config.analog_current_multiplier);

    // Airspeed

    ui->gbAirspeed->setChecked(config.enable_analog_airspeed);
    ui->sbAirspeedSlope->setValue(config.airspeed_slope / 100.0);
    ui->sbAirspeedOffset->setValue(config.airspeed_offset / 100.0);

    // Altitude

    if (config.i2c_module == i2c_module_t::I2C_NONE)
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
    // ui->sbDefRate->setValue(config.refresh_rate_def);

    // Averaging

    ui->sbRpmAvg->setValue(qRound(2 / config.alpha_rpm - 1));
    ui->sbVoltageAvg->setValue(qRound(2 / config.alpha_voltage - 1));
    ui->sbCurrentAvg->setValue(qRound(2 / config.alpha_current - 1));
    ui->sbTemperatureAvg->setValue(qRound(2 / config.alpha_temperature - 1));
    ui->sbVarioAvg->setValue(qRound(2 / config.alpha_vario - 1));
    ui->sbAirspeedAvg->setValue(qRound(2 / config.alpha_airspeed - 1));

    // Analog voltage multipliers

    ui->sbVoltage1Mult->setValue(config.analog_voltage_multiplier);
    // config.analog_voltage2_multiplier = ui->sbVoltage2Mult->value();

    // RPM Multipliers

    ui->sbPairOfPoles->setValue(config.pairOfPoles);
    ui->sbMainTeeth->setValue(config.mainTeeth);
    ui->sbPinionTeeth->setValue(config.pinionTeeth);

    // PWM out

    ui->cbPwmOut->setChecked(config.enable_pwm_out);

    // Smartport

    // config.smartport_data_id;
    ui->sbSensorId->setValue(config.smartport_sensor_id);

    // XBUS Clock stretch

    ui->cbClockStretch->setChecked(config.xbus_clock_stretch);
    ui->cbAlternativePacket->setChecked(config.xbus_use_alternative_volt_temp);

    // Ibus

    ui->cbAlternativeCoordinates->setChecked(config.ibus_alternative_coordinates);

    // Jeti Ex

    if (config.jeti_gps_speed_units_kmh == true)
        ui->cbSpeedUnitsGps->setCurrentIndex(0);
    else
        ui->cbSpeedUnitsGps->setCurrentIndex(1);

    // HW V4/V5 parameters

    ui->cbInitDelay->setChecked(config.enable_esc_hw4_init_delay);
    ui->cbEscAutoOffset->setChecked(!config.esc_hw4_is_manual_offset);
    ui->sbEscOffset->setValue(config.esc_hw4_offset);
    // config.esc_hw4_init_delay_duration = 10000;
    ui->sbCurrentThresold->setValue(config.esc_hw4_current_thresold);
    ui->sbVoltageDivisor->setValue(config.esc_hw4_divisor);
    ui->sbCurrentMultiplier->setValue(config.esc_hw4_current_multiplier);
    ui->sbCurrentMax->setValue(config.esc_hw4_current_max);

    // Fuel flow

    ui->gbFuelmeter->setChecked(config.enable_fuel_flow);
    ui->sbMlPulse->setValue(config.fuel_flow_ml_per_pulse);

    // Fuel pressure

    ui->gbFuelPressure->setChecked(config.enable_fuel_pressure);
    if (config.xgzp68xxd_k == 8192)
        ui->cbMaxPressure->setCurrentIndex(0);
    else if (config.xgzp68xxd_k == 4096)
        ui->cbMaxPressure->setCurrentIndex(1);
    else if (config.xgzp68xxd_k == 2048)
        ui->cbMaxPressure->setCurrentIndex(2);
    else if (config.xgzp68xxd_k == 1024)
        ui->cbMaxPressure->setCurrentIndex(3);
    else if (config.xgzp68xxd_k == 512)
        ui->cbMaxPressure->setCurrentIndex(4);
    else if (config.xgzp68xxd_k == 256)
        ui->cbMaxPressure->setCurrentIndex(5);
    else if (config.xgzp68xxd_k == 128)
        ui->cbMaxPressure->setCurrentIndex(6);
    else if (config.xgzp68xxd_k == 64)
        ui->cbMaxPressure->setCurrentIndex(7);
    else if (config.xgzp68xxd_k == 32)
        ui->cbMaxPressure->setCurrentIndex(8);
    else if (config.xgzp68xxd_k == 16)
        ui->cbMaxPressure->setCurrentIndex(9);
    else if (config.xgzp68xxd_k == 8)
        ui->cbMaxPressure->setCurrentIndex(10);
    else
        ui->cbMaxPressure->setCurrentIndex(11);
}

void MainWindow::getConfigFromUi() {
    /* Config version  */

    config.version = CONFIG_VERSION;

    /* Receiver protocol */

    config.rx_protocol = (rx_protocol_t)ui->cbReceiver->currentData().toInt();

    /* Serial Monitor */

    config.serial_monitor_baudrate = ui->cbBaudrate->currentText().toInt();
    config.serial_monitor_stop_bits = ui->cbStopbits->currentText().toInt();
    if (ui->cbParity->currentText() == "None")
        config.serial_monitor_parity = 0;
    else if (ui->cbParity->currentText() == "Odd")
        config.serial_monitor_parity = 1;
    else
        config.serial_monitor_parity = 2;
    config.serial_monitor_timeout_ms = ui->sbTimeout->value();
    config.serial_monitor_inverted = ui->cbInverted->isChecked();

    /* Sensors */

    // ESC

    if (ui->gbEsc->isChecked())
        config.esc_protocol = (esc_protocol_t)(ui->cbEsc->currentIndex() + 1);
    else
        config.esc_protocol = esc_protocol_t::ESC_NONE;

    // GPS

    config.enable_gps = ui->gbGps->isChecked();
    config.gps_baudrate = ui->cbGpsBaudrate->currentText().toInt();

    // Voltage

    config.enable_analog_voltage = ui->gbVoltage1->isChecked();
    config.analog_voltage_multiplier = ui->sbVoltage1Mult->value();

    // Current

    config.enable_analog_current = ui->gbCurrent->isChecked();
    config.analog_current_type = (analog_current_type_t)ui->cbCurrentSensorType->currentIndex();
    config.analog_current_quiescent_voltage = ui->sbQuiescentVoltage->value();
    if (ui->cbCurrentSensorType->currentIndex() == analog_current_type_t::CURRENT_TYPE_HALL) {
        if (ui->cbCurrentAutoOffset->isChecked()) {
            config.analog_current_autoffset = true;
            config.analog_current_offset = 0;
        } else {
            config.analog_current_autoffset = false;
            config.analog_current_offset = ui->sbQuiescentVoltage->value();
        }
        config.analog_current_multiplier = 1000 / ui->sbCurrentSens->value();
    } else if (ui->cbCurrentSensorType->currentIndex() == analog_current_type_t::CURRENT_TYPE_SHUNT) {
        config.analog_current_autoffset = false;
        config.analog_current_offset = 0;
        config.analog_current_multiplier = ui->sbAnalogCurrentMultiplier->value();
    }

    // Temperature

    config.enable_analog_ntc = ui->cbTemperature1->isChecked();

    // Airspeed

    config.enable_analog_airspeed = ui->gbAirspeed->isChecked();
    config.airspeed_slope = ui->sbAirspeedSlope->value() * 100;
    config.airspeed_offset = ui->sbAirspeedOffset->value() * 100;

    // Altitude

    if (ui->gbAltitude->isChecked())
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

    // Analog rate

    config.analog_rate = ui->sbAnalogRate->value();

    // RPM Multipliers

    config.pairOfPoles = ui->sbPairOfPoles->value();
    config.mainTeeth = ui->sbMainTeeth->value();
    config.pinionTeeth = ui->sbPinionTeeth->value();
    config.rpm_multiplier = config.pinionTeeth / (1.0 * config.mainTeeth * config.pairOfPoles);

    // PWM out

    config.enable_pwm_out = ui->cbPwmOut->isChecked();

    // Smartport

    // config.smartport_data_id = 0x5000;
    config.smartport_sensor_id = ui->sbSensorId->value();

    // XBUS Clock stretch

    config.xbus_clock_stretch = ui->cbClockStretch->isChecked();
    config.xbus_use_alternative_volt_temp = ui->cbAlternativePacket->isChecked();

    // Jeti Ex

    if (ui->cbSpeedUnitsGps->currentText() == "km/h")
        config.jeti_gps_speed_units_kmh = true;
    else
        config.jeti_gps_speed_units_kmh = false;

    // Ibus

    config.ibus_alternative_coordinates = ui->cbAlternativeCoordinates->isChecked();

    // HW V4/V5 parameters

    config.enable_esc_hw4_init_delay = ui->cbInitDelay->isChecked();
    config.esc_hw4_is_manual_offset = !ui->cbEscAutoOffset->isChecked();
    config.esc_hw4_offset = ui->sbEscOffset->value();
    // config.esc_hw4_init_delay_duration = 10000;
    config.esc_hw4_current_thresold = ui->sbCurrentThresold->value();
    config.esc_hw4_divisor = ui->sbVoltageDivisor->value();
    config.esc_hw4_current_multiplier = ui->sbCurrentMultiplier->value();
    config.esc_hw4_current_max = ui->sbCurrentMax->value();

    // Fuel flow

    config.enable_fuel_flow = ui->gbFuelmeter->isChecked();
    config.fuel_flow_ml_per_pulse = ui->sbMlPulse->value();

    // Fuel pressure

    config.enable_fuel_pressure = ui->gbFuelPressure->isChecked();
    if (ui->cbMaxPressure->currentIndex() == 0)
        config.xgzp68xxd_k = 8192;
    else if (ui->cbMaxPressure->currentIndex() == 1)
        config.xgzp68xxd_k = 4096;
    else if (ui->cbMaxPressure->currentIndex() == 2)
        config.xgzp68xxd_k = 2048;
    else if (ui->cbMaxPressure->currentIndex() == 3)
        config.xgzp68xxd_k = 1024;
    else if (ui->cbMaxPressure->currentIndex() == 4)
        config.xgzp68xxd_k = 512;
    else if (ui->cbMaxPressure->currentIndex() == 5)
        config.xgzp68xxd_k = 256;
    else if (ui->cbMaxPressure->currentIndex() == 6)
        config.xgzp68xxd_k = 128;
    else if (ui->cbMaxPressure->currentIndex() == 7)
        config.xgzp68xxd_k = 64;
    else if (ui->cbMaxPressure->currentIndex() == 8)
        config.xgzp68xxd_k = 32;
    else if (ui->cbMaxPressure->currentIndex() == 9)
        config.xgzp68xxd_k = 16;
    else if (ui->cbMaxPressure->currentIndex() == 10)
        config.xgzp68xxd_k = 8;
    else
        config.xgzp68xxd_k = 4;

    // Debug

    config.debug = 0;  // disabled from msrc_gui
}

void MainWindow::requestSerialConfig() {
    // disable debug
    char header = 0x30;
    serial->write(&header, 1);
    char command = 0x34;
    serial->write(&command, 1);
    isDebug = false;
    QTimer::singleShot(2000, this, SLOT(readSerialConfig()));
}

void MainWindow::readSerialConfig() {
    serial->readAll();
    // request config
    char header = 0x30;
    serial->write(&header, 1);
    char command = 0x31;
    serial->write(&command, 1);
}

QStringList MainWindow::fillPortsInfo() {
    const QList<QSerialPortInfo> infos = QSerialPortInfo::availablePorts();
    QStringList list;
    ui->cbPortList->clear();
    for (const QSerialPortInfo &info : infos) {
        list.append(info.portName() + " (" + info.manufacturer() + " " + info.description() + ")");
        ui->cbPortList->addItem(info.portName() + " (" + info.manufacturer() + " " + info.description() + ")",
                                info.portName());
    }
    return list;
}

void MainWindow::checkPorts() {
    const QList<QSerialPortInfo> infos = QSerialPortInfo::availablePorts();
    QStringList list;
    for (const QSerialPortInfo &info : infos) {
        list.append(info.portName() + " (" + info.manufacturer() + " " + info.description() + ")");
    }

    std::sort(portsList.begin(), portsList.end());
    std::sort(list.begin(), list.end());

    if (portsList != list) {
        QString currentPort = ui->cbPortList->currentText();
        if (isConnected && !list.contains(currentPort)) closeSerialPort();
        portsList = fillPortsInfo();
        ui->cbPortList->setCurrentIndex(ui->cbPortList->findText(currentPort));
        if (ui->cbPortList->currentIndex() == -1 && ui->cbPortList->count()) ui->cbPortList->setCurrentIndex(0);
    }
}

void MainWindow::exitApp() { QApplication::quit(); }

void MainWindow::enableWidgets(QWidget *widget, bool enable) {
    QList<QWidget *> widgets = widget->findChildren<QWidget *>();
    QWidget *child;
    foreach (child, widgets) child->setEnabled(enable);
}

void MainWindow::on_cbReceiver_currentIndexChanged(const QString &arg1) {
    if (arg1 == "Spektrum XBUS") {
        ui->cbClockStretch->setVisible(true);
        ui->cbAlternativePacket->setVisible(true);

    } else {
        ui->cbClockStretch->setVisible(false);
        ui->cbAlternativePacket->setVisible(false);
    }

    if (arg1 == "Frsky Smartport" || arg1 == "Frsky D") {
        ui->gbRate->setVisible(true);
    } else {
        ui->gbRate->setVisible(false);
    }

    if (arg1 == "Frsky Smartport") {
        ui->lbSensorId->setVisible(true);
        ui->sbSensorId->setVisible(true);
    } else {
        ui->lbSensorId->setVisible(false);
        ui->sbSensorId->setVisible(false);
    }

    if (arg1 == "Flysky IBUS") {
        ui->cbAlternativeCoordinates->setVisible(true);
    } else {
        ui->cbAlternativeCoordinates->setVisible(false);
    }

    if (arg1 == "Jeti Ex Bus") {
        ui->cbSpeedUnitsGps->setVisible(true);
        ui->lbSpeedUnitsGps->setVisible(true);
    } else {
        ui->cbSpeedUnitsGps->setVisible(false);
        ui->lbSpeedUnitsGps->setVisible(false);
    }
    if (arg1 == "Serial Monitor") {
        ui->cbBaudrate->setVisible(true);
        ui->cbStopbits->setVisible(true);
        ui->cbParity->setVisible(true);
        ui->sbTimeout->setVisible(true);
        ui->cbInverted->setVisible(true);
        ui->lbBaudrate->setVisible(true);
        ui->lbStopbits->setVisible(true);
        ui->lbParity->setVisible(true);
        ui->lbTimeout->setVisible(true);
    } else {
        ui->cbBaudrate->setVisible(false);
        ui->cbStopbits->setVisible(false);
        ui->cbParity->setVisible(false);
        ui->sbTimeout->setVisible(false);
        ui->cbInverted->setVisible(false);
        ui->lbBaudrate->setVisible(false);
        ui->lbStopbits->setVisible(false);
        ui->lbParity->setVisible(false);
        ui->lbTimeout->setVisible(false);
    }
}

void MainWindow::on_cbEsc_currentIndexChanged(const QString &arg1) {
    if (arg1 == "Hobbywing V4/Flyfun (not VBAR firmware)")
        ui->gbEscParameters->setVisible(true);
    else
        ui->gbEscParameters->setVisible(false);
}

void MainWindow::on_cbEscModel_currentIndexChanged(const QString &arg1) {
    if (arg1 == "Platinum PRO v4 25/40/60") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(0);
        ui->sbCurrentMax->setValue(0);
        ui->cbInitDelay->setChecked(false);
    } else if (arg1 == "Platinum PRO v4 80A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(4000 / 8);
        ui->sbCurrentMax->setValue(100);
        ui->cbInitDelay->setChecked(false);
    } else if (arg1 == "Platinum PRO v4 100A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(4000 / 9);
        ui->sbCurrentMax->setValue(120);
        ui->cbInitDelay->setChecked(false);
    } else if (arg1 == "Platinum PRO v4 120A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(4000 / 10);
        ui->sbCurrentMax->setValue(140);
        ui->cbInitDelay->setChecked(false);
    } else if (arg1 == "Platinum PRO v4 150A") {
        ui->sbVoltageDivisor->setValue(15.75);
        ui->sbCurrentMultiplier->setValue(4000 / 10);
        ui->sbCurrentMax->setValue(170);
        ui->cbInitDelay->setChecked(false);
    } else if (arg1 == "Platinum PRO v4 130A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(4000 / 11.3);
        ui->sbCurrentMax->setValue(150);
        ui->cbInitDelay->setChecked(false);
    } else if (arg1 == "Platinum PRO v4 200A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(4000 / 16.9);
        ui->sbCurrentMax->setValue(220);
        ui->cbInitDelay->setChecked(false);
    } else if (arg1 == "FlyFun 30/40A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(0);
        ui->sbCurrentMax->setValue(0);
        ui->cbInitDelay->setChecked(true);
    } else if (arg1 == "FlyFun 60A") {
        ui->sbVoltageDivisor->setValue(11);
        ui->sbCurrentMultiplier->setValue(4000 / 6);
        ui->sbCurrentMax->setValue(80);
        ui->cbInitDelay->setChecked(true);
    } else if (arg1 == "FlyFun 80A") {
        ui->sbVoltageDivisor->setValue(15.75);
        ui->sbCurrentMultiplier->setValue(4000 / 12.4);
        ui->sbCurrentMax->setValue(100);
        ui->cbInitDelay->setChecked(true);
    } else if (arg1 == "FlyFun 120A") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(4000 / 15);
        ui->sbCurrentMax->setValue(140);
        ui->cbInitDelay->setChecked(true);
    } else if (arg1 == "FlyFun 110A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(4000 / 15);
        ui->sbCurrentMax->setValue(130);
        ui->cbInitDelay->setChecked(true);
    } else if (arg1 == "FlyFun 130A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(4000 / 15);
        ui->sbCurrentMax->setValue(150);
        ui->cbInitDelay->setChecked(true);
    } else if (arg1 == "FlyFun 160A-HV") {
        ui->sbVoltageDivisor->setValue(21);
        ui->sbCurrentMultiplier->setValue(4000 / 15);
        ui->sbCurrentMax->setValue(180);
        ui->cbInitDelay->setChecked(true);
    }
}

void MainWindow::on_cbBarometerType_currentIndexChanged(const QString &arg1) {
    if (arg1 == "BMP280") {
        ui->cbAltitudeFilter->setVisible(true);
        ui->lbAltitudeFilter->setVisible(true);
    } else {
        ui->cbAltitudeFilter->setVisible(false);
        ui->lbAltitudeFilter->setVisible(false);
    }
}

void MainWindow::on_gbEsc_toggled(bool enabled) {
    enableWidgets(ui->gbEsc, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbVoltage1_toggled(bool enabled) {
    enableWidgets(ui->gbVoltage1, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbTemperature1_toggled(bool checked) {
    Q_UNUSED(checked);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbAltitude_toggled(bool enabled) {
    enableWidgets(ui->gbAltitude, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbCurrent_toggled(bool enabled) {
    enableWidgets(ui->gbCurrent, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbBarometerType_currentTextChanged(const QString &arg1) {
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbEsc_currentTextChanged(const QString &arg1) {
    generateCircuit(ui->lbCircuit);
    if (arg1 == "Hobbywing V4")
        ui->cbPwmOut->setVisible(true);
    else
        ui->cbPwmOut->setVisible(false);
}

void MainWindow::on_cbReceiver_currentTextChanged(const QString &arg1) {
    Q_UNUSED(arg1);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_btCircuit_clicked() {
    CircuitDialog circuitDialog;
    circuitDialog.setModal(true);
    circuitDialog.mainWindow = this;
    circuitDialog.exec();
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    Q_UNUSED(event);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbGps_toggled(bool enabled) {
    enableWidgets(ui->gbGps, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbAirspeed_toggled(bool enabled) {
    enableWidgets(ui->gbAirspeed, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbFuelmeter_toggled(bool enabled) {
    enableWidgets(ui->gbFuelmeter, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbCurrentAutoOffset_toggled(bool checked) {
    if (checked) {
        ui->lbQuiescentVoltage->setVisible(false);
        ui->sbQuiescentVoltage->setVisible(false);
    } else if (ui->cbCurrentSensorType->currentIndex() == analog_current_type_t::CURRENT_TYPE_HALL) {
        ui->lbQuiescentVoltage->setVisible(true);
        ui->sbQuiescentVoltage->setVisible(true);
    }
}

void MainWindow::on_cbCurrentSensorType_currentTextChanged(const QString &arg1) {
    if (arg1 == "Hall effect") {
        ui->cbCurrentAutoOffset->setVisible(true);
        ui->lbCurrentSens->setVisible(true);
        ui->sbCurrentSens->setVisible(true);
        ui->lbAnalogCurrentMultiplier->setVisible(false);
        ui->sbAnalogCurrentMultiplier->setVisible(false);
        if (ui->cbCurrentAutoOffset->isChecked()) {
            ui->lbQuiescentVoltage->setVisible(false);
            ui->sbQuiescentVoltage->setVisible(false);
        } else {
            ui->lbQuiescentVoltage->setVisible(true);
            ui->sbQuiescentVoltage->setVisible(true);
        }
    }
    if (arg1 == "Shunt resistor") {
        ui->cbCurrentAutoOffset->setVisible(false);
        ui->lbCurrentSens->setVisible(false);
        ui->sbCurrentSens->setVisible(false);
        ui->lbQuiescentVoltage->setVisible(false);
        ui->sbQuiescentVoltage->setVisible(false);
        ui->lbAnalogCurrentMultiplier->setVisible(true);
        ui->sbAnalogCurrentMultiplier->setVisible(true);
    }
}

void MainWindow::on_cbClockStretch_toggled(bool checked) {
    Q_UNUSED(checked);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbEscAutoOffset_stateChanged(int arg1) {
    if (arg1)
        ui->sbEscOffset->setVisible(false);
    else
        ui->sbEscOffset->setVisible(true);
}

void MainWindow::on_btScroll_clicked() {
    autoscroll = !autoscroll;
    if (autoscroll)
        ui->btScroll->setText("No scroll");
    else
        ui->btScroll->setText("Autoscroll");
}
