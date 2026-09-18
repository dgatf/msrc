#include "mainwindow.h"

#include "circuitdialog.h"
#include "qobject.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow), serial(new QSerialPort()) {
    ui->setupUi(this);

    this->setWindowTitle(QString::asprintf("MSRC Link %s", PROJECT_VERSION));
    ui->tbViews->setCurrentIndex(0);
    ui->ptDebug->ensureCursorVisible();
    ui->tbConfig->setEnabled(true);
    ui->tbConfig->setCurrentIndex(0);
    ui->btDebug->setDisabled(true);
    ui->btUpdate->setDisabled(true);
    enableWidgets(ui->scrollAreaWidgetContentsReceiver, false);
    enableWidgets(ui->scrollAreaWidgetContentsSensors, false);

    ui->cbEsc->addItem("Hobbywing V3", ESC_HW3);
    ui->cbEsc->addItem("Hobbywing V4/Flyfun (not VBAR firmware)", ESC_HW4);
    ui->cbEsc->addItem("PWM", ESC_PWM);
    ui->cbEsc->addItem("Castle Link", ESC_CASTLE);
    ui->cbEsc->addItem("Kontronik", ESC_KONTRONIK);
    ui->cbEsc->addItem("Kiss", ESC_APD_F);
    ui->cbEsc->addItem("APD HV", ESC_APD_HV);
    ui->cbEsc->addItem("HobbyWing V5", ESC_HW5);
    ui->cbEsc->addItem("Smart ESC/BAT", ESC_SMART);
    ui->cbEsc->addItem("OMP M4", ESC_OMP_M4);
    ui->cbEsc->addItem("ZTW", ESC_ZTW);
    ui->cbEsc->addItem("OpenYGE", ESC_OPENYGE);

    ui->cbGpsBaudrate->addItems({"115200", "57600", "38400", "9600"});

    ui->cbReceiver->addItem("Frsky Smartport", RX_SMARTPORT);
    ui->cbReceiver->addItem("Frsky D", RX_FRSKY_D);
    ui->cbReceiver->addItem("Frsky FPort", RX_FPORT);
    ui->cbReceiver->addItem("Frsky FBUS", RX_FBUS);
    ui->cbReceiver->addItem("Spektrum XBUS", RX_XBUS);
    ui->cbReceiver->addItem("Spektrum SRXL", RX_SRXL);
    ui->cbReceiver->addItem("Spektrum SRXL2", RX_SRXL2);
    ui->cbReceiver->addItem("Flysky IBUS", RX_IBUS);
    ui->cbReceiver->addItem("Futaba SBUS2", RX_SBUS);
    ui->cbReceiver->addItem("Jeti Ex Bus", RX_JETIEX);
    ui->cbReceiver->addItem("Jeti Ex Sensor", RX_JETIEX_SENSOR);
    ui->cbReceiver->addItem("Multiplex Sensor Bus", RX_MULTIPLEX);
    ui->cbReceiver->addItem("ELRS/CRSF", RX_CRSF);
    ui->cbReceiver->addItem("Sanwa", RX_SANWA);
    ui->cbReceiver->addItem("HOTT", RX_HOTT);
    ui->cbReceiver->addItem("Hitec", RX_HITEC);
    ui->cbReceiver->addItem("JR Propo", RX_JR_PROPO);
    ui->cbReceiver->addItem("GHST", RX_GHST);
    ui->cbReceiver->addItem("Serial Monitor", SERIAL_MONITOR);

    ui->cbBarometerType->addItem("BMP280", I2C_BMP280);
    ui->cbBarometerType->addItem("MS5611", I2C_MS5611);
    ui->cbBarometerType->addItem("BMP180", I2C_BMP180);

    ui->cbCurrentSensorType->addItem("Hall effect", CURRENT_TYPE_HALL);
    ui->cbCurrentSensorType->addItem("Shunt resistor", CURRENT_TYPE_SHUNT);

    ui->cbGpsProtocol->addItem("UBLOX", UBLOX);
    ui->cbGpsProtocol->addItem("NMEA", NMEA);

    ui->cbParity->addItem("None", 0);
    ui->cbParity->addItem("Odd", 1);
    ui->cbParity->addItem("Even", 2);

    ui->cbStopbits->addItem("1", 1);
    ui->cbStopbits->addItem("2", 2);

    ui->cbSerialFormat->addItem("Hex", FORMAT_HEX);
    ui->cbSerialFormat->addItem("String", FORMAT_STRING);

    ui->cbAltitudeFilter->addItem("Low", 1);
    ui->cbAltitudeFilter->addItem("Medium", 2);
    ui->cbAltitudeFilter->addItem("High", 3);
    ui->cbAltitudeFilter->setCurrentIndex(2);

    ui->cbMaxPressure->addItem("< 1 kPa (K = 8192)", 8192);
    ui->cbMaxPressure->addItem("< 2 kPa (K = 4096)", 4096);
    ui->cbMaxPressure->addItem("< 4 kPa (K = 2048)", 2048);
    ui->cbMaxPressure->addItem("< 8 kPa (K = 1024)", 1024);
    ui->cbMaxPressure->addItem("< 16 kPa (K = 512)", 512);
    ui->cbMaxPressure->addItem("< 32 kPa (K = 256)", 256);
    ui->cbMaxPressure->addItem("< 65 kPa (K = 128)", 128);
    ui->cbMaxPressure->addItem("< 130 kPa (K = 64)", 64);
    ui->cbMaxPressure->addItem("< 260 kPa (K = 32)", 32);
    ui->cbMaxPressure->addItem("< 500 kPa (K = 16)", 16);
    ui->cbMaxPressure->addItem("< 1000 kPa (K = 8)", 8);
    ui->cbMaxPressure->addItem("> 1000 kPa (K = 4)", 4);

    ui->sbEscOffset->setVisible(false);
    ui->cbCurrentAutoOffset->setChecked(true);

    ui->cbGpsRate->addItems({"1", "5", "10", "20"});
    ui->cbSpeedUnitsGps->addItems({"km/h", "kts"});
    ui->lbQuiescentVoltage->setText("Zero current output voltage, V<sub>IOUT</sub> (V)");
    ui->cbVarioAutoOffset->setVisible(false);
    ui->cbSerialMonitorGpio->addItems({"1", "5", "6"});
    ui->cbBaudrate->addItems({"115200", "57600", "38400", "19200", "9600", "4800"});
    ui->cbLipoType->addItem({"INA3221"});
    ui->cbIna3221Filter->addItem("1", 0B000);
    ui->cbIna3221Filter->addItem("4", 0B001);
    ui->cbIna3221Filter->addItem("16", 0B010);
    ui->cbIna3221Filter->addItem("64", 0B011);
    ui->cbIna3221Filter->addItem("128", 0B100);
    ui->cbIna3221Filter->addItem("256", 0B101);
    ui->cbIna3221Filter->addItem("512", 0B110);
    ui->cbIna3221Filter->addItem("1024", 0B111);

    ui->cbGyroAccSens->addItems({"2", "4", "8", "16"});
    ui->cbGyroSens->addItems({"250", "500", "1000", "2000"});
    ui->cbGyroSamplerate->addItems({"1000", "500", "333", "250", "200", "167", "144", "125"});
    ui->cbGyroSamplerate->setVisible(false);
    ui->lbGyroSamplerate->setVisible(false);
    ui->cbDynModel->addItem("Portable", GPS_DYNMODEL_PORTABLE);
    ui->cbDynModel->addItem("Stationary", GPS_DYNMODEL_STATIONARY);
    ui->cbDynModel->addItem("Pedestrian", GPS_DYNMODEL_PEDESTRIAN);
    ui->cbDynModel->addItem("Automotive", GPS_DYNMODEL_AUTOMOTIVE);
    ui->cbDynModel->addItem("Sea", GPS_DYNMODEL_SEA);
    ui->cbDynModel->addItem("Airborne 1g", GPS_DYNMODEL_AIRBORNE1);
    ui->cbDynModel->addItem("Airborne 2g", GPS_DYNMODEL_AIRBORNE2);
    ui->cbDynModel->addItem("Airborne 4g", GPS_DYNMODEL_AIRBORNE4);
    ui->cbVSpeedInterval->addItem("250", 25);
    ui->cbVSpeedInterval->addItem("500", 50);
    ui->cbVSpeedInterval->addItem("1000", 100);
    ui->cbVSpeedInterval->addItem("1500", 150);
    ui->cbVSpeedInterval->addItem("2000", 200);
    ui->cbVSpeedInterval->addItem("2500", 250);

    ui->lbConnections->setText(
        "| Sensor/Receiver                           | Board GPIO|"
        "\n| :---:                                     | :---:            |"
        "\n| 3.3-5v                                    | 5v               |"
        "\n| GND                                       | GND              |"
        "\n| Smartport, SBUS, SRXL, IBUS, SB, Jeti Ex, Sanwa, Hott, SRXL2, FPort, FBUS, GHST  | 0<sup>(1)</sup> & 1 |"
        "\n| Frsky D, ELRS/CRSF Rx                     | 0                |"
        "\n| Serial monitor                            | 1                |"
        "\n| Hitec, XBUS SDA                           | 2<sup>(2)</sup>  |"
        "\n| Hitec, XBUS SCL                           | 3<sup>(2)</sup>  |"
        "\n| ESC serial, Serial monitor, Smart ESC     | 5                |"
        "\n| Phase sensor (PWM in), Smart ESC          | 4                |"
        "\n| Castle. Receiver signal                   | 4                |"
        "\n| Castle. ESC signal                        | 5<sup>(2)</sup>  |"
        "\n| GPS Tx                                    | 6                |"
        "\n| GPS Rx (3)(4)                             | 14               |"
        "\n| XBUS. NPN clock stretch<sup>(3)</sup>     | 7                |"
        "\n| Sensor SDA                                | 8<sup>(2)</sup>  |"
        "\n| Sensor SCL                                | 9<sup>(2)</sup>  |"
        "\n| PWM out                                   | 10               |"
        "\n| Fuel meter (PWM in)                       | 11               |"
        "\n| Throttle PWM (Smart ESC)                  | 12               |"
        "\n| Reverse PWM (Smart ESC)                   | 13               |"
        "\n| Restore default config                    | 15               |"
        "\n| Voltage                                   | 26               |"
        "\n| Current                                   | 27               |"
        "\n| NTC                                       | 28               |"
        "\n| Airspeed                                  | 29               |"
        "\n| GPIOs                                     | 17 to 22         |"
        "\n  "
        "\n(1) with 100Ω resistor. This resistor is optional as RP2040 has internal protection resistors for GPIOs  "
        "\n(2) Pullups. See [6.3 Spektrum "
        "XBUS](https://github.com/dgatf/msrc/wiki/06.-Receiver-protocol#63-spektrum-xbus) and [6.9 "
        "Hitec](https://github.com/dgatf/msrc/wiki/06.-Receiver-protocol#69-hitec)  "
        "\n(3) Optional  "
        "\n(4) Only UBLOX compatible  ");
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

    ui->lbCircuit->resize(600, 400);  //(ui->lbCircuit->parentWidget()->width(),
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
    QSize size(label->width() - 10, label->height() - 10);
    QPixmap pix(size);
    QPainter paint(&pix);
    QImage image;

    paint.fillRect(0, 0, size.width(), size.height(), label->palette().color(QPalette::Base));
    image.load(":/res/rp2040_zero.png");
    paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));

    int receiver = ui->cbReceiver->currentData().toInt();
    int esc = ui->cbEsc->currentData().toInt();
    if (receiver != SERIAL_MONITOR) {
        if (ui->gbCurrent->isChecked()) {
            image.load(":/res/current_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbVoltage1->isChecked()) {
            image.load(":/res/voltage_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbTemp1->isChecked()) {
            image.load(":/res/ntc_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbAirspeed->isChecked()) {
            image.load(":/res/airspeed_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbGps->isChecked()) {
            image.load(":/res/gps_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbEsc->isChecked()) {
            // ui->lbEsc->setEnabled(true);
            // ui->cbEsc->setEnabled(true);
            // ui->lbEscModel->setEnabled(true);
            // ui->lbEscModel->setEnabled(true);
            // ui->gbRpmMultipliers->setEnabled(true);
            if (esc == ESC_HW3 ||
                esc == ESC_HW4 ||
                esc == ESC_KONTRONIK || esc == ESC_APD_F ||
                esc == ESC_APD_HV || esc == ESC_HW5 ||
                esc == ESC_OMP_M4 || esc == ESC_ZTW ||
                esc == ESC_OPENYGE)
                image.load(":/res/esc_rp2040_zero.png");
            else if (esc == ESC_PWM)
                image.load(":/res/pwm_rp2040_zero.png");
            else if (esc == ESC_CASTLE)
                image.load(":/res/castle_rp2040_zero.png");
            else if (esc == ESC_SMART)
                image.load(":/res/smart_esc.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        } else {
        }

        if (ui->gbAltitude->isChecked()) {
            image.load(":/res/vario_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbGyro->isChecked()) {
            image.load(":/res/vario_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbFuelPressure->isChecked()) {
            image.load(":/res/fuel_pressure.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbFuelmeter->isChecked()) {
            image.load(":/res/fuel_meter.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (ui->gbLipo->isChecked()) {
            image.load(":/res/vario_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }

        if (receiver == RX_FRSKY_D || receiver == RX_CRSF || receiver == RX_JETIEX_SENSOR) {
            image.load(":/res/receiver_frsky_d_rp2040_zero.png");
        } else if (receiver == RX_XBUS) {
            image.load(":/res/receiver_xbus_rp2040_zero.png");
        } else if (receiver == RX_HITEC) {
            image.load(":/res/receiver_hitec_rp2040_zero.png");
        } else {
            image.load(":/res/receiver_serial_rp2040_zero.png");
        }
        if (receiver == RX_XBUS && ui->cbClockStretch->isChecked() == true) {
            image.load(":/res/clock_stretch_xbus_rp2040_zero.png");
            paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));
        }
    }
    paint.drawImage(QPoint(0, 0), image.scaled(size, Qt::IgnoreAspectRatio));

    label->setPixmap(pix);
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
    QMessageBox::information(this, tr("About"),
                             QString::asprintf("MSRC Link %s\n\rDaniel Gorbea © 2020/2025", PROJECT_VERSION),
                             QMessageBox::Close);
}

void MainWindow::buttonSerialPort() {
    if (isConnected)
        closeSerialPort();
    else
        openSerialPort();
}

void MainWindow::buttonDebug() {
    if (ui->btDebug->text() == "Enable Log") {
        if (!isConnected) return;
        ui->btDebug->setText("Disable Log");
        serial->readAll();
        char header = 0x30;
        serial->write(&header, 1);
        char command = 0x33;
        serial->write(&command, 1);
        isDebug = true;
    } else if (ui->btDebug->text() == "Disable Log") {
        if (!isConnected) return;
        ui->btDebug->setText("Enable Log");
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
        enableWidgets(ui->scrollAreaWidgetContentsReceiver, true);
        enableWidgets(ui->scrollAreaWidgetContentsSensors, true);
        ui->cbPortList->setDisabled(true);
        ui->btDebug->setEnabled(true);
        ui->btDebug->setText("Enable Log");
    } else {
        statusBar()->showMessage("Not connected: " + serial->errorString());
        isConnected = false;
        ui->btUpdate->setEnabled(false);
        ui->btConnect->setText("Connect");
        ui->actionUpdateConfig->setEnabled(false);
        ui->actionDefaultConfig->setEnabled(false);
        enableWidgets(ui->scrollAreaWidgetContentsReceiver, false);
        enableWidgets(ui->scrollAreaWidgetContentsSensors, false);
        ui->btDebug->setEnabled(false);
        ui->btDebug->setText("Enable Log");
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
    enableWidgets(ui->scrollAreaWidgetContentsReceiver, false);
    enableWidgets(ui->scrollAreaWidgetContentsSensors, false);
    ui->cbPortList->setDisabled(false);
    ui->btDebug->setDisabled(true);
    ui->btDebug->setText("Enable Log");
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

    if (config.serial_monitor_gpio == 1)
        ui->cbSerialMonitorGpio->setCurrentText("1");
    else if (config.serial_monitor_gpio == 5)
        ui->cbSerialMonitorGpio->setCurrentText("5");
    else
        ui->cbSerialMonitorGpio->setCurrentText("6");
    int item = ui->cbBaudrate->findText(QString::number(config.serial_monitor_baudrate));
    if (item == -1)
        ui->cbBaudrate->setCurrentText(QString::number(config.serial_monitor_baudrate));
    else
        ui->cbBaudrate->setCurrentIndex(item);
    if (config.serial_monitor_parity > 2) config.serial_monitor_parity = 0;
    ui->cbParity->setCurrentIndex(ui->cbParity->findData(config.serial_monitor_parity));
    if (config.serial_monitor_stop_bits > 2 || config.serial_monitor_stop_bits < 1) config.serial_monitor_stop_bits = 1;
    ui->cbStopbits->setCurrentIndex(ui->cbStopbits->findData(config.serial_monitor_stop_bits));
    if (config.serial_monitor_timeout_ms > 100) config.serial_monitor_timeout_ms = 100;
    ui->sbTimeout->setValue(config.serial_monitor_timeout_ms);
    ui->cbInverted->setChecked(config.serial_monitor_inverted);
    ui->cbSerialFormat->setCurrentIndex(ui->cbSerialFormat->findData(config.serial_monitor_format));

    /* Sensors */

    // ESC

    if (config.esc_protocol == esc_protocol_t::ESC_NONE)
        ui->gbEsc->setChecked(false);
    else {
        ui->gbEsc->setChecked(true);
        ui->cbEsc->setCurrentIndex(ui->cbEsc->findData(config.esc_protocol));
    }

    // GPS

    ui->gbGps->setChecked(config.enable_gps);
    ui->cbGpsBaudrate->setCurrentText(QString::number(config.gps_baudrate));
    ui->cbGpsRate->setCurrentText(QString::number(config.gps_rate));
    ui->cbGpsProtocol->setCurrentIndex(ui->cbGpsProtocol->findData(config.gps_protocol));
    ui->cbDynModel->setCurrentIndex(
        ui->cbDynModel->findData(config.gps_dynmodel)
    );

    // Analog rate

    ui->sbAnalogRate->setValue(config.analog_rate);

    // Voltage

    ui->gbVoltage1->setChecked(config.enable_analog_voltage);
    if (config.sbus_battery_slot == true) {
        ui->ckSbusBattery->setChecked(true);
        ui->ckSbusExtVolt->setChecked(false);
    } else {
        ui->ckSbusBattery->setChecked(false);
        ui->ckSbusExtVolt->setChecked(true);
    }

    // Temperature

    ui->gbTemp1->setChecked(config.enable_analog_ntc);
    ui->sbTempOffset->setValue(config.ntc_offset);

    // Current

    ui->gbCurrent->setChecked(config.enable_analog_current);
    ui->cbCurrentSensorType->setCurrentIndex(ui->cbCurrentSensorType->findData(config.analog_current_type));
    ui->cbCurrentAutoOffset->setChecked(config.analog_current_autoffset);
    ui->sbQuiescentVoltage->setValue(config.analog_current_quiescent_voltage);
    if (config.analog_current_type == analog_current_type_t::CURRENT_TYPE_HALL)
        ui->sbCurrentSens->setValue(1000 / config.analog_current_multiplier);
    else
        ui->sbAnalogCurrentMultiplier->setValue(config.analog_current_multiplier);

    // Airspeed

    ui->gbAirspeed->setChecked(config.enable_analog_airspeed);
    ui->sbAirspeedVcc->setValue(config.airspeed_vcc / 100.0);
    ui->sbAirspeedOffset->setValue(config.airspeed_offset);

    // Vario

    if (config.i2c_module == i2c_module_t::I2C_NONE)
        ui->gbAltitude->setChecked(false);
    else
        ui->gbAltitude->setChecked(true);
    ui->cbBarometerType->setCurrentIndex(ui->cbBarometerType->findData(config.i2c_module));
    ui->cbAltitudeFilter->setCurrentIndex(ui->cbAltitudeFilter->findData(config.bmp280_filter));
    ui->cbVarioAutoOffset->setChecked(config.vario_auto_offset);
    int vSpeedIndex = ui->cbVSpeedInterval->findData(config.vario_vspeed_interval);
    ui->cbVSpeedInterval->setCurrentIndex(vSpeedIndex == -1 ? 0 : vSpeedIndex);

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

     ui->sbSensorIdJeti->setValue(config.sensor_id_jeti);

    // FPort

    ui->cbFPortInverted->setChecked(config.fport_inverted);

    // FBUS

    ui->cbFbusInverted->setChecked(config.fbus_inverted);

    // HW V4/V5 parameters

    ui->cbInitDelay->setChecked(config.enable_esc_hw4_init_delay);
    ui->cbEscAutoOffset->setChecked(!config.esc_hw4_is_manual_offset);
    ui->sbEscOffset->setValue(config.esc_hw4_offset);
    // config.esc_hw4_init_delay_duration = 10000;
    ui->sbVoltageMultiplier->setValue(config.esc_hw4_voltage_multiplier * 100000);
    ui->sbCurrentMultiplier->setValue(config.esc_hw4_current_multiplier * 100000);
    ui->cbHw4AutoDetect->setChecked(config.esc_hw4_auto_detect);

    // Smart esc

    ui->cbCalculateConsumption->setChecked(config.smart_esc_calc_consumption);

    // Fuel flow

    ui->gbFuelmeter->setChecked(config.enable_fuel_flow);
    ui->sbMlPulse->setValue(config.fuel_flow_ml_per_pulse);

    // Fuel pressure

    ui->gbFuelPressure->setChecked(config.enable_fuel_pressure);
    ui->cbMaxPressure->setCurrentIndex(ui->cbMaxPressure->findData(config.xgzp68xxd_k));

    // GPIOs
    ui->cbGpio17->setChecked(config.gpio_mask & 1);
    ui->cbGpio18->setChecked(config.gpio_mask & (1 << 1));
    ui->cbGpio19->setChecked(config.gpio_mask & (1 << 2));
    ui->cbGpio20->setChecked(config.gpio_mask & (1 << 3));
    ui->cbGpio21->setChecked(config.gpio_mask & (1 << 4));
    ui->cbGpio22->setChecked(config.gpio_mask & (1 << 5));
    ui->sbGpioInterval->setValue(config.gpio_interval);

    // Gyro MPU6050
    ui->gbGyro->setChecked(config.enable_gyro);
    ui->cbGyroAccSens->setCurrentIndex(config.mpu6050_acc_scale);
    ui->cbGyroSens->setCurrentIndex(config.mpu6050_gyro_scale);
    ui->sbGyroWeight->setValue(config.mpu6050_gyro_weighting);
    ui->sbGyroFilter->setValue(config.mpu6050_filter);
    // ui->cbGpsRate->setValue

    // INA3221 (lipo)
    ui->cbIna3221Filter->setCurrentIndex(ui->cbIna3221Filter->findData(config.ina3221_filter));
    ui->sbLipoCells->setValue(config.lipo_cells);
    ui->gbLipo->setChecked(config.enable_lipo);
    ui->ckLipoCurrent->setChecked(config.lipo_current);
    uint shunt = config.lipo_current_shunt;
    if (shunt < 1 || shunt > 100) shunt = 100;
    ui->sbLipoShunt->setValue(shunt);

    // SRXL2
    uint sensor_id_srxl2 = config.sensor_id_srxl2 ;
    if (sensor_id_srxl2 == 0 || sensor_id_srxl2 > 0x0F) sensor_id_srxl2 = 0x01;
    ui->sbSensorIdSrxl2->setValue(sensor_id_srxl2);
}

void MainWindow::getConfigFromUi() {
    /* Config version  */

    config.version = CONFIG_VERSION;

    /* Receiver protocol */

    config.rx_protocol = (rx_protocol_t)ui->cbReceiver->currentData().toInt();

    /* Serial Monitor */

    config.serial_monitor_baudrate = ui->cbBaudrate->currentText().toInt();
    config.serial_monitor_gpio = ui->cbSerialMonitorGpio->currentText().toInt();
    config.serial_monitor_stop_bits = ui->cbStopbits->currentData().toInt();
    config.serial_monitor_parity = ui->cbParity->currentData().toUInt();
    config.serial_monitor_timeout_ms = ui->sbTimeout->value();
    config.serial_monitor_inverted = ui->cbInverted->isChecked();
    config.serial_monitor_format =(serial_monitor_format_t)(ui->cbSerialFormat->currentData().toUInt());

    /* Sensors */

    // ESC

    if (ui->gbEsc->isChecked())
        config.esc_protocol = (esc_protocol_t)ui->cbEsc->currentData().toInt();
    else
        config.esc_protocol = esc_protocol_t::ESC_NONE;

    // GPS

    config.enable_gps = ui->gbGps->isChecked();
    config.gps_baudrate = ui->cbGpsBaudrate->currentText().toInt();
    config.gps_rate = ui->cbGpsRate->currentText().toInt();
    config.gps_protocol = ui->cbGpsProtocol->currentData().toUInt();
    config.gps_dynmodel = ui->cbDynModel->currentData().toUInt();

    // Voltage

    config.enable_analog_voltage = ui->gbVoltage1->isChecked();
    config.analog_voltage_multiplier = ui->sbVoltage1Mult->value();
    config.sbus_battery_slot = ui->ckSbusBattery->isChecked();

    // Current

    config.enable_analog_current = ui->gbCurrent->isChecked();
    config.analog_current_type = (analog_current_type_t)ui->cbCurrentSensorType->currentData().toInt();
    config.analog_current_quiescent_voltage = ui->sbQuiescentVoltage->value();
    if (ui->cbCurrentSensorType->currentData().toInt() == CURRENT_TYPE_HALL) {
        if (ui->cbCurrentAutoOffset->isChecked()) {
            config.analog_current_autoffset = true;
            config.analog_current_offset = 0;
        } else {
            config.analog_current_autoffset = false;
            config.analog_current_offset = ui->sbQuiescentVoltage->value();
        }
        config.analog_current_multiplier = 1000 / ui->sbCurrentSens->value();
    } else if (ui->cbCurrentSensorType->currentData().toInt() == CURRENT_TYPE_SHUNT) {
        config.analog_current_autoffset = false;
        config.analog_current_offset = 0;
        config.analog_current_multiplier = ui->sbAnalogCurrentMultiplier->value();
    }

    // Temperature

    config.enable_analog_ntc = ui->gbTemp1->isChecked();
    config.ntc_offset = ui->sbTempOffset->value();

    // Airspeed

    config.enable_analog_airspeed = ui->gbAirspeed->isChecked();
    config.airspeed_vcc = ui->sbAirspeedVcc->value() * 100;
    config.airspeed_offset = ui->sbAirspeedOffset->value();

    // Vario

    if (ui->gbAltitude->isChecked())
        config.i2c_module = (i2c_module_t)(ui->cbBarometerType->currentData().toInt());
    else
        config.i2c_module = i2c_module_t::I2C_NONE;
    config.bmp280_filter = ui->cbAltitudeFilter->currentData().toUInt();
    config.vario_auto_offset = ui->cbVarioAutoOffset->isChecked();

    config.vario_vspeed_interval =  ui->cbVSpeedInterval->currentData().toUInt();

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

    config.sensor_id_jeti = ui->sbSensorIdJeti->value();

    // Ibus

    config.ibus_alternative_coordinates = ui->cbAlternativeCoordinates->isChecked();

    // FPort

    config.fport_inverted = ui->cbFPortInverted->isChecked();

    // FBUS

    config.fbus_inverted = ui->cbFbusInverted->isChecked();

    // HW V4/V5 parameters

    config.enable_esc_hw4_init_delay = ui->cbInitDelay->isChecked();
    config.esc_hw4_is_manual_offset = !ui->cbEscAutoOffset->isChecked();
    config.esc_hw4_offset = ui->sbEscOffset->value();
    config.esc_hw4_voltage_multiplier = ui->sbVoltageMultiplier->value() / 100000.0;
    config.esc_hw4_current_multiplier = ui->sbCurrentMultiplier->value() / 100000.0;
    config.esc_hw4_auto_detect = ui->cbHw4AutoDetect->isChecked();

    // Smart esc

    config.smart_esc_calc_consumption = ui->cbCalculateConsumption->isChecked();

    // Fuel flow

    config.enable_fuel_flow = ui->gbFuelmeter->isChecked();
    config.fuel_flow_ml_per_pulse = ui->sbMlPulse->value();

    // Fuel pressure

    config.enable_fuel_pressure = ui->gbFuelPressure->isChecked();
    config.xgzp68xxd_k = ui->cbMaxPressure->currentData().toUInt();

    // GPIOs
    config.gpio_mask = ui->cbGpio17->isChecked();
    config.gpio_mask |= ui->cbGpio18->isChecked() << 1;
    config.gpio_mask |= ui->cbGpio19->isChecked() << 2;
    config.gpio_mask |= ui->cbGpio20->isChecked() << 3;
    config.gpio_mask |= ui->cbGpio21->isChecked() << 4;
    config.gpio_mask |= ui->cbGpio22->isChecked() << 5;
    config.gpio_interval = ui->sbGpioInterval->value();

    // Gyro MPU6050
    config.enable_gyro = ui->gbGyro->isChecked();
    config.mpu6050_acc_scale = ui->cbGyroAccSens->currentIndex();
    config.mpu6050_gyro_scale = ui->cbGyroSens->currentIndex();
    config.mpu6050_gyro_weighting = ui->sbGyroWeight->value();
    config.mpu6050_filter = ui->sbGyroFilter->value();
    // config.mpu6050_rate = ui->cbGyroSamplerate->currentText().toInt();

    // INA3221 (lipo)
    config.ina3221_filter = ui->cbIna3221Filter->currentData().toUInt();
    config.lipo_cells = ui->sbLipoCells->value();
    config.enable_lipo = ui->gbLipo->isChecked();
    config.lipo_current = ui->ckLipoCurrent->isChecked();
    config.lipo_current_shunt = ui->sbLipoShunt->value();

    // SRXL2
    config.sensor_id_srxl2 = ui->sbSensorIdSrxl2->value();
    if (config.sensor_id_srxl2 < 0x01 || config.sensor_id_srxl2 > 0x0F) config.sensor_id_srxl2 = 0x01;

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

void MainWindow::on_cbReceiver_currentIndexChanged(int index)
{
    int value = ui->cbReceiver->itemData(index).toInt();
    if (value == RX_XBUS) {
        ui->cbClockStretch->setVisible(true);
    } else {
        ui->cbClockStretch->setVisible(false);
    }

    if (value == RX_SRXL2) {
        ui->lbSensorIdSrxl2->setVisible(true);
        ui->sbSensorIdSrxl2->setVisible(true);
    } else {
        ui->lbSensorIdSrxl2->setVisible(false);
        ui->sbSensorIdSrxl2->setVisible(false);
    }

    if (value == RX_XBUS || value == RX_SRXL || value == RX_SRXL2) {
        ui->cbAlternativePacket->setVisible(true);
    } else {
        ui->cbAlternativePacket->setVisible(false);
    }

    if (value == RX_SMARTPORT || value == RX_FRSKY_D || value == RX_FPORT || value == RX_FBUS) {
        ui->gbRate->setVisible(true);
    } else {
        ui->gbRate->setVisible(false);
    }

    if (value == RX_FPORT) {
        ui->cbFPortInverted->setVisible(true);
    } else {
        ui->cbFPortInverted->setVisible(false);
    }

    if (value == RX_FBUS) {
        ui->cbFbusInverted->setVisible(true);
    } else {
        ui->cbFbusInverted->setVisible(false);
    }

    if (value == RX_SMARTPORT || value == RX_FBUS) {
        ui->lbSensorId->setVisible(true);
        ui->sbSensorId->setVisible(true);
    } else {
        ui->lbSensorId->setVisible(false);
        ui->sbSensorId->setVisible(false);
    }

    if (value == RX_IBUS) {
        ui->cbAlternativeCoordinates->setVisible(true);
    } else {
        ui->cbAlternativeCoordinates->setVisible(false);
    }

    if (value == RX_JETIEX || value == RX_JETIEX_SENSOR) {
        ui->cbSpeedUnitsGps->setVisible(true);
        ui->lbSpeedUnitsGps->setVisible(true);
        ui->lbSensorIdJeti->setVisible(true);
        ui->sbSensorIdJeti->setVisible(true);
    } else {
        ui->cbSpeedUnitsGps->setVisible(false);
        ui->lbSpeedUnitsGps->setVisible(false);
        ui->lbSensorIdJeti->setVisible(false);
        ui->sbSensorIdJeti->setVisible(false);
    }

    if (value == RX_SBUS) {
        ui->ckSbusBattery->setVisible(true);
        ui->ckSbusExtVolt->setVisible(true);
    } else {
        ui->ckSbusBattery->setVisible(false);
        ui->ckSbusExtVolt->setVisible(false);
    }

    if (value == SERIAL_MONITOR) {
        ui->cbBaudrate->setVisible(true);
        ui->cbStopbits->setVisible(true);
        ui->cbParity->setVisible(true);
        ui->sbTimeout->setVisible(true);
        ui->cbInverted->setVisible(true);
        ui->lbBaudrate->setVisible(true);
        ui->lbStopbits->setVisible(true);
        ui->lbParity->setVisible(true);
        ui->lbTimeout->setVisible(true);
        ui->lbSerialFormat->setVisible(true);
        ui->cbSerialFormat->setVisible(true);
        ui->gbSensors->setVisible(false);
        ui->gbAverage->setVisible(false);
        ui->lbSerialMonitorGpio->setVisible(true);
        ui->cbSerialMonitorGpio->setVisible(true);
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
        ui->lbSerialFormat->setVisible(false);
        ui->cbSerialFormat->setVisible(false);
        ui->gbSensors->setVisible(true);
        ui->gbAverage->setVisible(true);
        ui->lbSerialMonitorGpio->setVisible(false);
        ui->cbSerialMonitorGpio->setVisible(false);
    }

    // Fuel meter
    if (value == RX_SMARTPORT || value == RX_JETIEX || value == RX_JETIEX_SENSOR || value == RX_XBUS ||
        value == RX_HOTT || value == RX_FPORT || value == RX_FBUS) {
        ui->gbFuelmeter->setVisible(true);
    } else {
        ui->gbFuelmeter->setVisible(false);
    }

    // Fuel pressure
    if (value == RX_SRXL || value == RX_SRXL2 || value == RX_JETIEX || value == RX_JETIEX_SENSOR ||
        value == RX_XBUS || value == RX_HOTT) {
        ui->gbFuelPressure->setVisible(true);
    } else {
        ui->gbFuelPressure->setVisible(false);
    }

    // GPIO
    if (value == RX_SMARTPORT || value == RX_FPORT || value == RX_FBUS) {
        ui->gbGpio->setVisible(true);
    } else {
        ui->gbGpio->setVisible(false);
    }

    // Airspeed
    if (value == RX_SANWA || value == RX_GHST) {
        ui->gbAirspeed->setVisible(false);
    } else {
        ui->gbAirspeed->setVisible(true);
    }

    // Temperature
    if (value == RX_GHST) {
        ui->gbTemp1->setVisible(false);
    } else {
        ui->gbTemp1->setVisible(true);
    }

    // GPS, current, vario
    if (value == RX_SANWA) {
        ui->gbGps->setVisible(false);
        ui->gbCurrent->setVisible(false);
        ui->gbAltitude->setVisible(false);
    } else {
        ui->gbGps->setVisible(true);
        ui->gbCurrent->setVisible(true);
        ui->gbAltitude->setVisible(true);
    }

    // Lipo
    if (value == RX_CRSF || value == RX_SMARTPORT || value == RX_FPORT || value == RX_FBUS || value == RX_HOTT || value == RX_JETIEX || value == RX_JETIEX_SENSOR || value == RX_SRXL || value == RX_SRXL2) {
        ui->gbLipo->setVisible(true);
    } else {
        ui->gbLipo->setVisible(false);
    }

    // Average elements
    if (value == RX_SANWA) {
        ui->lbRpmAvg->setVisible(true);
        ui->sbRpmAvg->setVisible(true);
        ui->lbVoltageAvg->setVisible(true);
        ui->sbVoltageAvg->setVisible(true);
        ui->lbCurrentAvg->setVisible(false);
        ui->sbCurrentAvg->setVisible(false);
        ui->lbTemperatureAvg->setVisible(true);
        ui->sbTemperatureAvg->setVisible(true);
        ui->lbVarioAvg->setVisible(false);
        ui->sbVarioAvg->setVisible(false);
        ui->lbAirspeedAvg->setVisible(false);
        ui->sbAirspeedAvg->setVisible(false);
    } else if (value == RX_GHST) {
        ui->lbRpmAvg->setVisible(false);
        ui->sbRpmAvg->setVisible(false);
        ui->lbVoltageAvg->setVisible(true);
        ui->sbVoltageAvg->setVisible(true);
        ui->lbCurrentAvg->setVisible(true);
        ui->sbCurrentAvg->setVisible(true);
        ui->lbTemperatureAvg->setVisible(false);
        ui->sbTemperatureAvg->setVisible(false);
        ui->lbVarioAvg->setVisible(true);
        ui->sbVarioAvg->setVisible(true);
        ui->lbAirspeedAvg->setVisible(false);
        ui->sbAirspeedAvg->setVisible(false);
    } else {
        ui->lbRpmAvg->setVisible(true);
        ui->sbRpmAvg->setVisible(true);
        ui->lbVoltageAvg->setVisible(true);
        ui->sbVoltageAvg->setVisible(true);
        ui->lbCurrentAvg->setVisible(true);
        ui->sbCurrentAvg->setVisible(true);
        ui->lbTemperatureAvg->setVisible(true);
        ui->sbTemperatureAvg->setVisible(true);
        ui->lbVarioAvg->setVisible(true);
        ui->sbVarioAvg->setVisible(true);
        ui->lbAirspeedAvg->setVisible(true);
        ui->sbAirspeedAvg->setVisible(true);
    }

    // Vario
    if (value == RX_XBUS || value == RX_HOTT) {
        ui->lbVSpeedInterval->setVisible(false);
        ui->cbVSpeedInterval->setVisible(false);
    } else {
        ui->lbVSpeedInterval->setVisible(true);
        ui->cbVSpeedInterval->setVisible(true);
    }
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_cbEsc_currentIndexChanged(int index)
{
    int value = ui->cbEsc->itemData(index).toInt();
    if (value == ESC_SMART)
        ui->cbCalculateConsumption->setVisible(true);
    else
        ui->cbCalculateConsumption->setVisible(false);
    if (value == ESC_HW4) {
        ui->gbEscParameters->setVisible(true);
        ui->cbPwmOut->setVisible(true);
        ui->cbHw4AutoDetect->setVisible(true);
    } else {
        ui->gbEscParameters->setVisible(false);
        ui->cbPwmOut->setVisible(false);
        ui->cbHw4AutoDetect->setVisible(false);
    }
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbEsc_toggled(bool enabled) {
    enableWidgets(ui->gbEsc, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbVoltage1_toggled(bool enabled) {
    enableWidgets(ui->gbVoltage1, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_gbTemp1_toggled(bool checked) {
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

void MainWindow::on_cbBarometerType_currentIndexChanged(int index)
{
    int value = ui->cbBarometerType->itemData(index).toInt();
    if (value == I2C_BMP280) {
        ui->cbAltitudeFilter->setVisible(true);
        ui->lbAltitudeFilter->setVisible(true);
    } else {
        ui->cbAltitudeFilter->setVisible(false);
        ui->lbAltitudeFilter->setVisible(false);
    }
    // Q_UNUSED(arg1);
    // generateCircuit(ui->lbCircuit);
}

void MainWindow::on_btCircuit_clicked() {
    CircuitDialog circuitDialog;
    circuitDialog.setModal(true);
    circuitDialog.mainWindow = this;
    circuitDialog.exec();
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
    } else if (ui->cbCurrentSensorType->currentData().toInt() == analog_current_type_t::CURRENT_TYPE_HALL) {
        ui->lbQuiescentVoltage->setVisible(true);
        ui->sbQuiescentVoltage->setVisible(true);
    }
}

void MainWindow::on_cbCurrentSensorType_currentIndexChanged(int index)
{
    int value = ui->cbCurrentSensorType->itemData(index).toInt();
    if (value == CURRENT_TYPE_HALL) {
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
    if (value == CURRENT_TYPE_SHUNT) {
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

void MainWindow::on_gbFuelPressure_toggled(bool enabled) {
    enableWidgets(ui->gbFuelPressure, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_ckSbusBattery_toggled(bool checked) { ui->ckSbusExtVolt->setChecked(!checked); }

void MainWindow::on_ckSbusExtVolt_toggled(bool checked) { ui->ckSbusBattery->setChecked(!checked); }

void MainWindow::on_cbHw4AutoDetect_toggled(bool checked) {
    if (checked) {
        ui->gbEscParameters->setVisible(false);
    } else {
        ui->gbEscParameters->setVisible(true);
    }
}

void MainWindow::on_gbGyro_toggled(bool enabled) {
    enableWidgets(ui->gbGyro, enabled);
    generateCircuit(ui->lbCircuit);
}

void MainWindow::on_ckLipoCurrent_toggled(bool enabled)
{
    ui->lbLipoShunt->setVisible(enabled);
    ui->sbLipoShunt->setVisible(enabled);
}


