#include "circuitdialog.h"
#include "ui_circuitdialog.h"

CircuitDialog::CircuitDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CircuitDialog)
{
    ui->setupUi(this);
    setWindowFlags(windowFlags() | Qt::CustomizeWindowHint |
                                   Qt::WindowMinimizeButtonHint |
                                   Qt::WindowMaximizeButtonHint |
                                   Qt::WindowCloseButtonHint);
    ui->lbCircuit->resize(this->size());
    lbCircuit = ui->lbCircuit;
}

CircuitDialog::~CircuitDialog()
{
    delete ui;
}

void CircuitDialog::on_btClose_clicked()
{
    this->hide();
}

void CircuitDialog::resizeEvent(QResizeEvent* event)
{
    Q_UNUSED(event);
    ui->lbCircuit->resize(this->size());
    mainWindow->generateCircuit(ui->lbCircuit);
}

