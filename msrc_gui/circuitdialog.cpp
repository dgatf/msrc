#include "circuitdialog.h"
#include "ui_circuitdialog.h"

CircuitDialog::CircuitDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CircuitDialog)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
    ui->saCircuit->takeWidget();
    ui->saCircuit->setWidget(ui->lbCircuit);
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
    if (ui->lbCircuit->height() < ui->saCircuit->height()) ui->lbCircuit->setFixedHeight(ui->saCircuit->height());
    if (ui->lbCircuit->width() < ui->saCircuit->width()) ui->lbCircuit->setFixedWidth(ui->saCircuit->width());
    mainWindow->generateCircuit(ui->lbCircuit);
}

void CircuitDialog::on_btZoomIn_released()
{
     ui->lbCircuit->setFixedHeight(ui->lbCircuit->height() * 1.2);
     ui->lbCircuit->setFixedWidth(ui->lbCircuit->width() * 1.2);
     mainWindow->generateCircuit(ui->lbCircuit);
}

void CircuitDialog::on_btZoomOut_released()
{
     ui->lbCircuit->setFixedHeight(ui->lbCircuit->height() / 1.2);
     ui->lbCircuit->setFixedWidth(ui->lbCircuit->width() / 1.2);
     if (ui->lbCircuit->height() < ui->saCircuit->height()) ui->lbCircuit->setFixedHeight(ui->saCircuit->height());
     if (ui->lbCircuit->width() < ui->saCircuit->width()) ui->lbCircuit->setFixedWidth(ui->saCircuit->width());
     mainWindow->generateCircuit(ui->lbCircuit);
}
