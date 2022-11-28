#include "circuitdialog.h"
#include "ui_circuitdialog.h"

CircuitDialog::CircuitDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CircuitDialog)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::SubWindow);
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
    mainWindow->generateCircuit(ui->lbCircuit);
}

