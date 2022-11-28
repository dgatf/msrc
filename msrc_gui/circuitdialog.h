#ifndef CIRCUITDIALOG_H
#define CIRCUITDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QDebug>
#include "mainwindow.h"

namespace Ui {
class CircuitDialog;
}

class CircuitDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CircuitDialog(QWidget *parent = nullptr);
    ~CircuitDialog();
    QLabel *lbCircuit;
    MainWindow *mainWindow;
    void resizeEvent(QResizeEvent* event);


private slots:
    void on_btClose_clicked();

private:
    Ui::CircuitDialog *ui;
};

#endif // CIRCUITDIALOG_H
