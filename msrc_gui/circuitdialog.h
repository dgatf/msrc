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
    MainWindow *mainWindow;

    explicit CircuitDialog(QWidget *parent = nullptr);
    ~CircuitDialog();
    void resizeEvent(QResizeEvent* event);

private slots:
    void on_btClose_clicked();
    void on_btZoomIn_released();
    void on_btZoomOut_released();

private:
    Ui::CircuitDialog *ui;
};

#endif // CIRCUITDIALOG_H
