#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QClipboard>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

private slots:
    void generateConfig();
    void exitApp();


    void on_cbReceiver_currentIndexChanged(const QString &arg1);
    void on_cbEsc_currentIndexChanged(const QString &arg1);
    void on_gbEsc_toggled(bool arg1);
    void on_btCopy_clicked();
    void on_cbEscModel_currentIndexChanged(const QString &arg1);
    void on_cbBarometerType_currentIndexChanged(const QString &arg1);
};
#endif // MAINWINDOW_H
