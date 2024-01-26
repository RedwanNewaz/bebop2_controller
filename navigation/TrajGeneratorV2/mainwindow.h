#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHash>
#include <QDebug>
#include <QProcess>
#include <memory>
#include "eight.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_verticalSlider_sliderMoved(int position);

    void on_horizontalSlider_sliderMoved(int position);

    void on_saveButton_clicked();

    void on_pushButton_clicked();

    void on_processStandardOutput();

    void on_radioButtonSpiral_clicked();

    void on_radioButtonRect_clicked();

private:
    Ui::MainWindow *ui;
//    QVector<GeomPath::Eight*> paths_;
    GeomPath::Eight *path_;
    double pathScaleX_, pathScaleY_;
    QHash<int, QString> methods_;
    int sendCounter_;
    QProcess *proc;
};
#endif // MAINWINDOW_H
