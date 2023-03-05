#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QVector>
#include "dfa.h"
#include <memory>
#include <QString>
#include <QSettings>
#include <QFileDialog>
#include <QProcess>
#include "CPUSnapshot.h"

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
    void on_btn_start_clicked();
    void timer_callback();
    void on_op_type_sim_toggled(bool checked);

    void on_op_type_exp_toggled(bool checked);

    void on_actionset_bag_file_triggered();



protected:
    void set_states();
    bool file_exist(const QString& path);

private:
    Ui::MainWindow *ui;
    DFA *m_dfa;
    QTimer *m_timer;
    QVector<bool> m_state;
    QSettings *m_settings;
    std::unique_ptr<CPUSnapshot> currentSnap, previousSnap;
    bool m_button_status;
    QProcess *m_proc;
};
#endif // MAINWINDOW_H
