#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QVector>
#include <memory>
#include <QString>
#include <QProcess>
#include "CPUSnapshot.h"
#include "dfa.h"
#include "process_manager.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/// @brief It acts as a graphical user interface of the program. It provides users with a graphical interface to interact with the application and access its functionality.
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /// @brief Constructor for the Main Window. It initializes the UI elements and sets the initial state of the program 
    /// @param parent used to specify the parent of the current widget
    MainWindow(QWidget *parent = nullptr);
    /// @brief Destructor method for the MainWindow class. It cleans up any resources that were allocated by the class.
    ~MainWindow();

private slots:
    /// @brief A method that sets the current state of the various checkboxes in the GUI based on the options selected by the user.    void on_btn_start_clicked();
    void timer_callback();
    void on_op_type_sim_toggled(bool checked);

    void on_op_type_exp_toggled(bool checked);





protected:
    void set_states();


private:
    Ui::MainWindow *ui;
    DFA *m_dfa;
    QTimer *m_timer;
    QVector<bool> m_state;
    ProcessManager *m_manager_;

    std::unique_ptr<CPUSnapshot> currentSnap, previousSnap;
    bool m_button_status;
    QProcess *m_proc;
};
#endif // MAINWINDOW_H
