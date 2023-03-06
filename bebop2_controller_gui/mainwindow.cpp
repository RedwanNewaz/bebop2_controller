#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QFileInfo>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle("bebop2_controller_gui");
    setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);

    m_dfa = new DFA();
    // resize internal state
    m_state.resize(m_dfa->size());
    fill(m_state.begin(), m_state.end(), false);
    set_states();

    // start process manager
    m_manager_ = new ProcessManager();
    connect(ui->actionset_bag_file, SIGNAL(triggered()), m_manager_, SLOT(on_bag_file_triggered()));
    connect(ui->actionset_rviz_file, SIGNAL(triggered()), m_manager_, SLOT(on_rviz_file_triggered()));

    // set cpu usage
    previousSnap = std::make_unique<CPUSnapshot>();
    // start timer
    m_timer = new QTimer();
    connect(m_timer, SIGNAL(timeout(void)), this, SLOT(timer_callback(void)));
    m_timer->start(300);

    // button state
    m_button_status = false;
}

MainWindow::~MainWindow()
{
    delete ui;
    delete m_dfa;
    delete m_timer;
}


void MainWindow::on_btn_start_clicked()
{

    // toggle button color red / green
    QPalette pal = ui->btn_start->palette();
    if(!m_button_status)
    {
        qDebug() << "[MainWindow]: start button clicked";
        std::set<int>pids;
        for(int i=0; i<m_state.size(); ++i)
            if(m_state[i])
                pids.insert(i);

        pids.insert(START);
        m_manager_->start(pids);


        m_button_status = true;
        ui->btn_start->setText("Stop");
        ui->btn_start->setProperty("color", "red");
        pal.setColor(QPalette::Button, QColor(Qt::red));
        ui->btn_start->setAutoFillBackground(true);
        ui->btn_start->setPalette(pal);
        ui->btn_start->update();

    }
    else
    {

        m_manager_->stop();
        m_button_status = false;
        ui->btn_start->setText("Start");
        qDebug() << "[MainWindow]: stop button clicked";
        pal.setColor(QPalette::Button, QColor(Qt::gray));
        ui->btn_start->setAutoFillBackground(true);
        ui->btn_start->setPalette(pal);
        ui->btn_start->update();
    }

}

void MainWindow::timer_callback()
{
    QVector<bool>temp(m_state.size());
    temp[EXP]  = ui->op_type_exp->isChecked();
    temp[SIM] = ui->op_type_sim->isChecked();
    temp[BAG] = ui->sim_type_bag->isChecked();
    temp[DUMMY] = ui->sim_type_dummy->isChecked();
    temp[APRILTAG] = ui->node_type_apriltag->isChecked();
    temp[BEBOP] = ui->node_type_bebop->isChecked();
    temp[JOY] = ui->node_type_joy->isChecked();
    temp[RVIZ] = ui->node_type_rviz->isChecked();
    temp[EKF] = ui->filter_type_ekf->isChecked();
    temp[LOWPASS] = ui->filter_type_lowpass->isChecked();




    list<int>q;
    for(int i=0; i<m_dfa->size(); ++i)
        if(temp[i])
            q.push_back(i);
    q.push_back(START);
    if(m_dfa->isValid(q))
    {
        std::copy(temp.begin(), temp.end(), m_state.begin());
        set_states();
        ui->btn_start->setEnabled(true);
    }
    else
    {
        //qDebug()<< "[MainWindow] invalid choice";
        ui->btn_start->setEnabled(false);
    }

    // compute cpu usage
    currentSnap = std::make_unique<CPUSnapshot>();

    const float ACTIVE_TIME = currentSnap->GetActiveTimeTotal() - previousSnap->GetActiveTimeTotal();
    const float IDLE_TIME   = currentSnap->GetIdleTimeTotal() - previousSnap->GetIdleTimeTotal();
    const float TOTAL_TIME  = ACTIVE_TIME + IDLE_TIME;
    int usage = 100.f * ACTIVE_TIME / TOTAL_TIME;
    //    std::cout << "total cpu usage: " << usage << " %" << std::endl;
    previousSnap = std::make_unique<CPUSnapshot>();
    ui->cpu_usage_bar->setValue(usage);


}


void MainWindow::set_states()
{
    if(ui->op_type_exp->isChecked() != m_state[EXP])
        ui->op_type_exp->setChecked(m_state[EXP]);
    if(ui->op_type_sim->isChecked() != m_state[SIM])
        ui->op_type_sim->setChecked(m_state[SIM]);

    if(ui->sim_type_bag->isChecked() != m_state[BAG])
        ui->sim_type_bag->setChecked(m_state[BAG]);
    if(ui->sim_type_dummy->isChecked() != m_state[DUMMY])
        ui->sim_type_dummy->setChecked(m_state[DUMMY]);


    if(ui->node_type_apriltag->isChecked() != m_state[APRILTAG])
        ui->node_type_apriltag->setChecked(m_state[APRILTAG]);
    if(ui->node_type_bebop->isChecked() != m_state[BEBOP])
        ui->node_type_bebop->setChecked(m_state[BEBOP]);
    if(ui->node_type_rviz->isChecked() != m_state[RVIZ])
        ui->node_type_rviz->setChecked(m_state[RVIZ]);
    if(ui->node_type_joy->isChecked() != m_state[JOY])
        ui->node_type_joy->setChecked(m_state[JOY]);


    if(ui->filter_type_ekf->isChecked() != m_state[EKF])
        ui->filter_type_ekf->setChecked(m_state[EKF]);
    if(ui->filter_type_lowpass->isChecked() != m_state[LOWPASS])
        ui->filter_type_lowpass->setChecked(m_state[LOWPASS]);

    if(m_state[EXP])
    {
        ui->grp_sim->setDisabled(true);
        m_state[BAG] = m_state[DUMMY] = false;
    }
    else
        ui->grp_sim->setDisabled(false);
}



void MainWindow::on_op_type_sim_toggled(bool checked)
{
   if(checked)
   {
       fill(m_state.begin(), m_state.end(), false);
       for(auto v: m_dfa->defaultSimPath())
       {
           m_state[v] = true;
       }
       set_states();
   }

//   if(m_state[BAG])
//   {
//       QString bagFile = m_settings->value("bag_file").toString();
//        if(!file_exist(bagFile))
//        {
//            qDebug() << "bag file does not exist";
//            bagFile = QFileDialog::getOpenFileName(this, tr("Open File"),
//                                                            "/home",
//                                                            tr("Rosbag (*.bag)"));
//            m_settings->setValue("bag_file", bagFile);

//        }
//        qDebug() << bagFile;
//   }

}

void MainWindow::on_op_type_exp_toggled(bool checked)
{
    if(checked)
    {
        fill(m_state.begin(), m_state.end(), false);
        for(auto v: m_dfa->defaultExpPath())
        {
            m_state[v] = true;
        }

        set_states();
    }
}


