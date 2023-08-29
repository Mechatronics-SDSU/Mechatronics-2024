#include "pid_controller.hpp"
#include "ui_pid_controller.h"
#include <QComboBox>
#include <QtCore/QString>
#include <iostream>
#include <QTextEdit>
#include <QProcess>
#include <QWidget>
#include <memory> 
#include <QKeyEvent>
#include <QPushButton>
#include <QVBoxLayout>
#include <QDoubleValidator>

PIDController::PIDController(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PIDController)
{
    this->node = rclcpp::Node::make_shared("gui_node");
    this->kp_publisher = node->create_publisher<scion_types::msg::PidTuning>("kp_data", 10);
    this->ki_publisher = node->create_publisher<scion_types::msg::PidTuning>("ki_data", 10);
    this->kd_publisher = node->create_publisher<scion_types::msg::PidTuning>("kd_data", 10);

    ui->setupUi(this);

    connect(ui->kp_val,  &QLineEdit::returnPressed, this, &PIDController::on_KpValue_editingFinished);
    connect(ui->ki_val,  &QLineEdit::returnPressed, this, &PIDController::on_KiValue_editingFinished);
    connect(ui->kd_val,  &QLineEdit::returnPressed, this, &PIDController::on_KdValue_editingFinished);

    connect(ui->kp_verticalSlider,  SIGNAL(valueChanged(int)), ui->kp_progressBar, SLOT(setValue(int)));
    connect(ui->ki_verticalSlider,  SIGNAL(valueChanged(int)), ui->ki_progressBar, SLOT(setValue(int)));
    connect(ui->kd_verticalSlider,  SIGNAL(valueChanged(int)), ui->kd_progressBar, SLOT(setValue(int)));

    // Create a QDoubleValidator
    QDoubleValidator *pidValRange = new QDoubleValidator(this);
    pidValRange->setBottom(0.0);  // Set the lower bound
    pidValRange->setTop(1.0);   // Set the upper bound

    // Set the validator on the QLineEdit
    ui->kp_val->setValidator(pidValRange);
    ui->ki_val->setValidator(pidValRange);
    ui->kd_val->setValidator(pidValRange);
    

}

PIDController::~PIDController()
{
    delete ui;
}

//void PIDController::on_home_button_clicked()
//{
//    emit HomeClicked();
//}

void PIDController::on_KpValue_editingFinished()
{
    sendPIDVals(this->kpVal, ui->kp_val, ui->kpOutput, kp_publisher);
}

void PIDController::on_KiValue_editingFinished()
{
    sendPIDVals(this->kiVal, ui->ki_val, ui->kiOutput, ki_publisher);
}

void PIDController::on_KdValue_editingFinished()
{
    sendPIDVals(this->kdVal, ui->kd_val, ui->kdOutput, kd_publisher);
}

//publishing pid data to ros topics
void PIDController::sendPIDVals(float pidVal, QLineEdit* lineEdit, QTextEdit* output, 
                                rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr publisher){

    pidVal = lineEdit->text().toFloat();

    auto message = scion_types::msg::PidTuning();
    message.data = pidVal;
    message.axis = this->axis;
    publisher->publish(message);

    lineEdit->setPlaceholderText(lineEdit->text());
    lineEdit->clear();

    executeCommands(output, message);   

}

/*update pid axis to index changed in action combobox:
 (yaw = 0,
  pitch = 1, 
  roll = 2, 
  x_pos = 3, 
  y_pos = 4, 
  z_pos = 5)
*/
void PIDController::on_Tuning_Axis_currentIndexChanged(int index)
{
    this->axis = index; 
}

//print tuned pid data into pid text boxes 
 void PIDController::executeCommands(QTextEdit* textBoxObject, const scion_types::msg::PidTuning& output) 
    {
        QString textToDisplay = "Data: " + QString::number(output.data) +
                                ", Axis: " + QString::number(output.axis);

        textBoxObject->setReadOnly(false); // Allow modifications
        textBoxObject->append(textToDisplay);
        textBoxObject->setReadOnly(true); // Restore read-only mode
    }

//clear kp val content from kp text box
void PIDController::on_kpClearButton_clicked()
{
    ui->kpOutput->clear();
}

//clear ki val content from ki text box
void PIDController::on_kiClearButton_clicked()
{
    ui->kiOutput->clear();
}

//clear kd val content from kd text box
void PIDController::on_kdClearButton_clicked()
{
    ui->kdOutput->clear();
}

//clear all contents from three text boxes that display tuned pid values
void PIDController::on_clearAllButton_clicked()
{
    on_kpClearButton_clicked();
    on_kiClearButton_clicked();
    on_kdClearButton_clicked();
}

//display ros topic list
void PIDController::on_topicListButton_clicked()
{
    QProcess process;
    process.setWorkingDirectory("/home/mechatronics/gui-halie/src/ros2_ws"); // Set the desired working directory
    QStringList arguments;
    arguments << "-c" << "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 topic list";

    process.start("bash", arguments);

    process.waitForFinished();

    QByteArray output = process.readAllStandardOutput();
    ui->topicList->setStyleSheet("color: white;");
    ui->topicList->setText(output);
}

//detecting up/down key pressed --> call navigate function
void PIDController::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
    case Qt::Key_Down:
        navigateFocus(static_cast<Qt::Key>(event->key()));
        break;

    }
}

//Allows navigation between line edits for tuning pid using up/down arrow keys
void PIDController::navigateFocus(Qt::Key key)
{
    if (ui->kp_val->hasFocus()) {
        if (key == Qt::Key_Up) {
            ui->kd_val->setFocus();
        } else if (key == Qt::Key_Down) {
            ui->ki_val->setFocus();
        }
    } else if (ui->ki_val->hasFocus()) {
        if (key == Qt::Key_Up) {
            ui->kp_val->setFocus();
        } else if (key == Qt::Key_Down) {
            ui->kd_val->setFocus();
        }
    } else if (ui->kd_val->hasFocus()) {
        if (key == Qt::Key_Up) {
            ui->ki_val->setFocus();
        } else if (key == Qt::Key_Down) {
            ui->kp_val->setFocus();
        }
    }
}
