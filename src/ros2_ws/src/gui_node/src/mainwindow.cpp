#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include <iostream>
#include <filesystem>
#include <vector>
#include <nlohmann/json.hpp>
#include <QString>
#include "scion_types/msg/json_string.hpp"
#include <QFileDialog>
#include <QTextStream>
#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QTextEdit>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->stackedWidget->insertWidget(1, &_pid_controller);
    ui->stackedWidget->insertWidget(2, &_mission_planner);
    ui->nodes_to_enable->setReadOnly(true);

    QString styleSheet = "color: #607cff; background-color: #242526;";
    ui->nodes_to_enable->setStyleSheet(styleSheet);
    ui->enabled_launch_nodes->setStyleSheet(styleSheet);
//    connect(&_pid_controller, SIGNAL(HomeClicked()), this, SLOT(moveHome()));
//    connect(&_mission_planner, SIGNAL(HomeClicked()), this, SLOT(moveHome()));

    this->json_string = rclcpp::Node::make_shared("json_gui_nodes");
    this->json_string_publisher = json_string->create_publisher<scion_types::msg::JsonString>("gui_data", 10);

    connect(ui->launch_nodes,QOverload<int>::of(&QComboBox::activated), this, &MainWindow::launch_nodes_selected);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pid_controller_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);

}

void MainWindow::on_mission_planning_clicked()
{
        ui->stackedWidget->setCurrentIndex(2);
}

//void MainWindow::moveHome()
//{
//        ui->stackedWidget->setCurrentIndex(0);
//}

void MainWindow::on_homeButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void MainWindow::on_brain_toggled(bool checked)
{
    update_json_nodes(checked, ui->brain);
}
void MainWindow::on_mediator_toggled(bool checked)
{
    update_json_nodes(checked, ui->mediator);
}

void MainWindow::on_pid_toggled(bool checked)
{
    update_json_nodes(checked, ui->pid);
}

void MainWindow::update_json_nodes(bool checked, QCheckBox* node){

    if (checked){
        this->jsonMainArray.push_back(node->text().toStdString());
    } else {
        this->jsonMainArray.erase(std::remove(this->jsonMainArray.begin(), this->jsonMainArray.end(), 
                              node->text().toStdString()), this->jsonMainArray.end());
    }
    print_nodes_list("nodes_to_enable", this->main_nodes_string, this->jsonMainArray, ui->nodes_to_enable); 

}

void MainWindow::on_start_nodes_clicked()
{
    //publish json string for main nodes
    auto message = scion_types::msg::JsonString();
    message.main_nodes = this->main_nodes_string.dump(4);
    message.launch_nodes = this->launch_nodes_string.dump(4);
    this->json_string_publisher->publish(message);

}

void MainWindow::on_new_launch_file_clicked()
{
    launchFileName = QFileDialog::getSaveFileName(this, "Open Python File",
                       "/home/mechatronics/gui-halie/src/ros2_ws/src", "Python Files (*.py)");
    
    if (!launchFileName.isEmpty()) {
        QFile file(launchFileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream stream(&file);
            stream << "from launch import LaunchDescription\n"
                    "import launch_ros.actions\n"
                    "from launch_ros.actions import Node\n"
                    "import subprocess\n\n"
                    "def generate_launch_description():\n"
                    "   return LaunchDescription([])\n";
            file.close();
        }
        QFileInfo info(file.fileName()); // Get the file info
        QString justFileName = info.fileName(); // Extract the file
        ui->fileSelected->setText(justFileName);
        update_nodes_list(info.path());
    } else {
        return;
    }


}

void MainWindow::on_select_file_clicked()
{
    launchFileName = QFileDialog::getOpenFileName(this, "Open Python File",
                       "/home/mechatronics/gui-halie/src/ros2_ws/src", "Python Files (*.py)");
     if (launchFileName.isEmpty()){
        return;
     }
    QFileInfo info(launchFileName); // Get the file info
    ui->fileSelected->setText(info.fileName());
    update_nodes_list(info.path());

}

namespace fs = std::filesystem;

std::vector<std::string> MainWindow::getRosPackageNames(const std::string& directoryPath) {
    std::vector<std::string> packageNames;

    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        if (fs::is_directory(entry) && fs::exists(entry.path() / "package.xml")) {
            packageNames.push_back(entry.path().filename());
        }
    }

    return packageNames;
}

void MainWindow::update_nodes_list(QString path){
    std::vector<std::string> packageNames = getRosPackageNames(path.toStdString());
    ui->launch_nodes->clear();
    for (const std::string& packageName : packageNames){
        ui->launch_nodes->addItem(QString::fromStdString(packageName));
    }

}

void MainWindow::launch_nodes_selected(){

    auto it = std::find(this->jsonLaunchArray.begin(), this->jsonLaunchArray.end(), ui->launch_nodes->currentText().toStdString());

    bool itemAlreadyExists = (it != this->jsonLaunchArray.end());

    if (itemAlreadyExists) {// The item already exists in the JSON array
        this->jsonLaunchArray.erase(std::remove(this->jsonLaunchArray.begin(), this->jsonLaunchArray.end(), 
        ui->launch_nodes->currentText().toStdString()), this->jsonLaunchArray.end());

    } else {// The item doesn't exist in the JSON array
        this->jsonLaunchArray.push_back(ui->launch_nodes->currentText().toStdString());
    }

    // print_launch_nodes_list();
    print_nodes_list("launch_nodes_to_enable", this->launch_nodes_string,
                      this->jsonLaunchArray, ui->enabled_launch_nodes);
}


void MainWindow::print_nodes_list(const std::string& key, nlohmann::json& json_string, 
                                  const nlohmann::json& jsonArray, QTextEdit* output) {
    json_string[key] = jsonArray;
    output->setReadOnly(false); // Allow modifications
    output->setPlainText(QString::fromStdString(json_string.dump(4)));
    output->setReadOnly(true); // Restore read-only mode
}

void MainWindow::on_launchNodesEdit_clicked(){

    LaunchEdit *launchEdit = new LaunchEdit; // Create on the heap
    launchEdit->pythonFilePath = this->launchFileName.toStdString();
    launchEdit->printLaunchFile(launchEdit->launchDescription);
    launchEdit->show();

}



