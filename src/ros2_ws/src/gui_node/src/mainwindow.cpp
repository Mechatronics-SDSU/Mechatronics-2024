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





MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->stackedWidget->insertWidget(1, &_pid_controller);
    ui->stackedWidget->insertWidget(2, &_mission_planner);
    ui->nodes_to_enable->setReadOnly(true);
    // this->json_string["nodes_to_enable"] = this->jsonArray;

    QString styleSheet = "color: #607cff; background-color: #242526;";
    ui->nodes_to_enable->setStyleSheet(styleSheet);
    ui->enabled_launch_nodes->setStyleSheet(styleSheet);
//    connect(&_pid_controller, SIGNAL(HomeClicked()), this, SLOT(moveHome()));
//    connect(&_mission_planner, SIGNAL(HomeClicked()), this, SLOT(moveHome()));

    this->json_gui_node = rclcpp::Node::make_shared("json_gui_node");
    this->json_string_publisher = json_gui_node->create_publisher<scion_types::msg::JsonString>("gui_data", 10);


    this->json_launch_node = rclcpp::Node::make_shared("json_launch_node");
    this->json_launch_node_publisher = json_launch_node->create_publisher<scion_types::msg::JsonString>("launch_node_data", 10);

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

// void executeTerminalCommand(const char* command) {
//     int result = std::system(command);
    
//     if (result == 0) {
//         std::cout << "Command executed successfully." << std::endl;
//     } else {
//         std::cerr << "Command execution failed." << std::endl;
//     }
// }

// void MainWindow::on_visionButton_clicked()
// {
//     executeTerminalCommand("python3 ~/master/scripts/vision/make_dataset.py");
// }


void MainWindow::on_brain_toggled(bool checked)
{
    if (checked){
        this->jsonArray.push_back(ui->brain->text().toStdString());
    } else {
        this->jsonArray.erase(std::remove(this->jsonArray.begin(), this->jsonArray.end(), 
                              ui->brain->text().toStdString()), this->jsonArray.end());
    }
    print_nodes_list();
    // std::cout << "List of nodes to initiate after modifying the array: \n" << this->json_string.dump(4) << std::endl;
}

void MainWindow::print_nodes_list(){
    this->json_string["nodes_to_enable"] = this->jsonArray;
    // QString styleSheet = "color: #607cff; background-color: #242526;";
    // ui->nodes_to_enable->setStyleSheet(styleSheet);
    ui->nodes_to_enable->setReadOnly(false); // Allow modifications
    ui->nodes_to_enable->setPlainText(QString::fromStdString(this->json_string.dump(4)));
    ui->nodes_to_enable->setReadOnly(true); // Restore read-only mode
}

void MainWindow::on_mediator_toggled(bool checked)
{
    if (checked){
        this->jsonArray.push_back(ui->mediator->text().toStdString());
    } else {
        this->jsonArray.erase(std::remove(this->jsonArray.begin(), this->jsonArray.end(), 
                              ui->mediator->text().toStdString()), this->jsonArray.end());
    }
    print_nodes_list();
}

void MainWindow::on_pid_toggled(bool checked)
{
    if (checked){
        this->jsonArray.push_back(ui->pid->text().toStdString());
    } else {
        this->jsonArray.erase(std::remove(this->jsonArray.begin(), this->jsonArray.end(), 
                              ui->pid->text().toStdString()), this->jsonArray.end());
    }
    print_nodes_list();
}

void MainWindow::on_start_nodes_clicked()
{
    //publish json string for main nodes
    auto message = scion_types::msg::JsonString();
    message.data = this->json_string.dump(4);
    this->json_string_publisher->publish(message);


    //publish json string for launh nodes
    auto msg = scion_types::msg::JsonString();
    msg.data = this->launch_nodes_string.dump(4);
    this->json_launch_node_publisher->publish(msg);


}

void MainWindow::on_new_launch_file_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this, "Open Python File",
                       "/home/mechatronics/gui-halie/src/ros2_ws/src", "Python Files (*.py)");
    
    // QFileInfo info(filename);
    // info.open( QIODevice::WriteOnly );
    // QTextStream stream(&info);
    // stream << "Hello, new file!";
    // info.close();

    if (!filename.isEmpty()) {
        QFile file(filename);
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
        // set_current_file(justFileName);
        ui->fileSelected->setText(justFileName);
        update_nodes_list(info.path());
    }


}

void MainWindow::on_select_file_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, "Open Python File",
                       "/home/mechatronics/gui-halie/src/ros2_ws/src", "Python Files (*.py)");

    QFileInfo info(filename); // Get the file info
    // set_current_file(justFileName);
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
    // ui->enabled_launch_nodes->append(ui->launch_nodes->currentText());

    auto it = std::find(this->jsonLaunchArray.begin(), this->jsonLaunchArray.end(), ui->launch_nodes->currentText().toStdString());

    bool itemAlreadyExists = (it != this->jsonLaunchArray.end());

    if (itemAlreadyExists) {// The item already exists in the JSON array
        // Perform your desired action
        this->jsonLaunchArray.erase(std::remove(this->jsonLaunchArray.begin(), this->jsonLaunchArray.end(), 
        ui->launch_nodes->currentText().toStdString()), this->jsonLaunchArray.end());

    } else {
        // The item doesn't exist in the JSON array
        // Perform another action
        this->jsonLaunchArray.push_back(ui->launch_nodes->currentText().toStdString());
    }



    print_launch_nodes_list();
}


void MainWindow::print_launch_nodes_list(){
    this->launch_nodes_string["launch_nodes_to_enable"] = this->jsonLaunchArray;
    ui->enabled_launch_nodes->setReadOnly(false); // Allow modifications
    ui->enabled_launch_nodes->setPlainText(QString::fromStdString(this->launch_nodes_string.dump(4)));
    ui->enabled_launch_nodes->setReadOnly(true); // Restore read-only mode
}


// void MainWindow::print_nodes_list(){
//     this->json_string["nodes_to_enable"] = this->jsonArray;
//     // QString styleSheet = "color: #607cff; background-color: #242526;";
//     // ui->nodes_to_enable->setStyleSheet(styleSheet);
//     ui->nodes_to_enable->setReadOnly(false); // Allow modifications
//     ui->nodes_to_enable->setPlainText(QString::fromStdString(this->json_string.dump(4)));
//     ui->nodes_to_enable->setReadOnly(true); // Restore read-only mode
// }


// void print_nodes_list(std::string key, json json_string, json jsonArray, QTextEdit output)


