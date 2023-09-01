#include "launch_edit.hpp"
#include "ui_launch_edit.h"
#include <iostream>
#include <fstream>
#include <QDialog>
#include <string>

LaunchEdit::LaunchEdit(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LaunchEdit)
{
    ui->setupUi(this);

    QString styleSheet = "color: #607cff; background-color: #242526;";
    ui->launch_description->setStyleSheet(styleSheet);

}

LaunchEdit::~LaunchEdit()
{
    delete ui;
}

void LaunchEdit::printLaunchFile(const std::string& content){
    ui->launch_description->setReadOnly(false);
    ui->launch_description->setPlainText(QString::fromStdString(content));
    ui->launch_description->append(QString::fromStdString(getLaunchParamsVecAsString()));
}


void LaunchEdit::on_saveLaunchDescription_clicked()
{
    std::ofstream pythonFile;
    pythonFile.open(pythonFilePath, std::ios::trunc);

    if(!pythonFile.is_open() || !pythonFile.good()){
        std::cout<< "Failed to open python file." << std::endl;
    }

    std::string launchFileString = ui->launch_description->toPlainText().toStdString();

    pythonFile << launchFileString << std::endl;

}

void LaunchEdit::on_saveClose_clicked()
{
    on_saveLaunchDescription_clicked();
    this->close();

}

std::string LaunchEdit::addNodeString(const LaunchParameters& node)
{
    std::string nodeString;
    nodeString = "        launch_ros.actions.Node(\n";
    nodeString += "            package='" + node.pkgName + "', ";
    nodeString += "executable='" + node.execName + "', ";
    nodeString += "output='" + node.output + "',\n";

    if(!node.params.empty()){
        nodeString += "            parameters= [\n";
        for (const auto& pair : node.params){
            nodeString += "                            {";
            nodeString += "\"" + pair.first + "\": ";
            if (std::holds_alternative<std::string>(pair.second)) {
                nodeString += "\"" + std::get<std::string>(pair.second) + "\""; // Handle string type
            } else if (std::holds_alternative<bool>(pair.second)) {
                nodeString += std::get<bool>(pair.second) ? "True" : "False"; // Handle bool type
            } else if (std::holds_alternative<int>(pair.second)) {
                nodeString += std::to_string(std::get<int>(pair.second)); // Handle int type
            } else {
                nodeString += "Unsupported type"; // Handle unsupported types
            }
            nodeString += "},\n";

        }
        nodeString += "                        ]\n";
    }

    nodeString += "            ),\n";

    return nodeString;

}



std::string LaunchEdit::getLaunchParamsVecAsString() {

    std::string nodeList = "";

    for (const auto& params : launchParamsVec) {
        nodeList += addNodeString(params);
    }
        
    nodeList += "    ])";

    return nodeList;
}