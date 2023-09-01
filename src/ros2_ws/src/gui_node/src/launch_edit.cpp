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
    // connect(ui->death_button,&QPushButton::clicked, this, &LaunchEdit::on_death_button_clicked);

}

LaunchEdit::~LaunchEdit()
{
    delete ui;
}

void LaunchEdit::printLaunchFile(const std::string& content){
    ui->launch_description->setReadOnly(false);
    ui->launch_description->setPlainText(QString::fromStdString(content));
}


void LaunchEdit::on_saveLaunchDescription_clicked()
{
    std::ofstream pythonFile;
    pythonFile.open(pythonFilePath, std::ios::trunc);

    if(!pythonFile.is_open() || !pythonFile.good()){
        std::cout<< "Failed to open python file." << std::endl;
    }

    pythonFile << this->launchDescription << std::endl;

}

void LaunchEdit::on_saveClose_clicked()
{
    on_saveLaunchDescription_clicked();
    this->close();

}

// void LaunchEdit::addNodeString(const std::string& pkgName, const std::string& execName,
//                                const std::vector<std::map<std::string, std::string>>& params = {})
// {
//     nodeList += "    package='" + pkgName + "', ";
//     nodeList += "executable='" + execName + "', ";
//     nodeList += "output='screen'\n";

//     if(!params.empty()){
//         nodeList += "    parameters=[";
//         for (const auto& param : params){
//             vbasduikfbnisu ten toi la rose do
//             toi la ky su
//         }
//         nodeList += "]";
//     }

//     nodeList += "    ),\n";

// }
