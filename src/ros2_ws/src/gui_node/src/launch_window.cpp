#include "launch_window.hpp" // Include the header file
#include "ui_launch_window.h" // Include the auto-generated UI header

LaunchWindow::LaunchWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LaunchWindow) // Initialize the UI instance
{
    ui->setupUi(this); // Set up the UI
}

LaunchWindow::~LaunchWindow()
{
    delete ui; // Delete the UI instance to free memory
}
