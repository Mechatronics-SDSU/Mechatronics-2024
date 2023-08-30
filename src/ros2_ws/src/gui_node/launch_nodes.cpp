#include "launch_nodes.hpp"
#include "ui_launch_nodes.h"

launch_nodes::launch_nodes(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::launch_nodes)
{
    ui->setupUi(this);
}

launch_nodes::~launch_nodes()
{
    delete ui;
}
