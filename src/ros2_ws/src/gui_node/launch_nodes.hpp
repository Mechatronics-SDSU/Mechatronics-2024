#ifndef LAUNCH_NODES_HPP
#define LAUNCH_NODES_HPP

#include <QWidget>

namespace Ui {
class launch_nodes;
}

class launch_nodes : public QWidget
{
    Q_OBJECT

public:
    explicit launch_nodes(QWidget *parent = nullptr);
    ~launch_nodes();

private:
    Ui::launch_nodes *ui;
};

#endif // LAUNCH_NODES_HPP
