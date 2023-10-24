#ifndef VISION_HPP
#define VISION_HPP

#include <QWidget>

namespace Ui {
class Vision;
}

class Vision : public QWidget
{
    Q_OBJECT

public:
    explicit Vision(QWidget *parent = nullptr);
    ~Vision();

private:
    Ui::Vision *ui;

// private slots:
//     void updateImage();
};

#endif // VISION_HPP
