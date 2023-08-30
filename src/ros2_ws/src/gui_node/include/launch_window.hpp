#ifndef LAUNCHWINDOW_HPP
#define LAUNCHWINDOW_HPP

#include <QWidget>

// Forward declaration of Ui class
namespace Ui {
    class LaunchWindow;
}

class LaunchWindow : public QWidget
{
    Q_OBJECT

public:
    explicit LaunchWindow(QWidget *parent = nullptr);
    ~LaunchWindow();

private:
    Ui::LaunchWindow *ui; // Pointer to the UI class instance
};

#endif // LAUNCHWINDOW_HPP
