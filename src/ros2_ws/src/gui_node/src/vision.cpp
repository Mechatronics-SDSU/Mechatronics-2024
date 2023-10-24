#include "vision.hpp"
#include "ui_vision.h"
#include <mainwindow.hpp>
#include <QPixmap>
#include <QTimer>
#include <QStringList>
#include <QDir>

Vision::Vision(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Vision)
{
    ui->setupUi(this);
    QPixmap pix("/home/mechatronics/Downloads/hole.jpg");

    // QDir imageDir("/home/mechatronics/Downloads/");
    // imageFileList = imageDir.entryList(QStringList() << "*.png", QDir::Files);
    ui->picture->setPixmap(pix);
    ui->picture->setScaledContents(true);


    // QTimer* timer = new QTimer(this);
    // timer->setInterval(1000 / 60);  // Set the interval to 1/60th of a second (60 FPS)
    // connect(timer, &QTimer::timeout, this, &Vision::updateImage);
    // timer->start();

}

Vision::~Vision()
{
    delete ui;
}

// Vision::updateImage(){
//         // Load the next image
//         if (currentImageIndex < imageFileList.size()) {
//             QPixmap image;
//             image.load(imageFileList[currentImageIndex]);
//             imageLabel->setPixmap(image);
//             currentImageIndex++;
//         } else {
//             // If we've reached the end, loop back to the first image
//             currentImage
            
//             Index = 0;
//         }
// }
