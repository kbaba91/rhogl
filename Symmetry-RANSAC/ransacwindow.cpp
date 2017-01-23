#include "gloffwidget.h"
#include "glransacwidget.h"
#include "ransacwindow.h"
#include "mainwindow.h"
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>

RansacWindow::RansacWindow(MainWindow *mw, GLOffWidget *oriWidget, GLRANSACWidget *newWidget)
    : mainWindow(mw)
{
    //Original object
    glOriWidget = oriWidget;

    xOriSlider = createSlider();
    yOriSlider = createSlider();
    zOriSlider = createSlider();

    connect(xOriSlider, SIGNAL(valueChanged(int)), glOriWidget, SLOT(setXRotation(int)));
    connect(glOriWidget, SIGNAL(xRotationChanged(int)), xOriSlider, SLOT(setValue(int)));
    connect(yOriSlider, SIGNAL(valueChanged(int)), glOriWidget, SLOT(setYRotation(int)));
    connect(glOriWidget, SIGNAL(yRotationChanged(int)), yOriSlider, SLOT(setValue(int)));
    connect(zOriSlider, SIGNAL(valueChanged(int)), glOriWidget, SLOT(setZRotation(int)));
    connect(glOriWidget, SIGNAL(zRotationChanged(int)), zOriSlider, SLOT(setValue(int)));

    //Reconstructed object
    glWidget = newWidget;

    xSlider = createSlider();
    ySlider = createSlider();
    zSlider = createSlider();

    connect(xSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setXRotation(int)));
    connect(glWidget, SIGNAL(xRotationChanged(int)), xSlider, SLOT(setValue(int)));
    connect(ySlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setYRotation(int)));
    connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider, SLOT(setValue(int)));
    connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
    connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));

    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *container = new QHBoxLayout;

    container->addWidget(xOriSlider);
    container->addWidget(yOriSlider);
    container->addWidget(zOriSlider);
    container->addWidget(glOriWidget);
    container->addWidget(glWidget);
    container->addWidget(xSlider);
    container->addWidget(ySlider);
    container->addWidget(zSlider);

    QWidget *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);
    dockBtn = new QPushButton(tr("Undock"), this);
    connect(dockBtn, SIGNAL(clicked()), this, SLOT(dockUndock()));
    mainLayout->addWidget(dockBtn);

    setLayout(mainLayout);

    xOriSlider->setValue(15 * 16);
    yOriSlider->setValue(345 * 16);
    zOriSlider->setValue(0 * 16);
    xSlider->setValue(15 * 16);
    ySlider->setValue(345 * 16);
    zSlider->setValue(0 * 16);

    setWindowTitle(tr("Symmetry-RANSAC"));
}

QSlider *RansacWindow::createSlider()
{
    QSlider *slider = new QSlider(Qt::Vertical);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

void RansacWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void RansacWindow::dockUndock()
{
    if (parent()) {
        setParent(0);
        setAttribute(Qt::WA_DeleteOnClose);
        move(QApplication::desktop()->width() / 2 - width() / 2,
             QApplication::desktop()->height() / 2 - height() / 2);
        dockBtn->setText(tr("Dock"));
        show();
    } else {
        if (!mainWindow->centralWidget()) {
            if (mainWindow->isVisible()) {
                setAttribute(Qt::WA_DeleteOnClose, false);
                dockBtn->setText(tr("Undock"));
                mainWindow->setCentralWidget(this);
            } else {
                QMessageBox::information(0, tr("Cannot dock"), tr("Main window already closed"));
            }
        } else {
            QMessageBox::information(0, tr("Cannot dock"), tr("Main window already occupied"));
        }
    }
}




