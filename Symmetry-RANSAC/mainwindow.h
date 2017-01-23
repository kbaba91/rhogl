#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "Vertex.h"
#include "Mesh.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow();

private Q_SLOTS:
    void onAddNew();
    void onOpen();
    void onCompletion();
    //void onCGAL();
    void onHoleFilling();
};

#endif
