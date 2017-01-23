#ifndef RANSACWINDOW
#define RANSACWINDOW

#include <QWidget>

class QSlider;
class QPushButton;

class GLOffWidget;
class GLRANSACWidget;
class MainWindow;

class RansacWindow : public QWidget
{
    Q_OBJECT

public:
    RansacWindow(MainWindow *mw, GLOffWidget *oriWidget, GLRANSACWidget *newWidget);

protected:
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

private Q_SLOTS:
    void dockUndock();

private:
    QSlider *createSlider();

    GLOffWidget *glOriWidget;
    QSlider *xOriSlider;
    QSlider *yOriSlider;
    QSlider *zOriSlider;
    GLRANSACWidget *glWidget;
    QSlider *xSlider;
    QSlider *ySlider;
    QSlider *zSlider;
    QPushButton *dockBtn;
    MainWindow *mainWindow;
};

#endif // RANSACWINDOW

