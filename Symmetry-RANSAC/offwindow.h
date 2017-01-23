#ifndef OFFWINDOW
#define OFFWINDOW

#include <QWidget>

class QSlider;
class QPushButton;

class GLOffWidget;
class MainWindow;

class OffWindow : public QWidget
{
    Q_OBJECT

public:
    OffWindow(MainWindow *mw);

protected:
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

private Q_SLOTS:
    void dockUndock();

private:
    QSlider *createSlider();

    GLOffWidget *glWidget;
    QSlider *xSlider;
    QSlider *ySlider;
    QSlider *zSlider;
    QPushButton *dockBtn;
    MainWindow *mainWindow;
};

#endif // OFFWINDOW

