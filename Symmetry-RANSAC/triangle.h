#ifndef TRIANGLE
#define TRIANGLE

#include <QObject>
#include <qopengl.h>
#include <QVector>
#include <QVector3D>

class Triangle
{
public:
    QVector<QVector3D> vertices;
    QVector3D normal;
signals:

public slots:

};

#endif // TRIANGLE

