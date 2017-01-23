#ifndef SHAPE
#define SHAPE

#include "Mesh.h"
#include "Face.h"
#include <qopengl.h>
#include <QVector>
#include <QVector3D>

class Shape
{
public:
    Shape();
    static Harris3D::Mesh * mesh;
    static std::vector<int> interestPoints;
    const GLfloat *constData() const { return m_data.constData(); }
    const GLfloat *constInterest() const { return m_interest.constData(); }
    int count() const { return m_count; }
    int interestCount() const { return m_countInterest / 6; }
    int vertexCount() const { return m_count / 6; }

private:
    void render(Harris3D::Face f);
    void add(double * vertex, double * normal);
    void addInterest(double * vertex);

    QVector<GLfloat> m_data;
    QVector<GLfloat> m_interest;
    int m_count;
    int m_countInterest;
};

#endif // SHAPE

