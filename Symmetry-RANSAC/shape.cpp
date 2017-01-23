#include "shape.h"
#include "Mesh.h"
#include "Face.h"
#include <qmath.h>
#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <qmath.h>
#include <qopengl.h>
#include <QVector>
#include <QVector3D>

using namespace std;

Harris3D::Mesh * Shape::mesh;
std::vector<int> Shape::interestPoints;

Shape::Shape()
    : m_count(0), m_countInterest(0)
{
    m_data.resize((*mesh).getNumFaces() * 3 * 6);

    //Draw faces
    for(int i = 0; i<(*mesh).getNumFaces(); i++){
        render((*mesh).getFaces()[i]);
    }

    m_interest.resize(interestPoints.size() * 6);
    //Draw interest points
    for(unsigned int i = 0; i<interestPoints.size();i++){
        addInterest(mesh->getVertices()[interestPoints[i]].v);
    }
}

void Shape::addInterest(double * vertex)
{
    GLfloat *p = m_interest.data() + m_countInterest;
    *p++ = (vertex[0] / (*mesh).getDiagonal());
    *p++ = (vertex[1] / (*mesh).getDiagonal());
    *p++ = (vertex[2]/ (*mesh).getDiagonal());
    *p++ = 1.0f;
    *p++ = 1.0f;
    *p++ = 1.0f;
    m_countInterest += 6;
}

void Shape::add(double * vertex, double * normal)
{
    GLfloat *p = m_data.data() + m_count;
    *p++ = (vertex[0] / (*mesh).getDiagonal());
    *p++ = (vertex[1] / (*mesh).getDiagonal());
    *p++ = (vertex[2] / (*mesh).getDiagonal());
    *p++ = normal[0];
    *p++ = normal[1];
    *p++ = normal[2];
    m_count += 6;
}

void Shape::render(Harris3D::Face f)
{
    //Add vertices
    add((*mesh).getVertices()[f.getVertex(0)].v,f.normal);
    add((*mesh).getVertices()[f.getVertex(1)].v,f.normal);
    add((*mesh).getVertices()[f.getVertex(2)].v,f.normal);

}
