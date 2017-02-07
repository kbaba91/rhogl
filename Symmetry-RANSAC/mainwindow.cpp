#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QDirIterator>
#include <qmath.h>
#include <qopengl.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <cmath>
#include <vector>
#include "mainwindow.h"
#include "window.h"
#include "offwindow.h"
#include "gloffwidget.h"
#include "glransacwidget.h"
#include "ransacwindow.h"
#include "shape.h"
#include "rotation.h"
#include "Mesh.h"
#include "Clock.h"
#include "util.h"
#include "Properties.h"
#include "HarrisDetector.h"
#include "optdialog.h"
#include "core.h"
#include "simplify.h"
#include "obj.h"
#include "omp.h"

#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/IterativeLinearSolvers>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/filters/extract_indices.h>

#include <openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/Types.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/property_map.h>

using namespace std;
using namespace openvdb;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

MainWindow::MainWindow()
{
    QMenuBar *menuBar = new QMenuBar;
    QMenu *menuWindow = menuBar->addMenu(tr("&View"));
    QAction *open = new QAction(menuWindow);
    open->setText(tr("View file"));
    menuWindow->addAction(open);
    connect(open, SIGNAL(triggered()), this, SLOT(onOpen()));
    QMenu *menuRansac = menuBar->addMenu(tr("&Completion"));
    QAction *completion = new QAction(menuRansac);
    completion->setText(tr("Complete mesh"));
    menuRansac->addAction(completion);
    connect(completion, SIGNAL(triggered()), this, SLOT(onCompletion()));
    setMenuBar(menuBar);

    onScript();
}

double calculateEdge(Harris3D::Vertex * i, Harris3D::Vertex * j){
    return std::sqrt(pow(i->getX()-j->getX(),2)+pow(i->getY()-j->getY(),2)+pow(i->getZ()-j->getZ(),2));
}

double calculateAreaTriangle(Harris3D::Vertex * i, Harris3D::Vertex * j, Harris3D::Vertex * k){
    //Heron formula
    double a = calculateEdge(i,j);
    double b = calculateEdge(i,k);
    double c = calculateEdge(j,k);
    double s = (a+b+c)/2;

    return std::sqrt(s*(s-a)*(s-b)*(s-c));
}

double calculateNormalsDotProduct(Harris3D::Vertex * i, Harris3D::Vertex * j){
    return (i->normal[0]*j->normal[0]+i->normal[1]*j->normal[1]+i->normal[2]*j->normal[2]);
}

Harris3D::Vertex calculateBarycenter(Harris3D::Mesh * mesh, int i, int j, int k){
    //find barycenter
    Harris3D::Vertex barycenter;
    barycenter.setX((mesh->getVertices()[i].getX() + mesh->getVertices()[j].getX() + mesh->getVertices()[k].getX())/3);
    barycenter.setY((mesh->getVertices()[i].getY() + mesh->getVertices()[j].getY() + mesh->getVertices()[k].getY())/3);
    barycenter.setZ((mesh->getVertices()[i].getZ() + mesh->getVertices()[j].getZ() + mesh->getVertices()[k].getZ())/3);

    return barycenter;
}

double calculateVoronoiTriangle(Harris3D::Mesh * mesh, int i, int j){
    int neighbor1 = -1;
    int neighbor2 = -1;
    Harris3D::Vertex vertex1 = mesh->getVertices()[i];
    Harris3D::Vertex vertex2 = mesh->getVertices()[j];
    std::set<int>::iterator it1;
    std::set<int>::iterator it2;

    //find if they are neighbors
    for (it1 = vertex1.getAdjacentVertices().begin(); it1 != vertex1.getAdjacentVertices().end(); ++it1){
        for (it2 = vertex2.getAdjacentVertices().begin(); it2 != vertex2.getAdjacentVertices().end(); ++it2){
            if(*it1==*it2){
                if (neighbor1 == -1){
                    neighbor1 = *it1;
                }
                else{
                    neighbor2 = *it1;
                    break;
                }
            }
        }
    }

    if (neighbor1!=-1 && neighbor2!=-1){
        Harris3D::Vertex barycenter1 = calculateBarycenter(mesh, vertex1.getIndex(), vertex2.getIndex(), neighbor1);
        Harris3D::Vertex barycenter2 = calculateBarycenter(mesh, vertex1.getIndex(), vertex2.getIndex(), neighbor2);

        return calculateAreaTriangle(&vertex1, &barycenter1, &barycenter2);
    }
    else{
        return 0.0;
    }
}

double calculateVoronoiCell(Harris3D::Mesh * mesh, int i){
    double area = 0.0;
    std::set<int>::iterator it;
    Harris3D::Vertex vertex = mesh->getVertices()[i];

    for (it = vertex.getAdjacentVertices().begin(); it != vertex.getAdjacentVertices().end(); ++it){
        double aux = calculateVoronoiTriangle(mesh, vertex.getIndex(), *it);
        if (aux > 0.0){
            area += aux;
        }
    }

    return area;
}

Harris3D::Vertex calculateClosestCompatiblePoint(Harris3D::Mesh * inputMesh, int index, Harris3D::Mesh * targetMesh){
    Harris3D::Vertex candidate = targetMesh->getVertices()[0];
    Harris3D::Vertex vertex = inputMesh->getVertices()[index];
    double minEdge = calculateEdge(&vertex, &candidate);
    for (int i = 1; i<targetMesh->getNumVertices(); i++){
        double thisEdge = calculateEdge(&vertex, &targetMesh->getVertices()[i]);
        if ((thisEdge < minEdge) && (calculateNormalsDotProduct(&vertex, &targetMesh->getVertices()[i])>0)){
            minEdge = thisEdge;
            candidate = targetMesh->getVertices()[i];
        }
    }
    return candidate;
}

double calculateOmega(Harris3D::Mesh * mesh, Harris3D::Vertex * q, Harris3D::Vertex * v){
    Harris3D::Vertex * r = &mesh->getVertices()[0];
    double minEdge = calculateEdge(q, r);
    double h = 0;
    for (int i = 1; i<mesh->getNumVertices(); i++){
        double thisEdge = calculateEdge(q, &mesh->getVertices()[i]);
        h += thisEdge;
        if (thisEdge < minEdge){
            minEdge = thisEdge;
            r = &mesh->getVertices()[i];
        }
    }

    return std::exp(-pow(calculateEdge(v,r),2));
    h /= (double) mesh->getNumVertices();
    return std::exp(-pow(calculateEdge(v,r),2)/(pow(h,2)));
    //return std::exp(-pow(calculateEdge(v,r),2));
}

Harris3D::Mesh * mergeMeshes(vector<Harris3D::Mesh *>& meshes){
    Harris3D::Mesh * mesh = new Harris3D::Mesh();
    int nf = 0;
    for (unsigned i=0;i<meshes.size();i++){
        nf += meshes.at(i)->getNumFaces();
    }

    //Populate vertices
    mesh->setVertices(meshes.at(0)->getVertices());

    //Populate faces
    int faceIndex = 0;
    for (unsigned i=0;i<meshes.size();i++){
        for (int j=0;j<meshes.at(i)->getNumFaces();j++){
            mesh->insertFace(faceIndex,meshes.at(i)->getFaces()[j].getVertices()[0],meshes.at(i)->getFaces()[j].getVertices()[1],meshes.at(i)->getFaces()[j].getVertices()[2]);
            faceIndex++;
        }
    }

    return mesh;
}

void openVDB(Harris3D::Mesh * mesh){
    std::vector<Vec3s> points;
    std::vector<Vec3I> triangles;

    for (int i = 0; i < mesh->getNumVertices(); i++){
        Vec3s point(mesh->getVertices()[i].getX(),mesh->getVertices()[i].getY(),mesh->getVertices()[i].getZ());
        points.push_back(point);
    }
    for (int i = 0; i < mesh->getNumFaces(); i++){
        Vec3I triangle(mesh->getFaces()[i].getVertex(0),mesh->getFaces()[i].getVertex(1),mesh->getFaces()[i].getVertex(2));
        triangles.push_back(triangle);
    }
    openvdb::math::Transform xform;
    std::vector<Vec3s> newPoints;
    std::vector<Vec3I> newTriangles;
    std::vector<Vec4I> newQuads;
    FloatGrid::Ptr levelset = openvdb::tools::meshToLevelSet<FloatGrid>(xform, points, triangles);
    openvdb::tools::volumeToMesh(*levelset,newPoints,newTriangles,newQuads);

    mesh->cleanMesh();

    for (unsigned i = 0; i < newPoints.size(); i++){
        mesh->insertVertex(i, newPoints[i].x(), newPoints[i].y(), newPoints[i].z());

    }
    for (unsigned i = 0; i < newQuads.size(); i++){
        mesh->insertFace(2*i,newQuads[i].z(),newQuads[i].y(),newQuads[i].x());
        mesh->insertFace(2*i+1,newQuads[i].w(),newQuads[i].z(),newQuads[i].x());
    }
}

void decimate(Harris3D::Mesh * mesh, int numFaces){
    if(mesh->getNumFaces()>numFaces){
        for(int i=0;i<mesh->getNumVertices();i++){
            Simplify::Vertex v;
            v.p.x=mesh->getVertices()[i].x();
            v.p.y=mesh->getVertices()[i].y();
            v.p.z=mesh->getVertices()[i].z();
            Simplify::vertices.push_back(v);
        }

        for(int i=0;i<mesh->getNumFaces();i++){
            Simplify::Triangle t;
            t.v[0] = mesh->getFaces()[i].getVertex(0);
            t.v[1] = mesh->getFaces()[i].getVertex(1);
            t.v[2] = mesh->getFaces()[i].getVertex(2);
            Simplify::triangles.push_back(t);
        }

        cout << "Input: " << Simplify::triangles.size() << " triangles " << Simplify::vertices.size() << " vertices\n" << endl;

        Simplify::simplify_mesh(numFaces);

        cout << "Output: " << Simplify::triangles.size() << " triangles " << Simplify::vertices.size() << " vertices\n" << endl;

        //Reconstruct mesh
        mesh->cleanMesh();

        for (unsigned i = 0; i < Simplify::vertices.size(); i++){
            mesh->insertVertex(i, Simplify::vertices[i].p.x, Simplify::vertices[i].p.y, Simplify::vertices[i].p.z);
        }
        for (unsigned i = 0; i < Simplify::triangles.size(); i++){
            if(!Simplify::triangles[i].deleted){
                mesh->insertFace(i,Simplify::triangles[i].v[0],Simplify::triangles[i].v[1],Simplify::triangles[i].v[2]);
            }
        }
        Simplify::vertices.clear();
        Simplify::triangles.clear();
    }
}

void outputMesh(Harris3D::Mesh * mesh, string filename){
    ofstream outoff(filename);
    outoff << "OFF" << endl;
    outoff << mesh->getNumVertices() << " " << mesh->getNumFaces() << " 0" << endl;
    for (int i = 0; i < mesh->getNumVertices(); i++){
        outoff << mesh->getVertices()[i].getX() << " " << mesh->getVertices()[i].getY() << " " << mesh->getVertices()[i].getZ() << endl;
    }
    for (int i =  0; i < mesh->getNumFaces(); i++){
        outoff << 3 << " " << mesh->getFaces()[i].getVertex(0) << " " << mesh->getFaces()[i].getVertex(1) << " " << mesh->getFaces()[i].getVertex(2) << " " << endl;
    }
    outoff.close();
}

void outputInliers(string filename){
    ofstream out1(filename);
    out1 << Shape::interestPoints.size() << endl;
    for(unsigned i = 0; i < Shape::interestPoints.size() -1; i++){
        out1 << Shape::interestPoints[i] << endl;
    }
    out1.close();
}

void segmentateMesh(Harris3D::Mesh * mesh, vector<Harris3D::Mesh *>& meshes){
    //Create output file
    outputMesh(mesh, "output.off");

    //Mesh segmentation
    Polyhedron p;
    std::ifstream input("output.off");
    if ( !input || !(input >> p) || p.empty() ) {
      std::cerr << "Not a valid off file." << std::endl;
    }
    // create a property-map for SDF values
    typedef std::map<Polyhedron::Facet_const_handle, double> Facet_double_map;
    Facet_double_map internal_sdf_map;
    boost::associative_property_map<Facet_double_map> sdf_property_map(internal_sdf_map);
    // compute SDF values using default parameters for number of rays, and cone angle
    CGAL::sdf_values(p, sdf_property_map);
    // create a property-map for segment-ids
    typedef std::map<Polyhedron::Facet_const_handle, std::size_t> Facet_int_map;
    Facet_int_map internal_segment_map;
    boost::associative_property_map<Facet_int_map> segment_property_map(internal_segment_map);
    // segment the mesh using default parameters for number of levels, and smoothing lambda
    // Any other scalar values can be used instead of using SDF values computed using the CGAL function
    std::size_t number_of_segments = CGAL::segmentation_from_sdf_values(p, sdf_property_map, segment_property_map);
    std::cout << "Number of segments: " << number_of_segments << std::endl;

    meshes.clear();
    vector<int> sizes;

    for (unsigned i = 0; i<number_of_segments; i++){
        Harris3D::Mesh * auxMesh = new Harris3D::Mesh();
        auxMesh->setVertices(mesh->getVertices());
        meshes.push_back(auxMesh);
        sizes.push_back(0);
    }

    // print segment-ids
    for(Polyhedron::Facet_iterator facet_it = p.facets_begin();
        facet_it != p.facets_end(); ++facet_it) {
        // ids are between [0, number_of_segments -1]
        sizes.at(segment_property_map[facet_it])+=1;
    }

    for (unsigned i = 0; i<number_of_segments; i++){
        sizes.at(i)=0;
    }

    for(Polyhedron::Facet_iterator facet_it = p.facets_begin();
        facet_it != p.facets_end(); ++facet_it) {
        // ids are between [0, number_of_segments -1]
        Polyhedron::Halfedge_around_facet_circulator circulator = facet_it->facet_begin();
        int vertex0 = std::distance(p.vertices_begin(),circulator->vertex());
        ++circulator;
        int vertex1 = std::distance(p.vertices_begin(),circulator->vertex());
        ++circulator;
        int vertex2 = std::distance(p.vertices_begin(),circulator->vertex());
        meshes.at(segment_property_map[facet_it])->insertFace(sizes.at(segment_property_map[facet_it]),vertex0,vertex1,vertex2);
        sizes.at(segment_property_map[facet_it])+=1;
    }

    //Find main mesh
    int position = 0;
    for(unsigned i = 0; i< meshes.size();i++){
        if (meshes.at(i)->getNumFaces() > meshes.at(position)->getNumFaces()){
            position = i;
        }
    }
    if(position != 0){
        swap(meshes[0], meshes[position]);
    }
}

Harris3D::Mesh * rotateMesh(Harris3D::Mesh * mesh, float angle, vector<float>& center, vector<float>& directionVector){
    //Create the mesh
    Harris3D::Mesh * auxMesh = new Harris3D::Mesh();

    //Obtain vertices
    for (int i=0; i< mesh->getNumVertices();i++){
        float * rotatedVertex = Rotation::rotateVertex(mesh->getVertices()[i].getX()-center[0],mesh->getVertices()[i].getY()-center[1],mesh->getVertices()[i].getZ()-center[2],angle,directionVector[0],directionVector[1],directionVector[2]);
        auxMesh->insertVertex(i, rotatedVertex[0]+center[0], rotatedVertex[1]+center[1], rotatedVertex[2]+center[2]);
    }

    //Obtain faces
    for (int i=0; i< mesh->getNumFaces();i++){
         auxMesh->insertFace(i,mesh->getFaces()[i].getVertex(0),mesh->getFaces()[i].getVertex(1),mesh->getFaces()[i].getVertex(2));
    }

    return auxMesh;
}

void ransac(Harris3D::Mesh * mesh, vector<float>& center, vector<float>& directionVector, Util::Properties& prop){
    vector<int> interestPoints;
    Harris3D::HarrisDetector hd(Shape::mesh, &prop);
    hd.detectInterestPoints(interestPoints);

    //Point cloud analysis
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width    = interestPoints.size();
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for(unsigned i = 0; i < cloud->points.size(); ++i){
        cloud->points[i].x  = mesh->getVertices()[interestPoints[i]].x();
        cloud->points[i].y  = mesh->getVertices()[interestPoints[i]].y();
        cloud->points[i].z  = mesh->getVertices()[interestPoints[i]].z();
    }

    std::vector<int> inliers;
    pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr model_circle(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle);
    ransac.setDistanceThreshold(.5);
    ransac.computeModel();
    ransac.getInliers(inliers);

    for(unsigned i = 0; i < inliers.size() -1; i++){
        Shape::interestPoints.push_back(interestPoints[inliers[i]]);
    }

    //Obtain coefficients
    Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients(model_coefficients);

    center.push_back(model_coefficients[0]);
    center.push_back(model_coefficients[1]);
    center.push_back(model_coefficients[2]);

    directionVector.push_back(model_coefficients[4]);
    directionVector.push_back(model_coefficients[5]);
    directionVector.push_back(model_coefficients[6]);
}

void reconstructMesh(Harris3D::Mesh * mesh, vector<float>& center, vector<float>& directionVector, float alpha, int rotationFactor){
    Harris3D::Mesh * bigMesh = new Harris3D::Mesh();

    int nLoop = 0;
    for (int j=0; j<360;j+=360/rotationFactor){
        float angle = (float) j;

        Harris3D::Mesh * auxMesh = rotateMesh(mesh, angle, center, directionVector);

        //Add vertices to newVertices
        int offset = nLoop * mesh->getNumVertices();
        for (int i=0; i< mesh->getNumVertices();i++){
            bigMesh->insertVertex(offset+i,auxMesh->getVertices()[i].getX(),auxMesh->getVertices()[i].getY(),auxMesh->getVertices()[i].getZ());
        }

        //Obtain faces
        int faceOffset = nLoop * mesh->getNumFaces();
        for (int i=0; i< mesh->getNumFaces();i++){
            bigMesh->insertFace(faceOffset+i,auxMesh->getFaces()[i].getVertex(0)+offset,auxMesh->getFaces()[i].getVertex(1)+offset,auxMesh->getFaces()[i].getVertex(2)+offset);
        }
        cout << "Loop: " << nLoop << endl;
        nLoop++;
    }
    mesh->setVertices(bigMesh->getVertices());
    mesh->setFaces(bigMesh->getFaces());
}

void alignMeshes(Harris3D::Mesh * mesh, vector<float>& center, vector<float>& directionVector, float alpha, int rotationFactor){
    Harris3D::Mesh * bigMesh = new Harris3D::Mesh();

    int nLoop = 0;
    for (int s=0; s<360;s+=360/rotationFactor){
        float angle = (float) s;

        //Find rotated mesh
        Harris3D::Mesh * auxMesh = rotateMesh(mesh, angle, center, directionVector);

        //Calculate translations
        for (int w = 0; w < 1; w++){
            int n = 3 * auxMesh->getNumVertices();
            Eigen::VectorXd x(n), b(n);
            Eigen::SparseMatrix<double> a(n,n);
            x.setZero();
            b.setZero();
            a.setZero();
            std::vector<Eigen::Triplet<double>> tripletList;

            for (int j = 0; j<auxMesh->getNumVertices(); j++){
                Harris3D::Vertex v = auxMesh->getVertices()[j];
                Harris3D::Vertex q = calculateClosestCompatiblePoint(auxMesh, j, Shape::mesh);
                double aux = 0.0;
                std::set<int>::iterator it;
                for (it = v.getAdjacentVertices().begin(); it != v.getAdjacentVertices().end(); ++it){
                    double voronoiTriangle = calculateVoronoiTriangle(auxMesh, j, *it);
                    double edge = calculateEdge(&auxMesh->getVertices()[j], &auxMesh->getVertices()[*it]);

                    double coef = (2 * alpha * voronoiTriangle) / edge;
                    aux += coef;

                    //dim 1
                    tripletList.push_back(Eigen::Triplet<double>(3*j,3*(*it),-coef));

                    //dim 2
                    tripletList.push_back(Eigen::Triplet<double>(3*j+1,3*(*it)+1,-coef));

                    //dim 3
                    tripletList.push_back(Eigen::Triplet<double>(3*j+2,3*(*it)+2,-coef));
                }
                double bCoef = 2*(1 - alpha)*calculateOmega(auxMesh,&q,&v)*calculateVoronoiCell(auxMesh, j);

                aux += bCoef;

                //A
                tripletList.push_back(Eigen::Triplet<double>(3*j,3*j,aux));
                tripletList.push_back(Eigen::Triplet<double>(3*j+1,3*j+1,aux));
                tripletList.push_back(Eigen::Triplet<double>(3*j+2,3*j+2,aux));

                //B
                b(3*j) = bCoef * (q.getX() - v.getX());
                b(3*j+1) = bCoef * (q.getY() - v.getY());;
                b(3*j+2) = bCoef * (q.getZ() - v.getZ());;

                //cout << aux << ", " << bCoef << endl;
            }

            //X
            Eigen::BiCGSTAB<Eigen::SparseMatrix<double> > solver;
            //Eigen::ConjugateGradient<Eigen::SparseMatrix <double>> solver;
            a.setFromTriplets(tripletList.begin(), tripletList.end());
            solver.compute(a);
            x = solver.solve(b);

            //Translate mesh
            for(int i = 0; i<auxMesh->getNumVertices(); i++){
                auxMesh->getVertices()[i].setX(auxMesh->getVertices()[i].getX() + x[3*i]);
                auxMesh->getVertices()[i].setY(auxMesh->getVertices()[i].getY() + x[3*i+1]);
                auxMesh->getVertices()[i].setZ(auxMesh->getVertices()[i].getZ() + x[3*i+2]);
                cout << x[3*i] << ", " << x[3*i+1] << ", " << x[3*i+2] << endl;
            }
        }

        //Add vertices to newVertices
        int offset = nLoop * mesh->getNumVertices();
        for (int i=0; i< mesh->getNumVertices();i++){
            bigMesh->insertVertex(offset+i,auxMesh->getVertices()[i].getX(),auxMesh->getVertices()[i].getY(),auxMesh->getVertices()[i].getZ());
        }

        //Obtain faces
        int faceOffset = nLoop * mesh->getNumFaces();
        for (int i=0; i< mesh->getNumFaces();i++){
            bigMesh->insertFace(faceOffset+i,auxMesh->getFaces()[i].getVertex(0)+offset,auxMesh->getFaces()[i].getVertex(1)+offset,auxMesh->getFaces()[i].getVertex(2)+offset);
        }
        cout << "Loop: " << nLoop << endl;
        nLoop++;
    }
    mesh->setVertices(bigMesh->getVertices());
    mesh->setFaces(bigMesh->getFaces());
}

float calculateMagnitude(vector<float>& v){
    return std::sqrt(pow(v.at(0),2)+pow(v.at(1),2)+pow(v.at(2),2));
}

float calculateDotProduct(vector<float>& v1, vector<float>& v2){
    return (v1.at(0)*v2.at(0) + v1.at(1)*v2.at(1) + v1.at(2)*v2.at(2));
}

void calculateCrossProduct(vector<float>& v1, vector<float>& v2, vector<float>& result){
    result.clear();
    result.push_back(v1.at(1)*v2.at(2)-v1.at(2)*v2.at(1));
    result.push_back(v1.at(2)*v2.at(0)-v1.at(0)*v2.at(2));
    result.push_back(v1.at(0)*v2.at(1)-v1.at(1)*v2.at(0));
}

float calculatePointLineDistance(vector<float>& point, vector<float>& linePoint, vector<float>& lineVector){
    vector<float> v1;
    v1.push_back(point.at(0)-linePoint.at(0));
    v1.push_back(point.at(1)-linePoint.at(1));
    v1.push_back(point.at(2)-linePoint.at(2));
    vector<float> v1xv2;
    calculateCrossProduct(v1,lineVector,v1xv2);
    float sign = v1[0]*lineVector[0]+v1[1]*lineVector[1]+v1[2]*lineVector[2];
    if (sign > 0){
        return (calculateMagnitude(v1xv2)/calculateMagnitude(lineVector));
    }
    else{
        return 1000.0;
    }
    
}

void makeUnitVector(vector<float>& v){
    float size = std::sqrt(pow(v.at(0),2) + pow(v.at(1),2) + pow(v.at(2),2));
    v.at(0) /= size;
    v.at(1) /= size;
    v.at(2) /= size;
}

int detectClosestPoint(Harris3D::Mesh * mesh, vector<float>& linePoint, vector<float>& lineVector){
    vector<float> point;
    point.push_back(mesh->getVertices()[0].getX());
    point.push_back(mesh->getVertices()[0].getY());
    point.push_back(mesh->getVertices()[0].getZ());
    float minDistance = calculatePointLineDistance(point,linePoint,lineVector);
    int minVertex = 0;
    for (int i = 0; i< mesh->getNumVertices(); i++){
        point.clear();
        point.push_back(mesh->getVertices()[i].getX());
        point.push_back(mesh->getVertices()[i].getY());
        point.push_back(mesh->getVertices()[i].getZ());
        float distance = calculatePointLineDistance(point,linePoint,lineVector);
        if (distance < minDistance){
            minDistance = distance;
            minVertex = i;
        }
    }
    return minVertex;
}

float calculatePointPlaneDistance(vector<float>& plane, vector<float>& point){
    return ((plane.at(0)*point.at(0) + plane.at(1)*point.at(1) + plane.at(2)*point.at(2) + plane.at(3))/(std::sqrt(pow(plane.at(0),2)+pow(plane.at(1),2)+pow(plane.at(2),2))));
}

void calculatePlane(vector<float>& point, vector<float>& v1, vector<float>& v2, vector<float>& result){
    result.clear();
    vector<float> v1xv2;
    calculateCrossProduct(v1,v2,v1xv2);
    result.push_back(v1xv2.at(0));
    result.push_back(v1xv2.at(1));
    result.push_back(v1xv2.at(2));
    float d;
    d = -(v1xv2.at(0)*point.at(0)+v1xv2.at(1)*point.at(1)+v1xv2.at(2)*point.at(2));
    result.push_back(d);
}

void detectPointsInPlane(Harris3D::Mesh * mesh, vector<float>& plane, vector<int>& points){
    float threshold = mesh->getDiagonal()*0.01;
    vector<float> point;
    for (int i = 0; i<mesh->getNumVertices(); i++){
        point.clear();
        point.push_back(mesh->getVertices()[i].getX());
        point.push_back(mesh->getVertices()[i].getY());
        point.push_back(mesh->getVertices()[i].getZ());

        if (abs(calculatePointPlaneDistance(plane,point)) < threshold){
            points.push_back(i);
            Shape::interestPoints.push_back(i);
        }
    }
}


void MainWindow::onCompletion(){
    if (!centralWidget()){
        QStringList filenames = QFileDialog::getOpenFileNames(this, tr("Open Files"), "/home", tr("Object Files (*.off *.obj)"));
        if (filenames.isEmpty()){
            QMessageBox::information(0, "error", "No file chosen");
        }
        else{
            // create and read Polyhedron
            vector<string> files;
            for (int i = 0; i<filenames.count(); i++){
                files.push_back(filenames.at(i).toUtf8().constData());
            }

            //Set properties
            Util::Properties prop;
            OptDialog mDialog(&prop);

            //Load mesh to memory
            Shape::mesh = new Harris3D::Mesh(files);

            //Apply openvdb
            if (atoi(prop.getProperty("openvdb").c_str()) == 1){
                openVDB(Shape::mesh);
            }

            //Decimate if mesh is complex
            if (atoi(prop.getProperty("decimate").c_str()) == 1){
                decimate(Shape::mesh,20000);
            }

            //Segmentate mesh
            vector<Harris3D::Mesh *> meshes;
            if (atoi(prop.getProperty("segmentate").c_str()) == 1){
                segmentateMesh(Shape::mesh, meshes);
            }
            else{
                meshes.push_back(Shape::mesh);
            }

            //Find interest points
            vector<float> center;
            vector<float> directionVector;
            ransac(meshes.at(0), center, directionVector, prop);

            //Show interest points
            if (atoi(prop.getProperty("keypoints").c_str())==1){
                Shape::interestPoints.clear();
            }

            //Show original object
            GLOffWidget *oriWidget = new GLOffWidget;

            //Rotate mesh
            double alpha = atof(prop.getProperty("alpha").c_str());
            int rotationFactor = atoi(prop.getProperty("rotation").c_str());
            if (atoi(prop.getProperty("alignment").c_str())==1){
                alignMeshes(meshes.at(0), center, directionVector, alpha, rotationFactor);
            }
            else{
                reconstructMesh(meshes.at(0), center, directionVector, alpha, rotationFactor);
            }

            //Fill hole
//            if (atoi(prop.getProperty("fill-hole").c_str())==1){
//                fillHole(meshes.at(0),center,directionVector);
//            }

            //Rebuild mesh
            Shape::mesh = mergeMeshes(meshes);

            //Shape::mesh->setVertices(auxMesh->getVertices());
            //Shape::mesh->setFaces(auxMesh->getFaces());

            //Output mesh
            outputMesh(Shape::mesh, "output.off");

            //Output inliers
            //outputInliers("inliers.int");

            //Show completed object
            //Shape::interesPoints.clear();
            GLRANSACWidget *newWidget = new GLRANSACWidget;

            //Set the central widget
            setCentralWidget(new RansacWindow(this, oriWidget, newWidget));

            //Clean data
            //Shape::mesh->cleanMesh();
            //Shape::interestPoints.clear();
        }
    }
    else
        QMessageBox::information(0, tr("Cannot add new window"), tr("Already occupied. Undock first."));
}

void fillHole(Harris3D::Mesh * mesh, vector<float>& center, vector<float>& directionVector){
    //Detect closest point
    int point = detectClosestPoint(Shape::mesh, center, directionVector);
    //int rotationFactor = 4;
    //int offset = Shape::mesh->getNumVertices() / 4;
    vector<float> cPoint;
    cPoint.push_back(Shape::mesh->getVertices()[point].getX());
    cPoint.push_back(Shape::mesh->getVertices()[point].getY());
    cPoint.push_back(Shape::mesh->getVertices()[point].getZ());

    //Detect hole center
    float * rotatedPoint = Rotation::rotateVertex(Shape::mesh->getVertices()[point].getX()-center[0], Shape::mesh->getVertices()[point].getY()-center[1], Shape::mesh->getVertices()[point].getZ()-center[2], 180.0, directionVector[0], directionVector[1], directionVector[2]);
    vector<float> holeCenter;
    holeCenter.push_back((Shape::mesh->getVertices()[point].getX()+rotatedPoint[0]+center[0])/2);
    holeCenter.push_back((Shape::mesh->getVertices()[point].getY()+rotatedPoint[1]+center[1])/2);
    holeCenter.push_back((Shape::mesh->getVertices()[point].getZ()+rotatedPoint[2]+center[2])/2);

    //Detect hole edge
    vector<int> hole;

    for(float i = 0; i < 360.0; i += 10.0){
        float * auxPoint = Rotation::rotateVertex(Shape::mesh->getVertices()[point].getX()-center[0], Shape::mesh->getVertices()[point].getY()-center[1], Shape::mesh->getVertices()[point].getZ()-center[2], i , directionVector[0], directionVector[1], directionVector[2]);
        vector<float> direction;
        direction.push_back(auxPoint[0]+center[0]-holeCenter[0]);
        direction.push_back(auxPoint[1]+center[1]-holeCenter[1]);
        direction.push_back(auxPoint[2]+center[2]-holeCenter[2]);

        hole.push_back(detectClosestPoint(Shape::mesh, holeCenter, direction));
    }

    int centerIndex = Shape::mesh->getNumVertices();
    Shape::mesh->insertVertex(centerIndex, holeCenter[0], holeCenter[1], holeCenter[2]);

    //Fill hole
    for(int j = 0; j<hole.size(); j++){
        if (j==0){
            Shape::mesh->insertFace(Shape::mesh->getNumFaces(), hole[hole.size()-1], centerIndex, hole[j]);
        }
        else{
            Shape::mesh->insertFace(Shape::mesh->getNumFaces(), hole[j-1], centerIndex, hole[j]);
        }
    }
}

void MainWindow::onScript(){
    if (!centralWidget()){
        QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home");
        if (dir.isEmpty()){
            QMessageBox::information(0, "error", "No directory chosen");
        }
        else{
            //Set properties
            Util::Properties prop;
            OptDialog mDialog(&prop);
            QDirIterator it(dir, QStringList() << "*.off", QDir::Files, QDirIterator::Subdirectories);
            while (it.hasNext()){
                //Load mesh to memory
                QString filename = it.next();
                cout << filename.toUtf8().constData() << endl;
                try{
                    Shape::mesh = new Harris3D::Mesh(filename.toUtf8().constData());

                    //Get basename
                    std::string basename = boost::filesystem::basename(filename.toUtf8().constData());

                    //Apply openvdb
                    if (atoi(prop.getProperty("openvdb").c_str()) == 1){
                        openVDB(Shape::mesh);
                    }

                    //Decimate if mesh is complex
                    if (atoi(prop.getProperty("decimate").c_str()) == 1){
                        decimate(Shape::mesh,20000);
                    }

                    //Segmentate mesh
                    vector<Harris3D::Mesh *> meshes;
                    if (atoi(prop.getProperty("segmentation").c_str()) == 1){
                        segmentateMesh(Shape::mesh, meshes);
                    }
                    else{
                        meshes.push_back(Shape::mesh);
                    }

                    //Find interest points
                    vector<float> center;
                    vector<float> directionVector;
                    ransac(meshes.at(0), center, directionVector, prop);

                    //Show interest points
                    if (atoi(prop.getProperty("keypoints").c_str())==1){
                        Shape::interestPoints.clear();
                    }


                    //Rotate mesh
                    double alpha = atof(prop.getProperty("alpha").c_str());
                    int rotationFactor = atoi(prop.getProperty("rotation").c_str());
                    if (atoi(prop.getProperty("alignment").c_str())==1){
                        alignMeshes(meshes.at(0), center, directionVector, alpha, rotationFactor);
                    }
                    else{
                        reconstructMesh(meshes.at(0), center, directionVector, alpha, rotationFactor);
//                        int nLoop = 0;
//                        for (int j=0; j<360;j+=360/rotationFactor){
//                            float angle = (float) j;
                            
//                            Harris3D::Mesh * auxMesh = rotateMesh(meshes.at(0), angle, center, directionVector);
                            
//                            //Output mesh
//                            std::string outfile = basename + "_output" + std::to_string(nLoop) + ".off";
//                            cout << outfile << endl;
//                            outputMesh(Shape::mesh, outfile);
                            
//                            cout << "Loop: " << nLoop << endl;
//                            nLoop++;
//                        }
                        
                    }

                    //Fill hole
                    if (atoi(prop.getProperty("hole-filling").c_str())==1){
                        fillHole(meshes.at(0),center,directionVector);
                    }

                    //Rebuild mesh
                    if (atoi(prop.getProperty("segmentation").c_str()) == 1){
                        Shape::mesh = mergeMeshes(meshes);
                    }


                    //Output mesh
                    std::string outfile = basename + "_output.off";
                    cout << outfile << endl;
                    outputMesh(Shape::mesh, outfile);
                }
                catch (int e){
                  cout << filename.toUtf8().constData() << endl;
                }

                //Clean mesh
                Shape::interestPoints.clear();
                Shape::mesh->cleanMesh();
            }
            QMessageBox::information(0, tr("Exito"), tr("Exito"));
        }
    }
    else
        QMessageBox::information(0, tr("Cannot add new window"), tr("Already occupied. Undock first."));
}

void MainWindow::onHoleFilling(){
    if (!centralWidget()){
        QStringList filenames = QFileDialog::getOpenFileNames(this, tr("Open Files"), "/home", tr("Object Files (*.off *.obj)"));
        if (filenames.isEmpty()){
            QMessageBox::information(0, "error", "No file chosen");
        }
        else{
            // create and read Polyhedron
            vector<string> files;
            for (int i = 0; i<filenames.count(); i++){
                files.push_back(filenames.at(i).toUtf8().constData());
            }

            //Set properties
            Util::Properties prop;
            OptDialog mDialog(&prop);

            Shape::mesh = new Harris3D::Mesh(files);

            //Apply openvdb
            openVDB(Shape::mesh);

            //Decimate if mesh is complex
            decimate(Shape::mesh,20000);

            //RANSAC
            vector<float> center;
            vector<float> directionVector;
            ransac(Shape::mesh, center, directionVector, prop);

            //Reconstruct mesh
            double alpha = atof(prop.getProperty("alpha").c_str());
            reconstructMesh(Shape::mesh, center, directionVector, alpha, 4);

            //Detect closest point
            int point = detectClosestPoint(Shape::mesh, center, directionVector);
            int offset = Shape::mesh->getNumVertices();
            vector<float> vPoint;
            vPoint.push_back(Shape::mesh->getVertices()[point].getX());
            vPoint.push_back(Shape::mesh->getVertices()[point].getY());
            vPoint.push_back(Shape::mesh->getVertices()[point].getZ());
            float cpDistance = calculatePointLineDistance(vPoint, center, directionVector);

            //Detect hole center
            float * rotatedPoint = Rotation::rotateVertex(Shape::mesh->getVertices()[point].getX()-center[0],Shape::mesh->getVertices()[point].getY()-center[1],Shape::mesh->getVertices()[point].getZ()-center[2],180.0,directionVector[0],directionVector[1],directionVector[2]);
            vector<float> holeCenter;
            holeCenter.push_back((Shape::mesh->getVertices()[point].getX()+rotatedPoint[0]+center[0])/2);
            holeCenter.push_back((Shape::mesh->getVertices()[point].getY()+rotatedPoint[1]+center[1])/2);
            holeCenter.push_back((Shape::mesh->getVertices()[point].getZ()+rotatedPoint[2]+center[2])/2);

            //Detect lower hole edge
            vector<int> hole;
            int rotationFactor = 8;
            for(float i = 0; i < 360.0/rotationFactor; i += 360.0/((float)rotationFactor * 4.0)){
                float * auxPoint = Rotation::rotateVertex(Shape::mesh->getVertices()[point].getX()-center[0],Shape::mesh->getVertices()[point].getY()-center[1],Shape::mesh->getVertices()[point].getZ()-center[2],i,directionVector[0],directionVector[1],directionVector[2]);;
                vector<float> direction;
                direction.push_back(auxPoint[0]+center[0]-holeCenter[0]);
                direction.push_back(auxPoint[1]+center[1]-holeCenter[1]);
                direction.push_back(auxPoint[2]+center[2]-holeCenter[2]);

                hole.push_back(detectClosestPoint(Shape::mesh, holeCenter, direction));
            }

            //Detect plane
            vector<float> plane;
            vector<float> v;
            v.push_back(Shape::mesh->getVertices()[point].getX()-center.at(0));
            v.push_back(Shape::mesh->getVertices()[point].getY()-center.at(1));
            v.push_back(Shape::mesh->getVertices()[point].getZ()-center.at(2));
            calculatePlane(center,v,directionVector,plane);

            //Detect points in plane
            vector<int> planePoints;
            detectPointsInPlane(Shape::mesh, plane, planePoints);
            Shape::interestPoints.clear();

            //Calculate perpendicular plane
            vector<float> normalPlane;
            normalPlane.push_back(directionVector.at(0));
            normalPlane.push_back(directionVector.at(1));
            normalPlane.push_back(directionVector.at(2));
            normalPlane.push_back(-(directionVector.at(0)*Shape::mesh->getVertices()[point].getX()+directionVector.at(1)*Shape::mesh->getVertices()[point].getY()+directionVector.at(2)*Shape::mesh->getVertices()[point].getZ()));

            //Detect upper and lower hole boundary
            float maxDistance = 0;
            float minDistance = 0;
            float maxIndex = point;
            float minIndex = point;
            for (int i = 0; i< planePoints.size(); i++){
                vector<float> planePoint;
                planePoint.push_back(Shape::mesh->getVertices()[planePoints.at(i)].getX());
                planePoint.push_back(Shape::mesh->getVertices()[planePoints.at(i)].getY());
                planePoint.push_back(Shape::mesh->getVertices()[planePoints.at(i)].getZ());
                float distance = calculatePointPlaneDistance(normalPlane,planePoint);
                if (calculatePointLineDistance(planePoint,center,directionVector) < cpDistance+Shape::mesh->getDiagonal()*0.04){
                    if (distance > maxDistance){
                        maxDistance = distance;
                        maxIndex = planePoints.at(i);
                    }
                    if (distance < minDistance){
                        minDistance = distance;
                        minIndex = planePoints.at(i);
                    }
                }
            }
            cout << "Min point: " << minIndex << ", Max point: "<< maxIndex << endl;

            //Detect upper hole center
            rotatedPoint = Rotation::rotateVertex(Shape::mesh->getVertices()[maxIndex].getX()-center[0],Shape::mesh->getVertices()[maxIndex].getY()-center[1],Shape::mesh->getVertices()[maxIndex].getZ()-center[2],180.0,directionVector[0],directionVector[1],directionVector[2]);
            vector<float> upperHoleCenter;
            upperHoleCenter.push_back((Shape::mesh->getVertices()[maxIndex].getX()+rotatedPoint[0]+center[0])/2);
            upperHoleCenter.push_back((Shape::mesh->getVertices()[maxIndex].getY()+rotatedPoint[1]+center[1])/2);
            upperHoleCenter.push_back((Shape::mesh->getVertices()[maxIndex].getZ()+rotatedPoint[2]+center[2])/2);

            //Detect upper hole edge
            vector<int> upperHole;
            for(float i = 0; i < 360.0/rotationFactor; i += 360.0/((float)rotationFactor * 4.0)){
                float * auxPoint = Rotation::rotateVertex(Shape::mesh->getVertices()[maxIndex].getX()-center[0],Shape::mesh->getVertices()[maxIndex].getY()-center[1],Shape::mesh->getVertices()[maxIndex].getZ()-center[2],i,directionVector[0],directionVector[1],directionVector[2]);
                vector<float> direction;
                direction.push_back(auxPoint[0]+center[0]-upperHoleCenter[0]);
                direction.push_back(auxPoint[1]+center[1]-upperHoleCenter[1]);
                direction.push_back(auxPoint[2]+center[2]-upperHoleCenter[2]);

                upperHole.push_back(detectClosestPoint(Shape::mesh, upperHoleCenter, direction));
            }

            //Fill hole
            for(int i = 0; i<rotationFactor; i++){
                //core lower hole fill
                Shape::mesh->insertFace(Shape::mesh->getNumFaces(),point+offset*i,point+offset*((i+rotationFactor/4)%rotationFactor),point+offset*((i+rotationFactor/2)%rotationFactor));
                Shape::mesh->insertFace(Shape::mesh->getNumFaces(),point+offset*((i+rotationFactor/2)%rotationFactor),point+offset*((i+rotationFactor/4)%rotationFactor),point+offset*i);
                Shape::mesh->insertFace(Shape::mesh->getNumFaces(),point+offset*i,point+offset*((i+1)%rotationFactor),point+offset*((i+2)%rotationFactor));
                Shape::mesh->insertFace(Shape::mesh->getNumFaces(),point+offset*((i+2)%rotationFactor),point+offset*((i+1)%rotationFactor),point+offset*i);

                //auxiliary lower hole fill
                for(int j = 1; j<hole.size(); j++){
                    Shape::mesh->insertFace(Shape::mesh->getNumFaces(),hole[j-1]+offset*i,hole[j]+offset*i,point+offset*((i+1)%rotationFactor));
                    Shape::mesh->insertFace(Shape::mesh->getNumFaces(),point+offset*((i+1)%rotationFactor),hole[j]+offset*i,hole[j-1]+offset*i);
                }

                //core upper hole fill
                Shape::mesh->insertFace(Shape::mesh->getNumFaces(),maxIndex+offset*i,maxIndex+offset*((i+rotationFactor/4)%rotationFactor),maxIndex+offset*((i+rotationFactor/2)%rotationFactor));
                Shape::mesh->insertFace(Shape::mesh->getNumFaces(),maxIndex+offset*((i+rotationFactor/2)%rotationFactor),maxIndex+offset*((i+rotationFactor/4)%rotationFactor),maxIndex+offset*i);
                Shape::mesh->insertFace(Shape::mesh->getNumFaces(),maxIndex+offset*i,maxIndex+offset*((i+1)%rotationFactor),maxIndex+offset*((i+2)%rotationFactor));
                Shape::mesh->insertFace(Shape::mesh->getNumFaces(),maxIndex+offset*((i+2)%rotationFactor),maxIndex+offset*((i+1)%rotationFactor),maxIndex+offset*i);

                //auxiliary upper hole fill
                for(int j = 1; j<upperHole.size(); j++){
                    Shape::mesh->insertFace(Shape::mesh->getNumFaces(),upperHole[j-1]+offset*i,upperHole[j]+offset*i,maxIndex+offset*((i+1)%rotationFactor));
                    Shape::mesh->insertFace(Shape::mesh->getNumFaces(),maxIndex+offset*((i+1)%rotationFactor),upperHole[j]+offset*i,upperHole[j-1]+offset*i);
                }
            }

            //Shape::interestPoints.clear();

            //Output
            outputMesh(Shape::mesh, "output.off");

            //Set the central widget
            setCentralWidget(new OffWindow(this));

            //Clean data
            Shape::mesh->cleanMesh();
            Shape::interestPoints.clear();
        }
    }
    else
        QMessageBox::information(0, tr("Cannot add new window"), tr("Already occupied. Undock first."));
}

void MainWindow::onAddNew()
{
    if (!centralWidget())
        setCentralWidget(new Window(this));
    else
        QMessageBox::information(0, tr("Cannot add new window"), tr("Already occupied. Undock first."));
}

void MainWindow::onOpen()
{
    if (!centralWidget()){
        QString offFilename = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Object Files (*.off *.obj)"));
        QFile file(offFilename);
        int canOpen = file.open(QIODevice::ReadOnly);
        file.close();
        if (!canOpen){
            QMessageBox::information(0, "error", file.errorString());

        }
        else{
            Shape::mesh = new Harris3D::Mesh(offFilename.toUtf8().constData());

            //Set the central widget
            setCentralWidget(new OffWindow(this));

            //Clean
            Shape::mesh->cleanMesh();
            Shape::interestPoints.clear();
        }
    }
    else
        QMessageBox::information(0, tr("Cannot add new window"), tr("Already occupied. Undock first."));
}
