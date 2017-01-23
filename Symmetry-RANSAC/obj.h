#pragma once
#include <string>
#include <vector>
//#include "vec_math.h"

#ifndef uint
#define uint unsigned int
#endif

// ---------------------------------------- //
char int_to_str_out[100];
std::string int_to_str(const int x)
{
    sprintf(int_to_str_out,"%d",x);

    return std::string(int_to_str_out);
}
// ---------------------------------------- //
struct Tri3
{
    int points[3];
    int normals[3];
    int tex_coords[3];
    int material;
};
// ---------------------------------------- //
class Texture
{
    public:
    Texture():gl_handle(-1){}

    std::string filename;
    int gl_handle;
};
// ---------------------------------------- //
class Material
{
    public:

    Material()
    {
        ambient = vec3f ( 0.6 , 0.3 ,0 );
        diffuse = vec3f ( 0.3 , 0.3 ,0.3 );
        specular= vec3f ( 0,0,0 );
        alpha = 1;
        reflect = 0;
        name = "";
    }

    std::string name;
    vec3f diffuse;
    vec3f specular;
    vec3f ambient;
    float alpha,reflect;
    Texture diffuse_map;
    Texture diffuse_map2;
    Texture diffuse_map3;
    Texture diffuse_map4;
    Texture diffuse_map5;
    Texture ambient_map;
    Texture bump_map;
};
// ---------------------------------------- //
struct SubGroup{ int listid,material;};

class Geometry
{
    public:
    std::string name;
    std::string filename;
    std::vector<vec3f> points;
    std::vector<vec3f> normals;
    std::vector<vec3f> tex_coords;
    std::vector<Tri3> triangles;
    vec3f bb_min;
    vec3f bb_max;

    vec3f rotation;
    vec3f translation;

    vec3f initial_rotation;
    vec3f initial_translation;

    std::vector<SubGroup> subgroups;

    Geometry(){clear() ;}

    void clear()
    {
        points.clear();
        normals.clear();
        tex_coords.clear();
        triangles.clear();
        subgroups.clear();
        rotation = vec3f(0,0,0);
        translation = vec3f(0,0,0);
    };
    void init_bbox()
    {
        if (points.size ()<2) return;
        if (triangles.size ()<2) return;

        vec3f min,max;
        min = max = points[triangles[0].points[0]];

        for ( std::size_t i = 0 ; i < triangles.size () ; i++ )
        loopj(0,3)
        {
            vec3f p=points[triangles[i].points[j]];
            if ( p.x < min.x ) min.x = p.x;
            if ( p.y < min.y ) min.y = p.y;
            if ( p.z < min.z ) min.z = p.z;
            if ( p.x > max.x ) max.x = p.x;
            if ( p.y > max.y ) max.y = p.y;
            if ( p.z > max.z ) max.z = p.z;
        }
        bb_min = min;
        bb_max = max;

        translation=(bb_max+bb_min)*0.5;

        initial_rotation=vec3f(0,0,0);
        initial_translation=translation;

        bb_min = bb_min - translation;
        bb_max = bb_max - translation;
        loopi(0,points.size())points[i]=points[i]-translation;
    }
    void normalize_()
    {
        init_bbox();

        float sx=bb_max.x-bb_min.x;
        float sy=bb_max.y-bb_min.y;
        float sz=bb_max.z-bb_min.z;
        float maxsize=sx;
        if(sy>maxsize)maxsize=sy;
        if(sz>maxsize)maxsize=sz;

        float mx=(bb_max.x+bb_min.x)/2;
        float my=(bb_max.y+bb_min.y)/2;
        float mz=(bb_max.z+bb_min.z)/2;

        for ( std::size_t i = 0 ; i < points.size () ; i++ )
        {
            float x=points[i].x;
            float y=points[i].y;
            float z=points[i].z;

            x=(x-mx)/maxsize;
            y=(y-my)/maxsize;
            z=(z-mz)/maxsize;

            points[i].x=x;
            points[i].y=y;
            points[i].z=z;
        }
    }

    void init_subgroups(Material *materials)
    {
        SubGroup sg;

        sg.listid = glGenLists(1);
        glNewList(sg.listid, GL_COMPILE);


        //glBegin(GL_LINES);//
        glBegin(GL_TRIANGLES);

        int mat=-1;

        loopi(0,triangles.size())
        {
            Tri3 &t=triangles[i];

            if(mat!=t.material)
            {
                if(mat>=0)
                {
                    glEnd();
                    glEndList();
                    subgroups.push_back(sg);
                    sg.listid = glGenLists(1);
                    glNewList(sg.listid, GL_COMPILE);
                    glBegin(GL_TRIANGLES);
                }
                mat=t.material;
                sg.material=mat;
                if(sg.material<0)sg.material=0;
            }

            vec3f v0=points[t.points[0]];
            vec3f v1=points[t.points[1]];
            vec3f v2=points[t.points[2]];
            vec3f n0;
            n0.cross(v1-v0,v0-v2);
            n0.normalize();


            loopk(0,3)
            {
                int j=k%3;
                vec3f n(0,1,0);
                if(normals.size()>0)
                if(t.normals[j]>=0)
                    n=normals[t.normals[j]];
                glNormal3f(n0.x,n0.y,n0.z);
/*
                vec3f n(0,1,0);
                if(normals.size()>0)
                if(t.normals[j]>=0)
                    n=normals[t.normals[j]];
                glNormal3f(n.x,n.y,n.z);
                */

                vec3f tc(0,0,0);
                if(tex_coords.size()>0)
                if(t.tex_coords[j]>=0)
                    tc=tex_coords[t.tex_coords[j]];
                //glTexCoord2f(tc.x,tc.y);

                vec3f v=points[t.points[j]];
                glVertex3f(v.x,v.y,v.z);
            }
        }

        glEnd();
        glEndList();

        glFlush();

        subgroups.push_back(sg);
    };
};
// ---------------------------------------- //
class OBJ {
public:

    std::vector<Material>	 materials;
    std::vector<Geometry>	 objects;

    OBJ(){}
    ~OBJ(){

        //materials.clear();
        //objects.clear();
    }

    int push_back( Material& mat )
    {
        materials.push_back(mat);
        return materials.size();
    }

    // -------------- materials ------------ //
    int get_material_index ( std::string name )
    {
        for ( uint i = 0 ; i<materials.size(); i++ )
            if ( name.compare( materials[i].name ) == 0 ) return i;

        printf("couldnt find material %s\n",name.c_str() );
        return -1;
    }
    void print_materials ()
    {
        for(uint i=0;i<materials.size();i++)
        {
            printf("Material %i : %s\n",
                i,materials[i].name.c_str());
            printf("  Ambient RGB %2.2lf %2.2lf %2.2lf\n",
                materials[i].ambient.x,
                materials[i].ambient.y,
                materials[i].ambient.z);
            printf("  Specular RGB %2.2lf %2.2lf %2.2lf\n",
                materials[i].specular.x,
                materials[i].specular.y,
                materials[i].specular.z);
            printf("  Diffuse RGB %2.2lf %2.2lf %2.2lf\n",
                materials[i].diffuse.x,
                materials[i].diffuse.y,
                materials[i].diffuse.z);
            printf("  Diff. Tex : %s \n",
                materials[i].diffuse_map.filename.c_str()	);
            printf("  Alpha %2.2f\n",
                   materials[i].alpha);
        }
    }
    // -------------- objects -------------- //
    void print_objects ()
    {
        for ( uint i = 0 ; i<objects.size(); i++ )
        {
            cout << "Object[" << i << "] = " << objects[i].name.c_str() << endl;
            cout << "  - Tri3s: " << objects[i].triangles.size() << endl;
            cout << "  - Vertices: " << objects[i].points.size() << endl;
            cout << "  - Normals: " << objects[i].normals.size() << endl;
            cout << "  - TexCoords: " << objects[i].tex_coords.size() << endl;
            printf("  - Center   : %2.3lf %2.3lf %2.3lf\n",objects[i].translation.x,objects[i].translation.y,objects[i].translation.z);
            printf("  - BB max   : %2.3lf %2.3lf %2.3lf\n",objects[i].bb_max.x,objects[i].bb_max.y,objects[i].bb_max.z);
            printf("  - BB min   : %2.3lf %2.3lf %2.3lf\n",objects[i].bb_min.x,objects[i].bb_min.y,objects[i].bb_min.z);
        }
    }
    int get_object_index ( std::string& name )
    {
        for ( uint i = 0 ; i<objects.size(); i++ )
            if ( name.compare( objects[i].name ) == 0 ) return i;

        return -1;
    }

};
