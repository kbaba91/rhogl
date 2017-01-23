#ifndef ROTATION
#define ROTATION

class Rotation
{
public:
    static float * rotateVertex(float x, float y, float z, float angle, float u, float v, float w);

private:
    static float rotationMatrix[4][4];
    static float inputMatrix[4][1];
    static float outputMatrix[4][1];
    static void multiplyMatrix();
    static void setUpRotationMatrix(float angle, float u, float v, float w);
};

#endif // ROTATION

