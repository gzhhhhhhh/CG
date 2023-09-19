# CG
```C++
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f s1, s2, s3;
    s1 = _v[1] - _v[0];
    s2 = _v[2] - _v[1];
    s3 = _v[0] - _v[2];
    Eigen::Vector3f p(x, y, 0);
    // calculate the cross of two vector
    float direction1 = ((p - _v[0]).cross(s1)).z();
    float direction2 = ((p - _v[1]).cross(s2)).z();
    float direction3 = ((p - _v[2]).cross(s3)).z();
    // Determine if the sybol is the same
    if ((direction1 > 0 && direction2 > 0 && direction3 > 0) || (direction1 < 0 && direction2 < 0 && direction3 < 0))
        return true;
    return false;
}
```
insideTriangle(float x, float y, const Vector3f* _v)：返回点是否在三角形内部
在insidetrigangle函数中，先以一定的顺序(严格按照顺时针或逆时针)得到三条边向量。之后将三个顶点指向目标点的向量分别与对应的边向量进行叉积。
若得到的三个向量方向相同，则目标点在三角形内，否则在三角形外。
