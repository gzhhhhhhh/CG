# CG-homework1

### 在代码框架中我共修改了以下四个函数
``` C++

static bool insideTriangle(float x, float y, const Vector3f* _v);

void rst::rasterizer::rasterize_triangle(const Triangle& t);

void rst::rasterizer::clear(rst::Buffers buff);

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h);

```

### 具体内容如下：

#### 代码：

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

insideTriangle(float x, float y, const Vector3f* _v)：返回目标点是否在三角形内部

在insidetrigangle函数中，先以一定的顺序(Triangle类的定义中已经保证顶点为逆时针排序)得到三条边向量。<br> 
之后将三个顶点指向目标点的向量分别与对应的边向量进行叉积。 <br> 
若得到的三个向量方向相同，则目标点在三角形内，返回True，否则在三角形外,返回false。 




#### 代码：
```C++

void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // 创建三角形的 2 维 bounding box
    Eigen::Vector2f min_point, max_point;
    min_point.x() = MIN(MIN(v[0].x(), v[1].x()), v[2].x());
    min_point.y() = MIN(MIN(v[0].y(), v[1].y()), v[2].y());
    max_point.x() = MAX(MAX(v[0].x(), v[1].x()), v[2].x());
    max_point.y() = MAX(MAX(v[0].y(), v[1].y()), v[2].y());

    // iterate through the pixel and find if the current pixel is inside the triangle
    // If so, use the following code to get the interpolated z value.
    //float alpha, beta, gamma;
    //std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    float alpha, beta, gamma;

    std::vector<Eigen::Vector2f> super_sample_step_length{
        {0.25, 0.25},
        {0.75, 0.25},
        {0.25, 0.75},
        {0.75, 0.75},
    };
    //遍历此 bounding box 内的所有像素（使用其整数索引）
    for (int i = min_point.x(); i <= max_point.x(); i++) {
        for (int j = min_point.y(); j <= max_point.y(); j++) {
            int count = 0;
            float min_depth = FLT_MAX;
            //对每个像素进行2*2的超采样
            for (int k = 0; k < 4; k++) {
                //如果点在三角形内部，用题目所给代码求出深度值
                if (insideTriangle((float)i + super_sample_step_length[k][0], (float)j + super_sample_step_length[k][1], t.v)) {
                    std::tie(alpha, beta, gamma) = computeBarycentric2D(i, j, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    //更新最小深度值，以确定该像素的最终深度
                    if (z_interpolated < min_depth) {
                        min_depth = z_interpolated;
                    }
                    //如果当前采样点的深度值小于已存储的深度缓冲中的值，这就说明这个点离我们更近，更新深度缓冲和颜色缓冲。
                    if (z_interpolated < sample_depth_buffer[get_index(i, j)][k]) {
                        sample_depth_buffer[get_index(i, j)][k] = z_interpolated;
                        sample_frame_buffer[get_index(i, j)][k] = t.getColor();
                    }
                    count++;
                }
            }
            //如果当前像素超采样点有在三角形中的，对其进行一个颜色平均
            if (count != 0) {
                Eigen::Vector3f current_color = { 0, 0, 0 };
                for (int k = 0; k < 4; k++) {
                    current_color += sample_frame_buffer[get_index(i, j)][k];
                }
                set_pixel(Eigen::Vector3f((float)i, (float)j, min_depth), current_color / 4.0);
                depth_buf[get_index(i, j)] = min_depth;
            }
        }
    }
 }

```
void rst::rasterizer::rasterize_triangle(const Triangle& t) 这个函数中实现对三角形t的光栅化处理

首先先求出三角形t的包围面(bounding box).<br> 
之后再包围面中进行像素的遍历，由于要抗锯齿，这里对每个像素点进行2*2的超采样，依次对四个点进行判断，是否在三角形中。<br> 
若在三角形中，则运用题目所给的代码得到其插值，并计数器count加一，判断是否更新其最小深度值，以确定该像素的最终深度。<br> 
如果当前插值比原先深度缓存中的值更小，那么就说明当前像素前的物体比之前渲染的物体离视点更近，那么就对应的更新深度缓冲和颜色缓冲中的值。<br> 
如果计数器不等于0，就说明当前像素存在至少一个采样点在三角形内，将颜色信息进行平均，利用set_pixel赋给当前像素点。<br> 


#### 代码：

```C++
std::vector<std::vector<Eigen::Vector3f>> sample_frame_buffer;
std::vector<std::vector<float>> sample_depth_buffer;
```
在rasterizer.hpp中加上两行定义，sample_frame_buffer用来存每个超采样点的颜色信息，sample_depth_buffer用来存每个超采样点的深度信息

#### 代码：

```C++

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
        //清理超采样颜色列表
        std::fill(sample_frame_buffer.begin(), sample_frame_buffer.end(), std::vector<Eigen::Vector3f>(4, { 0, 0, 0 }));
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {   
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        //清理超采样深度列表
        std::fill(sample_depth_buffer.begin(), sample_depth_buffer.end(), std::vector<float>(4, std::numeric_limits<float>::infinity()));
    }
}

```

这个函数用于对缓冲数组进行清理，分别是对frame_buf、sample_frame_buffer、depth_buf和sample_depth_buffer进行清理<br> 

在定义了sample_frame_buffer和sample_depth_buffer之后，同样需要对其进行清理。利用std.fill函数清理超采样点的颜色信息和深度信息<br> 

#### 代码：

```C++
rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //初始化超采样列表
    sample_frame_buffer.resize(w * h);
    for (auto& row : sample_frame_buffer) {
            row.resize(4);
    }
    sample_depth_buffer.resize(w * h);
    for (auto& row : sample_depth_buffer) {
            row.resize(4);
     }
}
```

这个函数主要实现的是对缓冲数组的初始化，初始化了frame_buf、sample_frame_buffer、depth_buf和sample_depth_buffer数组<br> 
利用resize函数初始化vector的大小，因为像素点的个数是w*h，所以将四个vector的大小都定义为w*h，而对每个像素是进行2*2的采样，在sample_frame_buffer和sample_depth_buffer中每行的大小定义为4<br> 
