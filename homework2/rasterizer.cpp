// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

const bool ssaa = true;

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f p;
    Eigen::Vector3f tri;
    p = Eigen::Vector3f(_v[0].x() - x, _v[0].y() - y,1.0f);
    tri = Eigen::Vector3f(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 1.0f);
    Eigen::Vector3f f1 = p.cross(tri);
    p = Eigen::Vector3f(_v[1].x() - x, _v[1].y() - y, 1.0f);
    tri = Eigen::Vector3f(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 1.0f);
    Eigen::Vector3f f2 = p.cross(tri);
    p = Eigen::Vector3f(_v[2].x() - x, _v[2].y() - y, 1.0f);
    tri = Eigen::Vector3f(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 1.0f);
    Eigen::Vector3f f3 = p.cross(tri);
    if (f1[2] > 0) {
        if (f2[2] > 0 && f3[2] > 0)return true;
    }
    if (f1[2] < 0) {
        if (f2[2] < 0 && f3[2] < 0)return true;
    }
    return false;
}

Eigen::Vector3f computeBary(float x, float y, const Vector3f* tri)
{
    Eigen::Vector3f PA(tri[0].x() - x, tri[0].y() - y, 1.0f);
    Eigen::Vector3f PB(tri[1].x() - x, tri[1].y() - y, 1.0f);
    Eigen::Vector3f PC(tri[2].x() - x, tri[2].y() - y, 1.0f);
    float a = PB.cross(PC).norm();
    float b = PA.cross(PC).norm();
    float c = PA.cross(PB).norm();
    Eigen::Vector3f ans(a / (a + b + c), b / (a + b + c), c / (a + b + c));
    return ans;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport tr \ormation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
    if (ssaa) {
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++)
            {
                Eigen::Vector3f color(0, 0, 0);
                for (int i = 0; i < 4; i++)
                    color += frame_buf_2xSSAA[get_index(x, y)][i];
                color /= 4;
                set_pixel(Eigen::Vector3f(x, y, 1.0f), color);
            }

        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    //计算出包围盒
    float xmin = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    float ymin = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    float xmax = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    float ymax = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));
 
    //遍历
    for (int x = xmin; x < xmax; ++x) {
        for (int y = ymin; y < ymax; ++y) {
            Eigen::Vector3f point(x, y, 1.0);
            if (ssaa) {
                //child pixel
                int inside_count = 0;
                int update_depth = 0;
                int index = 0;
                for (float i = 0.25; i < 1.0; i += 0.5) {
                    for (float j = 0.25; j < 1.0; j += 0.5)
                    {
                        if (insideTriangle(x + i, y + j, t.v)) {
                            Eigen::Vector3f barycoord = computeBary(x + i, y + j, t.v);
                            float z_interpolated = barycoord[0] * v[0].z() / v[0].w() + barycoord[1] * v[1].z() / v[1].w() + barycoord[2] * v[2].z() / v[2].w();
                            float w_reciprocal = 1.0 / (barycoord[0] / v[0].w() + barycoord[1] / v[1].w() + barycoord[2] / v[2].w());
                            z_interpolated *= w_reciprocal;
                            if (z_interpolated < depth_buf_2xSSAA[get_index(x, y)][index]) {

                                point << x + i, y + j, 1.0;
                                //set_pixel_ssaa(point, index, t.getColor());
                                frame_buf_2xSSAA[get_index(x, y)][index] = t.getColor();
                                depth_buf_2xSSAA[get_index(x, y)][index] = z_interpolated;
                                inside_count++;
                                update_depth += z_interpolated;
                            }

                        }
                        index++;
                    }

                }
            }
            else {
                if (insideTriangle(x + 0.5f, y + 0.5f, t.v))
                {
                    // 计算重心坐标
                    Eigen::Vector3f barycoord = computeBary((float)x + 0.5, (float)y + 0.5, t.v);
                    // 深度插值和透视除法
                    float z_interpolated = barycoord[0] * v[0].z() / v[0].w() + barycoord[1] * v[1].z() / v[1].w() + barycoord[2] * v[2].z() / v[2].w();
                    // 透视校正
                    float w_reciprocal = 1.0 / (barycoord[0] / v[0].w() + barycoord[1] / v[1].w() + barycoord[2] / v[2].w());
                    z_interpolated *= w_reciprocal;
                    if (z_interpolated < depth_buf[get_index(x, y)]) {
                        Eigen::Vector3f point(x, y, 1.0f);
                        depth_buf[get_index(x, y)] = z_interpolated;
                        set_pixel(point, t.getColor());
                    }
                }
            }
        }
    }

    
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
        for (int i = 0; i < frame_buf_2xSSAA.size(); i++) {
            frame_buf_2xSSAA[i].resize(4);
            std::fill(frame_buf_2xSSAA[i].begin(), frame_buf_2xSSAA[i].end(), Eigen::Vector3f{ 0, 0, 0 });
        }
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        for (int i = 0; i < depth_buf_2xSSAA.size(); i++) {
            depth_buf_2xSSAA[i].resize(4);
            std::fill(depth_buf_2xSSAA[i].begin(), depth_buf_2xSSAA[i].end(), std::numeric_limits<float>::infinity());
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_2xSSAA.resize(w * h);
    depth_buf_2xSSAA.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on