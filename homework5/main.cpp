#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main()
{
    Scene scene(1280, 960);

    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2);//定义半径为2的球体
    sph1->materialType = DIFFUSE_AND_GLOSSY;//设置球体材质属性
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8);//设置球体颜色

    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);
    sph2->ior = 1.5;//菲涅尔系数设置？
    sph2->materialType = REFLECTION_AND_REFRACTION;

    scene.Add(std::move(sph1));//左值转化为右值引用后传递给add函数
    scene.Add(std::move(sph2));

    Vector3f verts[4] = {{-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16}};//网格顶点
    uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3};//网格顶点索引
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};//纹理坐标
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st);//定义三角形网格对象
    mesh->materialType = DIFFUSE_AND_GLOSSY;

    scene.Add(std::move(mesh));
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5));//增加光源对象，0.5的intensity光照
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5));    

    Renderer r;//定义着色器对象
    r.Render(scene);//着色

    return 0;
}