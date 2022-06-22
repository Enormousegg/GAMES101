#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

//// Implementation of Path Tracing
//Vector3f Scene::castRay(const Ray &ray, int depth) const
//{
//    // TO DO Implement Path Tracing Algorithm here
//    if (depth > 3) return Vector3f();
//    //Dont have Intersection In Scene
//    Intersection intersToScene = Scene::intersect(ray);
//    if (!intersToScene.happened)
//        return Vector3f();
//    if (intersToScene.m->hasEmission())
//        return intersToScene.m->getEmission();
//
//    Vector3f L_dir = { 0, 0, 0 }, L_indir = { 0, 0, 0 };
//
//    //Calculate the Intersection from point to light in order to calculate direct Color
//    Intersection LightPos;
//    float lightpdf = 0.0f;
//    sampleLight(LightPos, lightpdf);
//    Vector3f LightDir = LightPos.coords - intersToScene.coords;
//    float dis = dotProduct(LightDir, LightDir);
//    Vector3f LightDirNormal = LightDir.normalized();
//    Ray rayToLight(intersToScene.coords, LightDirNormal);
//    Intersection interToLight = Scene::intersect(rayToLight);
//    auto f_r = intersToScene.m->eval(ray.direction, LightDirNormal, intersToScene.normal);
//    if (interToLight.distance - LightDir.norm() > -0.005)
//    {
//
//        L_dir = LightPos.emit * f_r * dotProduct(LightDirNormal, intersToScene.normal) * dotProduct(-LightDirNormal, LightPos.normal) / dis / lightpdf;
//    }
//
//    //Calculate the Intersection from point to point in order to calculate indirect Color
//    if (get_random_float() > RussianRoulette)
//        return L_dir;
//
//    Vector3f wi = intersToScene.m->sample(ray.direction, intersToScene.normal).normalized();
//    //Ray indirRay = Ray(intersToScene.coords, wi);
//    Ray indirRay(intersToScene.coords, wi);
//    Intersection intersToPoint = Scene::intersect(indirRay);
//    if (intersToPoint.happened && !intersToPoint.m->hasEmission())
//    {
//        float pdf = intersToScene.m->pdf(ray.direction, wi, intersToScene.normal);
//        L_indir = castRay(indirRay, depth + 1) * intersToScene.m->eval(ray.direction, wi, intersToScene.normal) * dotProduct(wi, intersToScene.normal) / (RussianRoulette / pdf);
//    }
//
//    return L_dir + L_indir;
//}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    // ray is the wo_ray
    // p_inter is the intersection between ray and object plane
    // x_inter is the intersection between ray and light plane
    // ray from p to x is ws_ray


    Intersection p_inter = Scene::intersect(ray);//
    if (!p_inter.happened) {
        return Vector3f();
    }
    if (p_inter.m->hasEmission()) {
        return p_inter.m->getEmission();
    }

    //float EPLISON = 0.0001;
    float EPLISON = 0.005;
    Vector3f l_dir;
    Vector3f l_indir;

    // sampleLight(inter, pdf_light)
    Intersection x_inter;
    float pdf_light = 0.0f;
    sampleLight(x_inter, pdf_light);

    // Get x, ws, NN, emit from inter
    Vector3f p = p_inter.coords;
    Vector3f x = x_inter.coords;
    Vector3f ws_dir = (x - p).normalized();//入射光线单位向量
    float ws_distance = (x - p).norm();//光源与反射点距离
    Vector3f N = p_inter.normal.normalized();//交点法线
    Vector3f NN = x_inter.normal.normalized();//光源采样点法线
    Vector3f emit = x_inter.emit;//光源的Irradiance

    // Shoot a ray from p to x
    Ray ws_ray(p, ws_dir);
    Intersection ws_ray_inter = intersect(ws_ray);//反射点发出的光线与物体的交点
    // If the ray is not blocked in the middle，得到此点此个角度的radiance
    if (ws_ray_inter.distance - ws_distance > -EPLISON) {
        l_dir = emit * p_inter.m->eval(ray.direction, ws_ray.direction, N)
            * dotProduct(ws_ray.direction, N)
            * dotProduct(-ws_ray.direction, NN)
            / std::pow(ws_distance, 2)
            / pdf_light;
    }

    // Test Russian Roulette with probability RussianRoulette
    if (get_random_float() > RussianRoulette) {
        return l_dir;
    }

    l_indir = 0.0;

    Vector3f wi_dir = p_inter.m->sample(ray.direction, N).normalized();
    Ray wi_ray(p_inter.coords, wi_dir);
    // If ray r hit a non-emitting object at q
    Intersection wi_inter = intersect(wi_ray);
    if (wi_inter.happened && (!wi_inter.m->hasEmission())) {
        l_indir = castRay(wi_ray, depth + 1) * p_inter.m->eval(ray.direction, wi_ray.direction, N)
            * dotProduct(wi_ray.direction, N)
            / p_inter.m->pdf(ray.direction, wi_ray.direction, N)
            / RussianRoulette;
    }

    return l_dir + l_indir;
}