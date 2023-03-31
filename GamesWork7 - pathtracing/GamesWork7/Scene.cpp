//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}


//注意这里的objects是包含光源的，可以从main函数中发现所谓光源也被定义为三角形网格，存储到了objects中
//所以对光源的采样实际上也是对一个三角形网格采样，面积求和是针对光源的面积求和
void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {//随机取一个光源三角面
                objects[k]->Sample(pos, pdf);//在这个光源面上随机取一个点
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray& ray,
    const std::vector<Object*>& objects,
    float& tNear, uint32_t& index, Object** hitObject)
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

// Implementation of Path Tracing

//解法二：使用微表面模型BRDF计算,参照文章或者OpenGL：https://blog.csdn.net/weixin_44518102/article/details/122698851?spm=1001.2101.3001.6650.9&utm_medium=distribute.pc_relevant.none-task-blog-2~default~BlogCommendFromBaidu~Rate-9.pc_relevant_default&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~BlogCommendFromBaidu~Rate-9.pc_relevant_default&utm_relevant_index=12
//修改Material类中的eval函数
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir;
    Vector3f L_indir;

    // 从像素发出的光线与物体的交点
    Intersection obj_inter = intersect(ray);
    if (!obj_inter.happened)
        return L_dir;

    // 打到光源
    if (obj_inter.m->hasEmission())
        return obj_inter.m->getEmission();

    // 打到物体
    Vector3f p = obj_inter.coords;
    Material* m = obj_inter.m;
    Vector3f N = obj_inter.normal.normalized();
    Vector3f wo = ray.direction; // 像素到物体的向量

    // 有交点，对光源采样
    float pdf_L = 1.0; //可以不初始化
    Intersection light_inter;
    sampleLight(light_inter, pdf_L);    // 得到光源位置和对光源采样的pdf

    Vector3f x = light_inter.coords;
    Vector3f ws = (x - p).normalized(); //物体到光源
    Vector3f NN = light_inter.normal.normalized();
    Vector3f emit = light_inter.emit;
    float d = (x - p).norm();

    // 再次从光源发出一条光线，判断是否能打到该物体，即中间是否有阻挡
    Ray Obj2Light(p, ws);
    float d2 = intersect(Obj2Light).distance;
    // 是否阻挡，利用距离判断，需注意浮点数的处理
    if (d2 - d > -0.001) {// 做出来图片有横条的原因在这里：这个数字不能过于接近0
        Vector3f eval = m->eval(wo, ws, N); // wo不会用到
        float cos_theta = dotProduct(N, ws);
        float cos_theta_x = dotProduct(NN, -ws);//ws从物体指向光源，与NN的夹角大于180
        L_dir = emit * eval * cos_theta * cos_theta_x / std::pow(d, 2) / pdf_L;
    }

    // L_indir
    float P_RR = get_random_float();
    if (P_RR < RussianRoulette) {
        Vector3f wi = m->sample(wo, N).normalized();
        Ray r(p, wi);
        Intersection inter = intersect(r);
        // 判断打到的物体是否会发光取决于m
        if (inter.happened && !inter.m->hasEmission()) {
            Vector3f eval = m->eval(wo, wi, N);
            float pdf_O = m->pdf(wo, wi, N);
            float cos_theta = dotProduct(wi, N);
            //为了优化图中的白点，这里将pdf作一个优化，避免计算的颜色偏向极限值，即白色
            if (pdf_O > 0.001)
            {
                L_indir = castRay(r, depth + 1) * eval * cos_theta / pdf_O / RussianRoulette;
            }
           
        }
    }
    //4->16min
    return L_dir+L_indir ;

  
}