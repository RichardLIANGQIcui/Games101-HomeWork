#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

inline float deg2rad(const float& deg)//将角度转换为弧度制
{
    return deg * M_PI / 180.0;
}

//计算反射方向，文章参考https://blog.csdn.net/qq_41835314/article/details/124969379?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167903433116782427491291%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=167903433116782427491291&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-4-124969379-null-null.142^v74^insert_down38,201^v4^add_ask,239^v2^insert_chatgpt&utm_term=games101%E4%BD%9C%E4%B8%9A5%E9%94%99%E8%AF%AF&spm=1018.2226.3001.4187
//这里具体也可以画图推导
Vector3f reflect(const Vector3f& I, const Vector3f& N)
{
    return I - 2 * dotProduct(I, N) * N;
}

//计算折射,参考文章：https://blog.csdn.net/Motarookie/article/details/122425896
// 斯内尔定律(Snell’s Law)： 给出了描述光从一种介质传播到另一种介质时是如何折射的方程：
// ηisinθi = ηtsinθt;sin2θ + cos2θ = 1
//必须知道两种材质的折射率η1，η2，和入射角度θi，然后能得到θt
//折射向量的公式T=a*(I+c1*N)-N*c2;
//a=etai/etat,c1=N点乘I,c2=sqrt(1-a*a*(1-c1*c1));
Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior)//https://blog.csdn.net/MASILEJFOAISEGJIAE/article/details/104435265/
{
    //clamp是个区间限定函数，在这里其实就是把cosi的值限定在了[-1,1]之间
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    Vector3f n = N;
    //如果入射光从介质1(etai)->介质2(etat)，则夹角>90°；
   //如果入射光从介质2->介质1，则夹角<90,N也要反过来，折射率之比也需要换一下
    if (cosi < 0) { cosi = -cosi; }
    else { std::swap(etai, etat); n = -N; }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

//计算菲涅尔等式,I是入射方向，N是交点的法线，ior是材料的折射系数,计算反射光占比
//反射率取决于入射角度，入射光与法线的夹角越大，反射的能量越多
float fresnel(const Vector3f& I, const Vector3f& N, const float& ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0) { std::swap(etai, etat); }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // Total internal reflection//这里考虑了全内反射角
    if (sint >= 1) {
        return 1;
    }
    else {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}

//有了std::optional<T>，我们可以：
//很轻松的区分到底是有返回值还是没有，而不是用傻傻的"magic value"；
//在不采用异常机制的情况下，反映一个 no - value 的 case；毕竟异常机制太兴师动众了，尤其是在不那么“异常”的时刻；
//对我们的类进行更好的封装，避免暴露过多的实现接口给调用者，尤其是在这些接口可能导致错误的前提下。
//最后optional的引用内部变量是用->,而不是"."

//写代码时，有时候想要返回某种类型的对象，也就是说，我们可以有某个类型的值，也可以没有任何值。
//需要有一种方法来模拟类似指针的定义。c++17给出了std::optional<>来定义这样的对象，
//只是包含对象的内部内存加上一个布尔标志，可以在条件语句里充当判断作用，
//比如这里的payload，如果光线与场景中物体没有交点，就直接返回fl


//进行射线求交运算，并保存求交点的一些属性
std::optional<hit_payload> trace(
    const Vector3f& orig, const Vector3f& dir,
    const std::vector<std::unique_ptr<Object> >& objects)
{
    float tNear = kInfinity;
    std::optional<hit_payload> payload;
    //遍历场景中的物体，找出与光线相交的最近的一个物体的属性
    for (const auto& object : objects)//注意这里用auto遍历时，由于对象是个引用，所以auto后还得加&
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        //如果当前的tNearK>tNear,就说明该物体被挡住，不需要进行存储
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)
        {
            payload.emplace();//就地构造一个hit_payload对象
            payload->hit_obj = object.get();//智能指针调用get（）函数获取封装的对象指针，即Object*
            payload->tNear = tNearK;
            payload->index = indexK;//存的是三角形的索引
            payload->uv = uvK;//存的重心坐标
            tNear = tNearK;
        }
    }

    return payload;
}

//通过光线投射算法计算物体表面颜色
Vector3f castRay(
    const Vector3f& orig, const Vector3f& dir, const Scene& scene,
    int depth)//depth表示递归调用的次数，当超过maxDepth时就返回，避免一直递归下去
{
    if (depth > scene.maxDepth) {
        return Vector3f(0.0, 0.0, 0.0);
    }

    Vector3f hitColor = scene.backgroundColor;//如果以下条件不成立，就返回背景色
    if (auto payload = trace(orig, dir, scene.get_objects()); payload)
    {
        Vector3f hitPoint = orig + dir * payload->tNear;//得到交点位置
        Vector3f N; // normal
        Vector2f st; // st coordinates
        //通过getSurfaceProperties获取表面法线，对于三角形网格还额外获取一个重心坐标
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st);
        switch (payload->hit_obj->materialType) //根据不同的材质使用不同的计算方法
        {
        case REFLECTION_AND_REFRACTION:
        {
            Vector3f reflectionDirection = normalize(reflect(dir, N));
            Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));
           
            //对于epsilon的使用有两种说法
            // 说法一：
            //这里的epsilon是偏移值，因为计算机精度的问题，直接使用起点会产生噪声值
            //由于精度损失，通过计算得到的相交点坐标往往不会正中理论上的点位置，而是会向外或者向内一点点，
            //为了解决这个问题采用一个小小的偏移值epsilon来解决这个问题
            //由于计算问题会出现误差，如果点在面内就向N方向偏移；如果点在面外就向-N方向偏移,
            //说法二：(其实说法二会更好理解）
            //是因为之后可能会继续判断射线是否与物体有接触，所以要加上或减取一个很小的值，
            //防止有接触到当前点,反射的话一般往上移一点点，而折射的话一般往物体内移一点点
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                hitPoint - N * scene.epsilon :
                hitPoint + N * scene.epsilon;
            Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                hitPoint - N * scene.epsilon :
                hitPoint + N * scene.epsilon;
            Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
            Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
           
            //利用菲涅尔等式计算反射光的占比，透射光的占比=1-反射光的占比
            float kr = fresnel(dir, N, payload->hit_obj->ior);
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);//反射光加折射光是最终颜色
            break;
        }
        case REFLECTION:
        {
            float kr = fresnel(dir, N, payload->hit_obj->ior);
            Vector3f reflectionDirection = reflect(dir, N);

            //由于计算问题会出现误差，如果点在面内就向N方向偏移；如果点在面外就向-N方向偏移
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                hitPoint + N * scene.epsilon :
                hitPoint - N * scene.epsilon;

            //接着递归使用castRay()把计算的反射起点Orig和方向Dir带入，并*反射光线占比kr，
            //继续跟踪折射和反射射线
            hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
            break;
        }
        default:
        {
            // [comment]
            // We use the Phong illumation model int the default case. The phong model
            // is composed of a diffuse and a specular reflection component.
            // [/comment]
            Vector3f lightAmt = 0, specularColor = 0;
            Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
                hitPoint + N * scene.epsilon :
                hitPoint - N * scene.epsilon;
            // [comment]
            // Loop over all lights in the scene and sum their contribution up
            // We also apply the lambert cosine law
            // [/comment]
            for (auto& light : scene.get_lights()) {
                Vector3f lightDir = light->position - hitPoint;
                // square of the distance between hitPoint and the light
                float lightDistance2 = dotProduct(lightDir, lightDir);
                lightDir = normalize(lightDir);
                float LdotN = std::max(0.f, dotProduct(lightDir, N));
                //hitpoint在阴影里->inshadow=true->lightAmt=0(无环境光)
                //hitpoint不在阴影里->inshaow=false->lightAmt+=I/r²*max(0, cos<l,n>)
                auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);

                lightAmt += inShadow ? 0 : light->intensity * LdotN;
                Vector3f reflectionDirection = reflect(-lightDir, N);//注意这里加符号才是入射方向，符合函数的定义

                specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                    payload->hit_obj->specularExponent) * light->intensity;
            }
            //这里使用 evalDiffuseColor(st) 渲染出地板的效果，具体什么原理还未搞懂
            hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks;
            break;
        }
        }
    }

    return hitColor;
}

//计算主射线方向，这个方向是从相机出发(0,0,0)->像素中心(x,y,z)，
//把pixel从栅格空间->世界空间，其实就相当于做了一次逆向光栅化
//光栅化的空间转换如下：
//Step.1 —— world space -> screen space
//Step.2 —— screen space -> NDC space（归一化设备坐标）标准立方体中
// Step.3 —— NDC space -> raster space//最后转换到图像
//逆光栅化的过程就是上述步骤反着来
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;

    // Use this variable as the eye position to start your rays.
    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            //找到这些像素在栅格空间（raster space）中的坐标
        //与在世界空间(world space)中表达的相同像素的坐标之间的关系
        //1、栅格空间到NDC空间(-1~1)的转换
        //2、NDC空间到屏幕空间的转换
        //3、屏幕空间到世界空间的转换
            float x, y;
            //（1）先将范围映射到0~1
            float Screenx = (i + 0.5) / (float)scene.width;//注意要转浮点型
            float Screeny = (j + 0.5) / (float)scene.height;

            //（2）然后映射到（-1~1），因为世界坐标中相机对应图像原点，每个像素的位置实际上是存在负数的
            float NDCx = 2 * Screenx - 1;
            //对于位于x轴上方的像素是负的，对于位于x轴下方的像素是正的(而它应该是相反的)
            //可以理解为像素从左上角遍历，x是从-1~1变化，而y是从1~-1发生变化，所以这里y是相反的
            float NDCy = 1 - 2 * Screeny;

            //(3)以上的算法都假设了图像是正方形，而实际是长方形，所以这里再次用到宽高比对x方向作纠正，为什么是x方向这个自己画图知道
            float Camerax = NDCx * imageAspectRatio;

            //这里考虑视角的影响，可以理解为通过视角影响会对屏幕造成一个缩放的效果，
            //这个操作改变了我们看到的场景的多少，相当于放大(当视野减小时，我们看到的场景更少)
            // 和缩小(当视野增大时，我们看到的场景更多)
            //所以这里xy方向都要再乘上tan,得到摄像机空间坐标系下的点
            Camerax = NDCx * scale;
            float Cameray = NDCy * scale;

             x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
            y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = Vector3f(x, y, -1);
            dir = normalize(dir);
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);//这个功能是生成界面的进度条
    }

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);//fwrite() 函数将3个对象写入给定的输出流，每个对象的大小为 1个字节。
    }
    fclose(fp);
}
