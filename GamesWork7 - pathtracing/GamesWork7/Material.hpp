//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

using namespace std;

enum MaterialType { DIFFUSE, Microfacet};

class Material {
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f& I, const Vector3f& N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; }//���С��0���������ȷ�ģ�ֻ��Ҫ�������ʱcosi����ֵ
        else { std::swap(etai, etat); n = -N; }//�������0���Ǿ��Ǵӷ��������䣬���������ʺͷ��߶��ý���
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }


    //������Ϊ��0��0��1����localRayת��Ϊ����ΪN��������
      // �������� -> ��������
    // �����ϵ������Ǿֲ�����ϵ�µ�a����Ϊ��������N�����ϵ�(0,0,1)��������Ҫת��
    // ���оֲ�����ϵ�£�a.x,a.y,a.z�ڵ����������໥��ֱ������a.z�ķ������N�ķ���
    // ���裺
    // 1.�ٶ���B,C������λ������B,C��N�ó���B,C,N������ֱ����B,C,N���ǵ�λ����
    // 2.��a.x,a.y,a.z��ֵ�ֱ�ȥ��B,C,N ->����B,C,N���������ն�Ӧֵ�ı����Ŵ�
    // 3.�ٽ��õ�������������ӣ����������������г���ʾ��ԭʼ��a
  
    Vector3f toWorld(const Vector3f& a, const Vector3f& N) {
        Vector3f B, C;
        //����������жϣ�Ӧ����Ϊ�˱�����ַ�ĸΪ0�����
        if (std::fabs(N.x) > std::fabs(N.y)) {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            //���ǾͲ���y����£���֪xһ����ֵ��
            //��֤�������㣺1.��x,z������ʾ��һ����λ������2.��Ҫ��N��ֱ
            //CΪ��x��zƽ����N������N��ֱ�ĵ�λ������Ϊʲô�ǵ�λ�ʹ�ֱ�ģ�
            //ֻҪ���������ķ����������֪������ֱ�Ļ����Ϊ0����λ��������sqrt��x*x+y*y+z*z)=1
            C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f& wi, const Vector3f& N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);

    float DistributionGGX(Vector3f N, Vector3f H, float roughness)
    {
        float a = roughness * roughness;
        float a2 = a * a;
        float NdotH = max(dotProduct(N, H), 0.0f);
        float NdotH2 = NdotH * NdotH;

        float nom = a2;
        float denom = (NdotH2 * (a2 - 1.0) + 1);
        denom = M_PI * denom * denom;

        return nom / max(denom, 0.0000001f);
    }

    //k = r*r/8;,r=(roughness+1.0)
    float GeometrySchlickGGX(float NdotV, float k)
    {
        float nom = NdotV;
        float denom = NdotV * (1.0 - k) + k;

        return nom / denom;
    }

    float GeometrySmith(Vector3f N, Vector3f V, Vector3f L, float roughness)
    {
        float r = roughness + 1.0;
        float k = (r * r) / 8.0;
        float NdotV = max(dotProduct(N, V), 0.0f);
        float NdotL = max(dotProduct(N, L), 0.0f);

        float ggx2 = GeometrySchlickGGX(NdotV, k);
        float ggx1 = GeometrySchlickGGX(NdotL, k);

        return ggx1 * ggx2;
    }

    void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) { std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        //���ݿ�����˵��sint>1�ͻᷢ��ȫ�ڽǷ��䣬������û������
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }


};

Material::Material(MaterialType t, Vector3f e) {
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType() { return m_type; }
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}

//�ڰ����Ͼ��Ȳ���������Ĭ�ϵİ������ھֲ�����ϵ�ҷ��߷���Ϊ��0,0,1���ϵģ������Ƿ��߷���ΪN������ϵ�����Ժ�����Ҫת��
Vector3f Material::sample(const Vector3f& wi, const Vector3f& N) {
    switch (m_type) {
    case DIFFUSE:
    case Microfacet:
    {
        // uniform sample on the hemisphere
        float x_1 = get_random_float(), x_2 = get_random_float();
        float z = std::fabs(1.0f - 2.0f * x_1);//����������z������
        float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;//�ٶ�һ������Ƕ�
        Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
        return toWorld(localRay, N);//ת������������

        break;
    }
    }
}

float Material::pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
    switch (m_type) {
    case DIFFUSE:
    case Microfacet:
    {
        // uniform sample probability 1 / (2 * PI)
        if (dotProduct(wo, N) > 0.0f)
            return 0.5f / M_PI;
        else
            return 0.0f;
        break;
    }
    }
}



//΢����ģ��BRDF����������;��淴����
//fr =kd*fd+ ks*fs
//fs= (D*G*F)/(4*(N���l��*��N���v����
//fd = c/PI,cΪ������ɫ,cһ��ȡ1����Ϊ1.0/pI
//ksΪ������=F��kdΪ������=1-ks;

Vector3f Material::eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
    switch (m_type) {
        case DIFFUSE://��Է�΢�����㷨
        {
            //����Ҫ����һ���жϣ���N���wo����С��0����Ȼû����
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f)
            {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
            {
                return Vector3f(0.0f);
            }
            break;
        }

        case Microfacet:
        {
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f)
            {
                float roughness = 0.40;

                Vector3f V = -wi;//�������ط���
                Vector3f L = wo;//���䷽��
                Vector3f H = normalize(V + L);

                float D = DistributionGGX(N, H, roughness);

                float G = GeometrySmith(N, V, L, roughness);

                float F;
                float etat = 1.85;
                fresnel(wi, N, etat, F);

                Vector3f nominator = D * F * G;
                float denominator = 4 * max(dotProduct(N, V), 0.0f) * max(dotProduct(N, L), 0.0f);

                Vector3f specular = nominator / max(denominator, 0.0001f);

                float ks_ = F;
                float kd_ = 1 - ks_;

                Vector3f diffuse = 1.0f / M_PI;

                return kd_ * diffuse + ks_ * specular;

            }
            else
            {
                return Vector3f(0.0f);
            }
            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
