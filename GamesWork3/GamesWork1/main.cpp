//ͨ����ҵ����������Ⱦ���������ܽ����˵��Ⱦ��ҵ�е�ģ�������������
//׼��������Ҫ����һ��ģ����Ҫ��Щ�ࣿ��դ���ࡢ�����ࡢ��ɫ���ࡢ�������ࡢģ�ͼ�����
//��main������ʼ������Ⱦ��һ������
//��һ������ģ�����ݵ�����������У���������й���һ�����������������������ݣ�����λ�á����ߡ��������꣩
//�ڶ�����ģ��������ÿ�������β�ѹ��һ�������д��������������ڹ�դ��
//���������Ǵ�����դ���������������ھ���ɫ���������ɫ���ǰ�������ʾ����Ļ�ϵ���ɫ���뵽֡�����У�
// ����ͨ��OpenCV��ȡ֡��������ݳ���
// ���ȴ����դ������õı�þ����ڹ�դ�����н�����ռ�������õ���Ļ�ռ�
// Ȼ���ڹ�դ�����л��ж�ÿ�����ص��Ƿ����������ڣ�ͬʱ������Ȳ���
// �ڵĻ��ͽ�����ɫ������һ�����ص�����Զ�ͨ����ֵ�������������ֵ���㣬Ȼ�����Щ��ֵ���������Դ��뵽��ɫ���н��й��մ���
//���Ĳ����մ����У��õ���ɫ�������м���������ͼ�ռ��½��еģ���ͬ���ܵ���ɫ������ʵ�ֲ��õ���ɫ��
//��ҵ�еķ�����ɫ�����ѷ��ߵ�ֵ����ɫֵ�����������
//phong��ɫ����ʹ�ò��ַ�ģ����ɫ�������˻����⡢�������͸߹��Ӱ�죬�����kd�ǵ���������Ĳ�ֵ��ɫ
//bump��ɫ������͹��ͼ��������Ҫ���˽ⷨ�ߺ����߿ռ䣨https://zhuanlan.zhihu.com/p/144357517������ƪ������ϸ������ΪʲôҪʹ�����߿ռ��еķ��߶���������ռ�
//bump��ԭ�����ͨ�����������ȡ��͹��ͼ�ĸ߶���Ϣ�������߿ռ��µķ��ߡ�Ȼ�������TBN����õ���������µķ���
//λ����ͼ���Ƕ���λ�÷����˸ı䣬���ڰ�͹��ͼ�Ļ����ϵõ����µķ��ߣ�λ��Ҳ�Ƿ����˱仯�����µķ��ߺ�λ�ý��й���ģ�ͼ���




	//�ܽ���ҵ��Խ����һЩ����
//1��ģ�����ݶ�ȡ·�������пո񣬷���һƬ�ڡ�
//2��Texture�е�getColor��������Բ���uv�������ƣ������ֵ����С��1�����ܵ���1�������������bump��displacement�Ƚ��ʱ����
//3������������ҵ2�ĳ��������й�դ��ʱ����Ⱦ����ģ�ͻ����һ�����������α߿�������Ϊ��ҵ2�Ŀ��δ�����������
//4����ʹ�ó�������������ߺ����Ų��������ӣ��������߿�ȷʵ���˺ܶ࣬����ʹ�������ǵ�256����Ч����Ȼ���ںܶ�С�����������д��������������˵��Ӧ���к��ߣ�ģ���ڲ������к�ɫ��ɫ�Ĳ�����
//�����ҵ���⣬������߸�ÿ�������εĻ���˳���йأ��п����������ȡ���ܱߵĲ�����ܶ඼û����ֵ���ͻ������ɫ
//5����ҵ����ϱ���������⣬����Ӱ���������ʵ�֣�����t.toVector4() �����Ķ���Ѵ��ڶ����w������Ϊ1����w����
//͸��ͶӰ��ú�w��������ֵ����z������෴���������ں���͸��У��ʱ���õ�
//6�����ճ���������Ļ�ռ��н��У�����դ��������õ���Ļ�ռ�����ϵ�µĶ������ݺ���ͼ�ռ�Ķ�������
//һ����������������Ļ�ϻ�ͼ���ڶ��������������ڲ��ַ����ģ�͵ļ��㣬���յõ�����ɫ�Ǿ�����ɫ�������
//����ɫ���������õ������������ֶ���view�ռ������µ�



#include <opencv2/opencv.hpp>
#include <iostream>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

using namespace std;
using namespace Eigen;
using namespace rst;


Matrix4f get_view_matrix(Vector3f eye_pos)
{
	//����view�����õ�ԭ����ǽ������е�������ƶ���ԭ�㣬ͬʱ����ϵ�����ϵ��Ӧ������������ת
	//��������Ҳ�������һ���ƶ�
	//Ȼ�������ת���������ԭ���Ƕ�����ϵ��ת���������ϵ�ı�õ������Ҫ��ľ��󣬶����ھ�����
	//�����������Ը�������������Ǹþ����ת�þ���
	//����û������ת����������Ϊ�������ʱ������Լ�������ϵ���Ǻ�������һ�µ�
	Matrix4f view = Matrix4f::Identity();//Identity()��ʾ�õ�λ����Ա������г�ʼ��

	Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;

	view = translate * view;
	
	return view;
}


Matrix4f get_model_matrix(float angle)
{
	Matrix4f rotation;
	angle = angle * MY_PI/180.f;
	rotation << cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1;

	Matrix4f scale;
	scale << 2.5, 0, 0, 0,
			0, 2.5, 0, 0,
			0, 0, 2.5, 0,
			0, 0, 0, 1;

	Matrix4f translate;
	translate << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;


	return translate*rotation*scale;
}

Matrix4f toMatrix4f(Matrix3f m)
{
	Matrix4f tRet = Matrix4f::Zero();
	tRet.block<3, 3>(0, 0) = m;
	Vector4f a{0,0,0,1};
	tRet.row(3) = a;
	return tRet;
}

//����������ת����ת�������ﺯ��ʵ���ϸ�ͨ�ã����õ����޵����˹��ʽ
//cosa*I + (1-cosa)*(axis*axis.transpose()) + sina*nhat,I�ǵ�λ����axis����
Matrix4f get_rotation(Vector3f axis, float angle)
{
	//����������ʽһ�������ÿ������������I
	Matrix3f I = Matrix3f::Identity();
	//����Ƕ�
	float a = angle / 180.0 * MY_PI;
	float cosa = cos(a);
	float sina = sin(a);
	//������й�һ��
	axis = axis.normalized();
	//��nhat����
	Matrix3f nhat;
	nhat << 0, -axis.z(), axis.y(),
		axis.z(), 0, -axis.x(),
		-axis.y(), axis.x(), 0;

	return toMatrix4f(cosa * I + (1 - cosa) * (axis * axis.transpose()) + sina * nhat);

}

//��ֱ�Ŀ��ӽǶ�fov,�����aspect
Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	Matrix4f projection = Matrix4f::Identity();

	//ͶӰ�����ԭ���ǽ��ռ�ѹ���ɽ�ƽ���С�������壨ÿ���߳�Ϊ2������������ͶӰ

	//��һ������������ͶӰ����ѹ������������ľ���
	Matrix4f pToO;
	pToO << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -zNear * zFar, 0, 0, 1, 0;

	//������ͶӰ
	//������Ҫ��һЩ����������Ĭ���˳��������ֱ��ˮƽ��������Ķ���ԭ�㣬λ��ʱֻ�ı�Զ��

	//Զ������ı߳�
	float a = zNear - zFar;//������Ĭ�ϳ�z�Ḻ���򿴣���������zNear - zFar

	//��ֱ����߳�
	float fovY = eye_fov / 180.0 * MY_PI;
	float b = -abs(zNear) * tan(fovY / 2)*2;//��Ϊ��ܽ�z�����˷�ת����������Ҫ�Ӹ���,�������ͼƬ�ᵹ��

	//ˮƽ����߳�
	float c = aspect_ratio * b;

	//����ͶӰ����λ��������
	Matrix4f scale,trans;
	scale << 2.0 / c, 0, 0, 0,
		0, 2.0 / b, 0, 0,
		0, 0, 2 / a, 0,
		0, 0, 0, 1;
	trans << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, -(zNear + zFar) / 2,
		0, 0, 0, 1;
	
	projection = scale * trans * pToO;
	
	return projection;
}

Vector3f vertex_shader(const vertex_shader_payload& payload)
{
	return payload.position;
}

//����ʾ���ߵ���ɫ
Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
	//�����һ�д����ǽ����������ɱ����[-1,1]�ķ�Χת����[0,1]֮�䣬color�е�rgb�洢���Ƿ��ߵ�xyz
	Vector3f return_color = (payload.normal.head<3>().normalized() + Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
	Vector3f result;
	result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
	return result;
}

//��������,����������������������ط�������,���ǵ�λ����
//����vec�����ĳ���Ϊ1��1*costheta*2��ʾ����ͷ����������ɵ����εĶԽ��߳��ȣ��ٳ���axis������������������ȥvec�ɵ÷�������
static Vector3f reflect(const Vector3f& vec, const Vector3f& axis)
{
	auto costheta = vec.dot(axis);
	return (2 * costheta * axis - vec).normalized();
}

//�����Դ������λ�ú�ǿ��
struct light
{
	Vector3f position;
	Vector3f intensity;
};

Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
	Vector3f ka = Vector3f(0.005, 0.005, 0.005);
	Vector3f kd = payload.color;
	Vector3f ks = Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20,20,20},{500,500,500} };
	auto l2 = light{ {-20,20,0},{500,500,500} };

	vector<light> lights = { l1,l2 };
	Vector3f amb_light_intensity{ 10,10,10 };
	Vector3f eye_pos{ 0,0,10 };

	float p = 150;

	Vector3f color = payload.color;
	Vector3f point = payload.view_pos;
	Vector3f normal = payload.normal;

	Vector3f result_color = { 0,0,0 };
	for (auto& light : lights)
	{//�ٰ������Ϊһ����׼�������շ�������۷����������Ǵ���������ģ��ʶ���-point
		//�������h���ǹ�һ�����light_dir��view_dir���֮��õ��ķ���֮�����ٽ���һ�ι�һ��
		Vector3f lightDir = light.position-point;
		float distance = lightDir.norm();
		lightDir = lightDir.normalized();
		Vector3f viewDir = (eye_pos - point).normalized();
		Vector3f halfDir = (lightDir + viewDir).normalized();
		
		//ע�����������������ĳ˷�Ҫ�õ�cwiseProduct��������
		auto diffuse_light = kd.cwiseProduct(light.intensity / (distance * distance)) * max(0.0f, normal.dot(lightDir));
		auto specular_light = ks.cwiseProduct(light.intensity / (distance * distance)) * pow(max(0.0f, normal.dot(halfDir)), p);
		auto ambient_light = ka.cwiseProduct(amb_light_intensity);

		result_color += (diffuse_light + specular_light+ ambient_light);
	}
	

	return result_color * 255.f;
}

Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
	Vector3f return_color = { 0,0,0 };
	if (payload.texture)
	{
		return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
	}
	Vector3f texture_color;
	texture_color << return_color.x(), return_color.y(), return_color.z();

	//���ַ�ģ�ͼ�������������
	Vector3f ka = Vector3f(0.005, 0.005, 0.005);
	Vector3f kd = texture_color / 255.f;
	Vector3f ks = Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20,20,20},{500,500,500} };
	auto l2 = light{ {-20,20,0},{500,500,500} };

	vector<light> lights = { l1,l2 };
	Vector3f amb_light_intensity{ 10,10,10 };
	Vector3f eye_pos{ 0,0,10 };

	float p = 150;

	Vector3f color = texture_color;
	Vector3f point = payload.view_pos;
	Vector3f normal = payload.normal.normalized();

	Vector3f result_color = { 0,0,0 };

	for (auto& light : lights)
	{
		Vector3f lightDir = light.position - point;
		float distance = lightDir.norm();
		lightDir = lightDir.normalized();
		Vector3f viewDir = (eye_pos - point).normalized();
		Vector3f halfDir = (lightDir + viewDir).normalized();

		//ע�����������������ĳ˷�Ҫ�õ�cwiseProduct��������
		auto diffuse_light = kd.cwiseProduct(light.intensity / (distance * distance)) * max(0.0f, normal.dot(lightDir));
		auto specular_light = ks.cwiseProduct(light.intensity / (distance * distance)) * pow(max(0.0f, normal.dot(halfDir)), p);

		auto ambient_light = ka.cwiseProduct(amb_light_intensity);

		result_color += (diffuse_light + specular_light + ambient_light);
	}


	return result_color * 255.f;
}

Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
	Vector3f ka = Vector3f(0.005, 0.005, 0.005);
	Vector3f kd = payload.color;
	Vector3f kp = Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20,20,20},{500,500,500} };
	auto l2 = light{ {-20,20,0},{500,500,500} };

	vector<light> lights = { l1,l2 };
	Vector3f amb_light_intensity{ 10,10,10 };
	Vector3f eye_pos{ 0,0,10 };

	float p = 150;

	Vector3f color = payload.color;
	Vector3f point = payload.view_pos;
	Vector3f normal = payload.normal;

	float kh = 0.2, kn = 0.1;//��ʾ�����߶���ʵ�����Ӱ��̶�

	// TODO: Implement displacement mapping here
	Vector3f n = normal;
	Vector3f t = { n.x() * n.y() / sqrt(n.x() * n.x()+n.z()*n.z()),sqrt(n.x() * n.x() + n.z() * n.z()),n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()) };
	t = t.normalized();
	Vector3f b = n.cross(t);
	b = b.normalized();
	Matrix3f TBN ;//ע������TBN�ĸ�ֵ
	TBN << t,b,n ;

	float u = payload.tex_coords.x();
	float v = payload.tex_coords.y();
	float w = payload.texture->width;
	float h = payload.texture->height;

	//ע�����������getColor������u+1.0/w�����ǣ�u+1,������Ϊ����Ҫ��ϸ����getColor����
	//�ú�������������������������Ŀ�Ⱥ͸߶ȣ����յõ�u*width+1,,��Ӧ�������ƶ�һ����λ
	//��������Ĳ���Ҫ+1.0/w
	float dU = kh * kn * (payload.texture->getColor(u+1.0/w,v).norm()-payload.texture->getColor(u,v).norm());

	float dV = kh * kn * (payload.texture->getColor(u, v+1.0/h).norm() - payload.texture->getColor(u, v).norm());

	Vector3f ln = Vector3f(-dU, -dV, 1);//ͨ����ȡ������ͼ�ĸ߶ȼ�����ķ��ߣ��÷���λ�����߿ռ���

	Vector3f result_color = { 0,0,0 };

	result_color = (TBN * ln).normalized();//ͨ�����TBN����ת������������£�������ɫ

	return result_color * 255.f;
}

Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
	Vector3f ka = Vector3f(0.005, 0.005, 0.005);
	Vector3f kd = payload.color;
	Vector3f ks = Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20,20,20},{500,500,500} };
	auto l2 = light{ {-20,20,0},{500,500,500} };

	vector<light> lights = { l1,l2 };
	Vector3f amb_light_intensity{ 10,10,10 };
	Vector3f eye_pos{ 0,0,10 };

	float p = 150;

	Vector3f color = payload.color;
	Vector3f point = payload.view_pos;
	Vector3f normal = payload.normal;

	float kh = 0.2, kn = 0.1;

	// TODO: Implement displacement mapping here
	Vector3f n = normal;
	Vector3f t = { n.x() * n.y() / sqrt(n.x() * n.x()+n.z() * n.z()),sqrt(n.x() * n.x()),n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()) };
	t = t.normalized();
	Vector3f b = n.cross(t);
	b = b.normalized();
	Matrix3f TBN;
	TBN << t, b, n;

	float u = payload.tex_coords.x();
	float v = payload.tex_coords.y();
	float w = payload.texture->width;
	float h = payload.texture->height;

	float dU = kh * kn * (payload.texture->getColor(u + 1.0 / w, v).norm() - payload.texture->getColor(u, v).norm());

	float dV = kh * kn * (payload.texture->getColor(u, v + 1.0 / h).norm() - payload.texture->getColor(u, v).norm());

	Vector3f ln = Vector3f(-dU, -dV, 1);

	point = point + kn * n * payload.texture->getColor(u, v).norm();
	
	normal = (TBN * ln).normalized();

	Vector3f result_color = { 0,0,0 };
	for (auto& light : lights)
	{
		Vector3f lightDir = light.position - point;
		float distance = lightDir.norm();
		lightDir = lightDir.normalized();
		Vector3f viewDir = (eye_pos - point).normalized();
		Vector3f halfDir = (lightDir + viewDir).normalized();

		//ע�����������������ĳ˷�Ҫ�õ�cwiseProduct��������
		auto diffuse_light = kd.cwiseProduct(light.intensity / (distance * distance)) * max(0.0f, normal.dot(lightDir));
		auto specular_light = ks.cwiseProduct(light.intensity / (distance * distance)) * pow(max(0.0f, normal.dot(halfDir)), p);

		auto ambient_light = ka.cwiseProduct(amb_light_intensity);

		result_color += (diffuse_light + specular_light + ambient_light);
	}


	return result_color * 255.f;
}



int main(int argc, const char** argv)
{

	vector<Triangle*> TriangleList;

	float angle = 140;
	bool command_line = false;

	string filename = "output.png";//Ĭ������ļ���
	objl::Loader Loader;
	string obj_path = "D:/GAMES/HomeWork/3/Assignment3/Code/models/spot/";

	//load.objFile
	//�����и��ӣ����Ƕ�ȡģ�����ݣ�vs��ʱ����Զ�������ľ���·��������·�����ֺܶ�ո������һƬ��
	bool loadout = Loader.LoadFile("D:/GAMES/HomeWork/3/Assignment3/Code/models/spot/spot_triangulated_good.obj");
	for (auto mesh : Loader.LoadedMeshes)
	{
		for (int i = 0;i < mesh.Vertices.size();i+=3)//ע�����������������Ķ�������ÿ3������һ�������Σ�������i+=3
		{
			Triangle* t = new Triangle();
			for (int j = 0;j < 3;j++)
			{
				t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
				t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
				t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
			
			}
			TriangleList.push_back(t);
		}
	}

	rasterizer r(700, 700);//�趨��ɺ���ӿ�

	auto texture_path = "hmap.jpg";
	r.set_texture(Texture(obj_path + texture_path));

	function<Vector3f(fragment_shader_payload)> active_shader = displacement_fragment_shader;

	if (argc >= 2)
	{
		command_line = true;
		filename = string(argv[1]);

		if (argc == 3 && std::string(argv[2]) == "texture")
		{
			cout << "Rasterizing using the texture shader\n";
			active_shader = texture_fragment_shader;
			texture_path = "spot_texture.png";
			r.set_texture(Texture(obj_path + texture_path));
		}
		else if (argc == 3 && string(argv[2]) == "normal")
		{
			cout<< "Rasterizing using the normal shader\n";
			active_shader = normal_fragment_shader;
		}
		else if (argc == 3 && string(argv[2]) == "phong")
		{
			cout << "Rasterizing using the phong shader\n";
			active_shader = phong_fragment_shader;
		}
		else if (argc == 3 && string(argv[2]) == "bump")
		{
			cout << "Rasterizing using the bump shader\n";
			active_shader = bump_fragment_shader;
		}
		else if (argc == 3 && string(argv[2]) == "displacement")
		{
			cout << "Rasterizing using the displacement shader\n";
			active_shader = displacement_fragment_shader;
		}
	}

	Vector3f eye_pos = { 0,0,10 };

	//����ͬ����ɫ�������դ���������ڴ��������ɫ
	r.set_vertex_shader(vertex_shader);
	r.set_fragment_shader(active_shader);

	int key = 0;
	int frame_count = 0;

	if (command_line)//��������п��ر�־Ϊ������һ��if������Ϊ��Ӧ�������д���Ĳ����������ʼ�ǶȺ��ļ�����
	{
		r.clear(Buffers::Color | Buffers::Depth);//��ʼ��֡�������Ȼ��棨������ҵ������ҵֻ�漰һ��ͼ�Σ����Բ��漰��ȣ����Բ��ܣ�

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
		
		//��դ��
		r.draw(TriangleList);

		//����һ��mat�����ڴ洢ͼ�����ݣ�CV_32FC3 32��ʾһ�����ص�ռ32λ F��ʾ������ C3��ʾRGB��ɫͼ��(��ͨ��)
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//data()����һ��ָ���ڴ������ָ��
		image.convertTo(image, CV_8UC3, 1.0f);//��һ�������һ����������ת������һ����������
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);//��rgb�ռ�ת����OpenCV��bgr�ռ�

		cv::imwrite(filename, image);//����ͼ���ļ���

		return 0;
	}

	while (key != 27)////ֻҪû�м�⵽����ESC��ѭ��(ESC��ASCII����27)
	{
		r.clear(Buffers::Color | Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(TriangleList);
		
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		cv::imshow("image", image);//�ú�����ͼ�����ض��Ĵ�����ʾ
		cv::imwrite(filename, image);
		key = cv::waitKey(10);// ÿ��10msˢ��һ��ͼ��


		if (key == 'a')
		{
			angle -= 0.1;
		}
		else if (key == 'd')
		{
			angle += 0.1;
		}

	}

	return 0;
}

