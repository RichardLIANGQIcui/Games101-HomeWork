//�ҷ���������������ĺ�����ʹ��staic����ؼ����ܽ������ڶ��̵߳Ļ���������һ�����ϣ�
//һ��spp : 256, ��ʹ�ö��̡߳���staic�ؼ��֣���ʱ27hour
//����spp : 256, ʹ�ö��̣߳�������static�ؼ��֣���ʱ3Сʱ
//����spp : 256, ʹ�ö��̺߳�static����ʱ1Сʱ����
// ��������ı���ͨ������Ϊ�ֲ���̬����������ֵ������Ϊ�������õĽ������������
// �������ٴα�����ʱ������ֵ����һ�ε��ý������ֵ
//�ں�������������ʱ�� static �ؼ���ָ������ֻ��ʼ��һ�Σ�����֮����øú���ʱ������״̬
//����ĵ��þͲ����ٷ����ڴ�ռ䣬ֻҪ�ı����������
//��������������ڴ�Ҳ����ʱ�俪���ģ�ʹ��static��֤�˱������������������ڼ�ֻ��һ�����ɣ��������ֵ����

#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

int main(int argc, char** argv)
{
	Scene scene(784, 784);


	Material* red = new Material(DIFFUSE, Vector3f(0.0f));
	red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
	Material* green = new Material(DIFFUSE, Vector3f(0.0f));
	green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
	Material* white = new Material(DIFFUSE, Vector3f(0.0f));
	white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
	Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) + 15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) + 18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
	light->Kd = Vector3f(0.65f);

	MeshTriangle floor("D:/GAMES/HomeWork/7/PA7/Assignment7/models/cornellbox/floor.obj", white);
	MeshTriangle shortbox("D:/GAMES/HomeWork/7/PA7/Assignment7/models/cornellbox/shortbox.obj", white);
	MeshTriangle tallbox("D:/GAMES/HomeWork/7/PA7/Assignment7/models/cornellbox/tallbox.obj", white);
	MeshTriangle left("D:/GAMES/HomeWork/7/PA7/Assignment7/models/cornellbox/left.obj", red);
	MeshTriangle right("D:/GAMES/HomeWork/7/PA7/Assignment7/models/cornellbox/right.obj", green);
	MeshTriangle light_("D:/GAMES/HomeWork/7/PA7/Assignment7/models/cornellbox/light.obj", light);

	scene.Add(&floor);
	scene.Add(&shortbox);
	scene.Add(&tallbox);
	scene.Add(&left);
	scene.Add(&right);
	scene.Add(&light_);

	scene.buildBVH();

	Renderer r;

	auto start = std::chrono::system_clock::now();
	r.Render(scene);
	auto stop = std::chrono::system_clock::now();

	std::cout << "Render complete: \n";
	std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
	std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
	std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

	return 0;

}