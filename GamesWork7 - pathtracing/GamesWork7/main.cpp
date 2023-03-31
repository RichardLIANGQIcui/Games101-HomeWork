//我发现在生成随机数的函数中使用staic这个关键字能将程序在多线程的基础上提速一倍以上，
//一、spp : 256, 不使用多线程、和staic关键字，耗时27hour
//二、spp : 256, 使用多线程，但不用static关键字，耗时3小时
//三、spp : 256, 使用多线程和static，耗时1小时左右
// 这样定义的变量通常被称为局部静态变量，它的值不会因为函数调用的结束而被清除，
// 当函数再次被调用时，它的值是上一次调用结束后的值
//在函数中声明变量时， static 关键字指定变量只初始化一次，并在之后调用该函数时保留其状态
//后面的调用就不用再分配内存空间，只要改变变量本身即可
//反复构造和销毁内存也是有时间开销的，使用static保证了变量在整个程序运行期间只需一次生成，后面更新值即可

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