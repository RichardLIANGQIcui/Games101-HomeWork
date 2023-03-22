#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

using namespace std;

int main(int argc, char** argv)
{
	Scene scene(1280, 960);

	MeshTriangle bunny("D:/GAMES/HomeWork/6/PA6/Assignment6/models/bunny/bunny.obj");

	scene.Add(&bunny);
	scene.Add(make_unique<Light>(Vector3f(-20, 70, 20), 1));
	scene.Add(make_unique<Light>(Vector3f(20, 70, 20), 1));
	scene.buildBVH();

	Renderer r;

	auto start = chrono::system_clock::now();
	r.Render(scene);
	auto stop = chrono::system_clock::now();

	std::cout << "Render complete: \n";
	std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
	std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
	std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

	return 0;
}