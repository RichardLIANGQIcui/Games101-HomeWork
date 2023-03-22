#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

using namespace std;

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

void Renderer::Render(const Scene& scene)
{
	vector<Vector3f> framebuffer(scene.width * scene.height);
	float scale = tan(deg2rad(scene.fov) / 2);
	float imageAspectRatio = scene.width / (float)scene.height;
	Vector3f eye_pos(-1, 5, 10);
	int m = 0;
	for (uint32_t j = 0;j < scene.height;j++)
	{
		for (uint32_t i = 0;i < scene.width;i++)
		{
			float x = (2 *( i + 0.5) / (float)scene.width - 1)* imageAspectRatio* scale;
			float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

			Vector3f dir = Vector3f(x, y, - 1);
			dir = normalize(dir);
			framebuffer[m++] = scene.castRay(Ray(eye_pos, dir), 0);
		}
		UpdateProgress(j / (float)scene.height);
	}
	UpdateProgress(1.f);

	FILE* fp = fopen("binary.ppm", "wb");
	(void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
	for (auto i = 0;i < scene.width * scene.height;i++)
	{
		static unsigned char color[3];
		color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
		color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
		color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));

		fwrite(color, 1, 3, fp);
	}
	fclose(fp);
}