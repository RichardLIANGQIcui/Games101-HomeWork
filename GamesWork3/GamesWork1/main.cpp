//通过作业三对整个渲染管线做下总结或者说渲染作业中的模型是如何做到的
//准备工作：要绘制一个模型需要哪些类？光栅化类、纹理类、着色器类、三角形类、模型加载类
//从main函数开始解析渲染的一个流程
//第一步读入模型数据到加载类对象中，这个对象含有构建一个三角形所需各个顶点的数据（包括位置、法线、纹理坐标）
//第二步从模型中生成每个三角形并压入一个数组中存起来，后续用于光栅化
//第三步就是创建光栅化器，在三角形内就着色，这里的着色就是把最终显示到屏幕上的颜色传入到帧缓冲中，
// 后续通过OpenCV读取帧缓冲的数据成像
// 首先传入光栅化计算好的变幻矩阵，在光栅化器中将世界空间的坐标变幻到屏幕空间
// 然后在光栅化器中会判断每个像素点是否在三角形内，同时进行深度测试
// 在的话就进行着色，任意一个像素点的属性都通过插值三个顶点的属性值计算，然后把这些插值出来的属性传入到着色器中进行光照处理。
//第四步光照处理中，用到着色器，所有计算是在视图空间下进行的，不同功能的着色器可以实现不用的着色。
//作业中的法线着色器，把法线的值当颜色值输出到像素上
//phong着色器，使用布林冯模型着色，考虑了环境光、漫反射光和高光的影响，这里的kd是第三步传入的插值颜色
//bump着色器即凹凸贴图，这里需要先了解法线和切线空间（https://zhuanlan.zhihu.com/p/144357517），这篇文章详细解释了为什么要使用切线空间中的法线而不是世界空间
//bump的原理就是通过纹理坐标读取凹凸贴图的高度信息计算切线空间下的法线。然后再左乘TBN矩阵得到相机坐标下的法线
//位移贴图则是顶点位置发生了改变，即在凹凸贴图的基础上得到了新的法线，位置也是发生了变化。用新的法线和位置进行光照模型计算




	//总结作业三越到的一些错误
//1、模型数据读取路径不能有空格，否则一片黑、
//2、Texture中的getColor函数必须对参数uv进行限制，且最大值必须小于1，不能等于1，否则会在生成bump、displacement等结果时报错
//3、关于沿用作业2的超采样进行光栅化时，渲染出的模型会出现一个个的三角形边框，这是因为作业2的框架未处理黑线问题
//4、当使用超采样并处理黑线后，随着采样点增加，三角形线框确实少了很多，但即使采样点是到256个的效果仍然存在很多小线条，这里有待解决，按道理来说不应该有黑线，模型内不存在有黑色颜色的采样点
//按照我的理解，这个黑线跟每个三角形的绘制顺序有关，有可能这个像素取的周边的采样点很多都没被赋值，就会产生黑色
//5、作业框架上本身存在问题，但不影响最后结果的实现，比如t.toVector4() 函数的定义把存在顶点的w分量变为1，而w经过
//透视投影变幻后w存的是深度值，即z坐标的相反数，这里在后面透视校正时会用到
//6、最终成像都是在屏幕空间中进行，而光栅化处理会用到屏幕空间坐标系下的顶点数据和视图空间的顶点数据
//一个顶点数据用于屏幕上绘图，第二个顶点数据用于布林冯光照模型的计算，最终得到的颜色是经过着色器处理的
//而着色器处理所用到的所有数据又都是view空间坐标下的



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
	//这里view矩阵变幻的原理就是将场景中的摄像机移动到原点，同时相机上的坐标系对应世界坐标做旋转
	//其他物体也跟着相机一起移动
	//然后相机旋转矩阵求解答的原理是对坐标系旋转到相机坐标系的变幻的逆就是要求的矩阵，而由于矩阵是
	//正交矩阵，所以该正交矩阵的逆是该矩阵的转置矩阵
	//这里没有求旋转，猜想是因为定义相机时，相机自己的坐标系就是和坐标轴一致的
	Matrix4f view = Matrix4f::Identity();//Identity()表示用单位矩阵对变量进行初始化

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

//绕任意轴旋转的旋转矩阵，这里函数实际上更通用，并用到了罗德里格斯公式
//cosa*I + (1-cosa)*(axis*axis.transpose()) + sina*nhat,I是单位矩阵，axis是轴
Matrix4f get_rotation(Vector3f axis, float angle)
{
	//根据上述公式一步步算出每个参数，先求I
	Matrix3f I = Matrix3f::Identity();
	//再求角度
	float a = angle / 180.0 * MY_PI;
	float cosa = cos(a);
	float sina = sin(a);
	//对轴进行归一化
	axis = axis.normalized();
	//求nhat矩阵
	Matrix3f nhat;
	nhat << 0, -axis.z(), axis.y(),
		axis.z(), 0, -axis.x(),
		-axis.y(), axis.x(), 0;

	return toMatrix4f(cosa * I + (1 - cosa) * (axis * axis.transpose()) + sina * nhat);

}

//垂直的可视角度fov,长宽比aspect
Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	Matrix4f projection = Matrix4f::Identity();

	//投影矩阵的原理是将空间压缩成近平面大小的立方体（每条边长为2），再做正交投影

	//第一步，先作出将投影矩阵压缩成正交矩阵的矩阵
	Matrix4f pToO;
	pToO << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -zNear * zFar, 0, 0, 1, 0;

	//求正交投影
	//这里需要求到一些参数，包括默认了长方体的竖直和水平方向的中心都在原点，位移时只改变远近

	//远近方向的边长
	float a = zNear - zFar;//这里是默认朝z轴负方向看，所以是用zNear - zFar

	//竖直方向边长
	float fovY = eye_fov / 180.0 * MY_PI;
	float b = -abs(zNear) * tan(fovY / 2)*2;//因为框架将z进行了翻转，所以这里要加负号,否则最后图片会倒立

	//水平方向边长
	float c = aspect_ratio * b;

	//正交投影是先位移再缩放
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

//求显示法线的颜色
Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
	//下面的一行代码是将法线向量由本身的[-1,1]的范围转换到[0,1]之间，color中的rgb存储的是法线的xyz
	Vector3f return_color = (payload.normal.head<3>().normalized() + Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
	Vector3f result;
	result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
	return result;
}

//反射向量,参数就是入射光向量和像素法线向量,都是单位向量
//首先vec向量的长度为1，1*costheta*2表示入射和反射向量构成的菱形的对角线长度，再乘上axis将标量向量化，最后减去vec可得反射向量
static Vector3f reflect(const Vector3f& vec, const Vector3f& axis)
{
	auto costheta = vec.dot(axis);
	return (2 * costheta * axis - vec).normalized();
}

//定义光源，包含位置和强度
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
	{//①把这个作为一个标准，即光照方向和人眼方向向量都是从物体出发的，故都是-point
		//半程向量h，是归一化后的light_dir和view_dir相加之后得到的方向，之后需再进行一次归一化
		Vector3f lightDir = light.position-point;
		float distance = lightDir.norm();
		lightDir = lightDir.normalized();
		Vector3f viewDir = (eye_pos - point).normalized();
		Vector3f halfDir = (lightDir + viewDir).normalized();
		
		//注意这里向量和向量的乘法要用到cwiseProduct（）函数
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

	//布林冯模型几个参数的设置
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

		//注意这里向量和向量的乘法要用到cwiseProduct（）函数
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

	float kh = 0.2, kn = 0.1;//表示纹理法线对真实物体的影响程度

	// TODO: Implement displacement mapping here
	Vector3f n = normal;
	Vector3f t = { n.x() * n.y() / sqrt(n.x() * n.x()+n.z()*n.z()),sqrt(n.x() * n.x() + n.z() * n.z()),n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()) };
	t = t.normalized();
	Vector3f b = n.cross(t);
	b = b.normalized();
	Matrix3f TBN ;//注意这里TBN的赋值
	TBN << t,b,n ;

	float u = payload.tex_coords.x();
	float v = payload.tex_coords.y();
	float w = payload.texture->width;
	float h = payload.texture->height;

	//注意代码中这里getColor参数是u+1.0/w而不是，u+1,这是因为你需要仔细看下getColor函数
	//该函数中纹理坐标计算乘上了纹理的宽度和高度，最终得到u*width+1,,对应在纹理移动一个单位
	//所以这里的参数要+1.0/w
	float dU = kh * kn * (payload.texture->getColor(u+1.0/w,v).norm()-payload.texture->getColor(u,v).norm());

	float dV = kh * kn * (payload.texture->getColor(u, v+1.0/h).norm() - payload.texture->getColor(u, v).norm());

	Vector3f ln = Vector3f(-dU, -dV, 1);//通过读取纹理贴图的高度计算出的法线，该法线位于切线空间中

	Vector3f result_color = { 0,0,0 };

	result_color = (TBN * ln).normalized();//通过左乘TBN将其转换到相机坐标下，用于着色

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

		//注意这里向量和向量的乘法要用到cwiseProduct（）函数
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

	string filename = "output.png";//默认输出文件名
	objl::Loader Loader;
	string obj_path = "D:/GAMES/HomeWork/3/Assignment3/Code/models/spot/";

	//load.objFile
	//这里有个坑，就是读取模型数据，vs有时候会自动调整你的绝对路径，导致路径出现很多空格，最后结果一片黑
	bool loadout = Loader.LoadFile("D:/GAMES/HomeWork/3/Assignment3/Code/models/spot/spot_triangulated_good.obj");
	for (auto mesh : Loader.LoadedMeshes)
	{
		for (int i = 0;i < mesh.Vertices.size();i+=3)//注意这里遍历所有网格的顶点数，每3个顶点一个三角形，所以是i+=3
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

	rasterizer r(700, 700);//设定光珊器视口

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

	//将不同的着色器传入光栅化器中用于处理光照颜色
	r.set_vertex_shader(vertex_shader);
	r.set_fragment_shader(active_shader);

	int key = 0;
	int frame_count = 0;

	if (command_line)//如果命令行开关标志为开（这一段if代码是为了应用命令行传入的参数，比如初始角度和文件名）
	{
		r.clear(Buffers::Color | Buffers::Depth);//初始化帧缓存和深度缓存（本次作业本次作业只涉及一个图形，所以不涉及深度，可以不管）

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
		
		//光栅化
		r.draw(TriangleList);

		//创建一个mat类用于存储图像数据，CV_32FC3 32表示一个像素点占32位 F表示浮点型 C3表示RGB彩色图像(三通道)
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//data()返回一个指向内存数组的指针
		image.convertTo(image, CV_8UC3, 1.0f);//把一个矩阵从一种数据类型转换到另一种数据类型
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);//将rgb空间转换到OpenCV的bgr空间

		cv::imwrite(filename, image);//保存图像到文件中

		return 0;
	}

	while (key != 27)////只要没有检测到按下ESC就循环(ESC的ASCII码是27)
	{
		r.clear(Buffers::Color | Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(TriangleList);
		
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		cv::imshow("image", image);//该函数将图像在特定的窗口显示
		cv::imwrite(filename, image);
		key = cv::waitKey(10);// 每隔10ms刷新一次图像


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

