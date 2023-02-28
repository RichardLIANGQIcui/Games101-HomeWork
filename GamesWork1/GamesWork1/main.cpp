#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <Eigen/Eigen>

#include "rasterizer.hpp"
#include "Triangle.hpp"

//注意OpenCV的配置问题，如果配置不对是无法运行程序的。opencv是64位，所以vs上面的Debug平台也要改成64，而不是x86

using namespace std;
using namespace Eigen;
using namespace rst;

constexpr double MY_PI = 3.1415926;

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

Matrix4f get_model_matrix(float rotation_angle)//注意这里参数给的是旋转的角度，要把它转化成弧度制
{
	Matrix4f model = Matrix4f::Identity();

	Vector3f axis{ 1,1,1 };//旋转轴
	Matrix4f t;
	t << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	model = get_rotation(axis, rotation_angle);

	return model*t;
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
	float b = abs(zNear) * tan(fovY / 2)*2;

	//水平方向边长
	float c = aspect_ratio * b;

	//正交投影是先位移再缩放
	Matrix4f ortho,scale,trans;
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

int main(int argc, const char** argv)
{
	//梳理整个成像流程
//1、定义一些基本变量，比如角度、控制动作的bool变量、文件名
//2、处理用户输入，比如角度，这里比较例外，是用cmd命令处理
//3、设定一个有长宽的光栅器、定义相机位置，三角形顶点数据、保存图形数量的序号
//4、利用光栅器加载三角形顶点和索引并将对应序号返回
//5、处理键盘输入和定义帧数数量
//6、如果命令行开关为开，开始绘图流程
//7、先初始化或清空帧缓存中的数据，将外部变量的mvp矩阵通过光栅器传入，
//数据传入后调用光栅器的绘图函数，此时所有的数据都已存入光栅器的帧缓冲中
//8、利用opencv创建图像，并用光栅器中的帧缓冲的数据初始化图像，调用图像的转换和写入函数，保存到最开始定义的文件名中
//9、最后利用while循环假如用户没有按下esc按键，那么就继续重复上述成像的一个过程，每个循环加载每一帧的画面


	float angle = 0;
	bool command_line = false;
	string filename = "output.png";//默认输出文件名


	//打开 cmd（命令提示符程序）开始处理用户的输入,比如角度的输入，argc 表示传递的字符串的数目
	if (argc >= 3) {// 接收到的参数大于三个，即检测到通过命令行传入参数时,
		command_line = true;//设命令行开关标志为开
		angle = stof(argv[2]);//从命令行获取角度参数
		if (argc == 4)//接收到的参数为四个，那么说明命令行输入了文件名参数
		{
			filename = string(argv[3]);
		}
		else
		{
			return 0;
		}
	}

	rasterizer r(700, 700);//设定光珊器视口

	Vector3f eye_pos = { 0,0,5 };//定义摄像机的位置

	vector<Vector3f> pos{ {2,0,-2},{0,2,-2},{-2,0,-2}};//三角形三个顶点的位置

	vector<Vector3i> ind{ {0,1,2} };//保存多个图形的顶点和序号，本次作业只涉及一个图形，可以不管

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);

	int key = 0;//键盘输入
	int frame_count = 0;//一共生成的帧数

	if (command_line)//如果命令行开关标志为开（这一段if代码是为了应用命令行传入的参数，比如初始角度和文件名）
	{
		r.clear(Buffers::Color | Buffers::Depth);//初始化帧缓存和深度缓存（本次作业本次作业只涉及一个图形，所以不涉及深度，可以不管）

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
		

		//光栅化
		r.draw(pos_id, ind_id, Primitive::Triangle);

		//创建一个mat类用于存储图像数据，CV_32FC3 32表示一个像素点占32位 F表示浮点型 C3表示RGB彩色图像(三通道)
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//data()返回一个指向内存数组的指针
		image.convertTo(image, CV_8UC3, 1.0f);//把一个矩阵从一种数据类型转换到另一种数据类型
	
		cv::imwrite(filename, image);//保存图像到文件中

		return 0;
	}

	while (key != 27)////只要没有检测到按下ESC就循环(ESC的ASCII码是27)
	{
		r.clear(Buffers::Color | Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, Primitive::Triangle);
		
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);//该函数将图像在特定的窗口显示
		key = cv::waitKey(10);// 每隔10ms刷新一次图像
		cout << "frame count:" << frame_count++ << endl;

		if (key == 'a') {
			angle += 10;
		}
		else if (key == 'd') {
			angle -= 10;
		}

	}

	return 0;
}