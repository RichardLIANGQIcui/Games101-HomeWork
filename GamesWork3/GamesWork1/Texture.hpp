
#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

class Texture
{
private:
	cv::Mat image_deta;//用于存储或传递图像

public:
	Texture(const string& name)
	{
		image_deta = cv::imread(name);//读入图像
		cv::cvtColor(image_deta, image_deta, cv::COLOR_RGB2BGR);
		width = image_deta.cols;
		height = image_deta.rows;
	}

	int width, height;

	//越界ERROR:
	// OpenCV(4.2.0) Error: Assertion failed((unsigned)(i1* DataType<_Tp>::channels)
	//	< (unsigned)(size.p[1] * channels())) in cv::Mat::at, file D :
	//\ZGKEJIDATXX\OPEN CV\opencv\build\include\opencv2\core\mat.inl.hpp, line 1144
	//	OpenCV: terminate handler is called!The last OpenCV error is :
	//OpenCV(4.2.0) Error : Assertion failed((unsigned)(i1 * DataType<_Tp>::channels)
	//	< (unsigned)(size.p[1] * channels())) in cv::Mat::at, file D : 
	//\ZGKEJIDATXX\OPEN CV\opencv\build\include\opencv2\core\mat.inl.hpp, line 1144


	Vector3f getColor(float u, float v)
	{
		//防止越界
		if (u < 0) { u = 0.0f; }
		if (u> 1) { u = 0.999; }
		if (v < 0) { v = 0.0f; }
		if (v > 1) { v = 0.999; }

		auto u_img = u * width;
		auto v_img = (1-v) * height;//cv原点在左上角，而我们定义的光栅化原点在左下角。这导致我们的v变成了1-v
		auto color = image_deta.at<cv::Vec3b>(v_img, u_img);//at<Vec3b>像素值读写
		return Vector3f(color[0], color[1], color[2]);

	}

	Vector3f getColorBilinear(float u, float v)
	{
		//防止越界
		if (u < 0) { u = 0.0f; }
		if (u > 1) { u = 0.999; }
		if (v < 0) { v = 0.0f; }
		if (v > 1) { v = 0.999; }

		//左下角的纹理坐标
		int w1 = u * width;
		int h1 = (1-v) * height;

		//右下角纹理坐标
		float w2 = w1 + 1;
		float h2 = h1;

		//左上角的纹理坐标
		float w3 = w1;
		float h3 = h1 + 1;

		//右上角的纹理坐标
		float w4 = w1 + 1;
		float h4 = h1 + 1;

		Vector3f color1, color2, color3, color4, color_interpolation_1, color_interpolation_2;
		auto c1 = image_deta.at<cv::Vec3b>((float)w1, float(h1));
		color1 = Vector3f(c1[0], c1[1], c1[2]);

		auto c2 = image_deta.at<cv::Vec3b>((float)w2, float(h2));
		color2 = Vector3f(c2[0], c2[1], c2[2]);

		auto c3 = image_deta.at<cv::Vec3b>((float)w3, float(h3));
		color3 = Vector3f(c3[0], c3[1], c3[2]);

		auto c4 = image_deta.at<cv::Vec3b>((float)w4, float(h4));
		color4 = Vector3f(c4[0], c4[1], c4[2]);

		float s = u * width - w1;
		float t = (1 - v) * height - h1;

		color_interpolation_1 = color1 + s * (color2 - color1);

		color_interpolation_2 = color3 + s * (color4 - color3);

		Vector3f pixel_color = color_interpolation_1 + t * (color_interpolation_2 - color_interpolation_1);

		return pixel_color;
	
	}

};


#endif