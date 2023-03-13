//作业4的坑，这个Mat.at()函数传入参数时x和y的顺序是相反的,
//在使用OpenCV函数cv2.imread()读取图片后，查看其shape，发现是（height，weight，channel），
//也就是说，按图像坐标系来讲，imread().shape[0:2]返回的是（y,x）。意味着，
//直接使用imread()返回的图像里的坐标点时，与我们理解的图像坐标正好相反

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

vector<Point2f> control_points;

int X[8] = { -1,0,1,-1 ,1,-1,0,1};
int Y[8] = {-1,-1,-1,0,0,1,1,1};

void mouse_handle(int event,int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)//按下鼠标左键
	{
		cout << "Left button of the mouse is clicked-position(" << x << "," << y << ")" << endl;
		control_points.emplace_back(x, y);
	}
}

void native_bezier(const vector<Point2f>& points, Mat& window)
{
	auto& p_0 = points[0];
	auto& p_1 = points[1];
	auto& p_2 = points[2];
	auto& p_3 = points[3];

	for (double t = 0.0;t <= 1.0;t += 0.001)
	{
		//4个控制点时的伯恩斯坦多项式,算出时间t下曲线要经过的点
		auto point = pow(1 - t, 3) * p_0 + 3 * t * pow(1 - t, 2) * p_1 + 3 * pow(t, 2) * (1 - t) * p_2 + pow(t, 3) * p_3;

		window.at<Vec3b>(point.y, point.x)[2] = 255;//[2]表示把点设置为蓝色
		for (int i = 0;i < 8;i++)
		{
			float x1 = point.x + X[i];
			float y1 = point.y + Y[i];
			window.at<Vec3b>(y1, x1)[2] = 255 / sqrt(X[i] * X[i] + Y[i] * Y[i]);
		}

	}

}

Point2f recursive_bezier(const vector<Point2f>& control_points, float t)
{
	vector<Point2f> sample_points = control_points;
	
	while (sample_points.size() != 1)
	{
		vector<Point2f> temple_points;
		for (int i = 0;i < sample_points.size() - 1;i++)
		{
			Point2f point = (1 - t) * sample_points[i] + t*sample_points[i + 1];
			temple_points.push_back(point);
		}
		sample_points = temple_points;
	}

	return sample_points[0];
}

void bezier(const vector<Point2f>& control_points, Mat& window)
{
	for (double t = 0.0;t <= 1.0;t += 0.001)
	{
		//4个控制点时的伯恩斯坦多项式,算出时间t下曲线要经过的点
		auto point =recursive_bezier(control_points,t);

		window.at<Vec3b>(point.y, point.x)[1] = 255;//[2]表示把点设置为绿色
		for (int i = 0;i < 8;i++)
		{
			float x1 = point.x + X[i];
			float y1 = point.y + Y[i];
			window.at<Vec3b>(y1, x1)[2] = 255 / (X[i] * X[i] + Y[i] * Y[i]);
		}
		
	}
}

int main()
{
	Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
	cvtColor(window, window, COLOR_BGR2RGB);
	namedWindow("Bezier Curve", WINDOW_AUTOSIZE);

	setMouseCallback("Bezier Curve", mouse_handle, nullptr);

	int key = -1;
	while (key != 27)
	{
		for (auto& point : control_points)
		{
			circle(window, point, 3, { 255,255,255 }, 3);//每个控制点用一个圆形表示
		}

		if (control_points.size() == 4)
		{
			native_bezier(control_points, window);
			bezier(control_points, window);

			imshow("Bezier Curve", window);//展示窗口
			imwrite("my_bezier_curve.png", window);//生成一张图片
			key = cv::waitKey(0);

			return 0;
		}

		imshow("Bezier Curve", window);
		key = waitKey(20);
	}

	return 0;
}
