//��ҵ4�Ŀӣ����Mat.at()�����������ʱx��y��˳�����෴��,
//��ʹ��OpenCV����cv2.imread()��ȡͼƬ�󣬲鿴��shape�������ǣ�height��weight��channel����
//Ҳ����˵����ͼ������ϵ������imread().shape[0:2]���ص��ǣ�y,x������ζ�ţ�
//ֱ��ʹ��imread()���ص�ͼ����������ʱ������������ͼ�����������෴

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
	if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)//����������
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
		//4�����Ƶ�ʱ�Ĳ���˹̹����ʽ,���ʱ��t������Ҫ�����ĵ�
		auto point = pow(1 - t, 3) * p_0 + 3 * t * pow(1 - t, 2) * p_1 + 3 * pow(t, 2) * (1 - t) * p_2 + pow(t, 3) * p_3;

		window.at<Vec3b>(point.y, point.x)[2] = 255;//[2]��ʾ�ѵ�����Ϊ��ɫ
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
		//4�����Ƶ�ʱ�Ĳ���˹̹����ʽ,���ʱ��t������Ҫ�����ĵ�
		auto point =recursive_bezier(control_points,t);

		window.at<Vec3b>(point.y, point.x)[1] = 255;//[2]��ʾ�ѵ�����Ϊ��ɫ
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
			circle(window, point, 3, { 255,255,255 }, 3);//ÿ�����Ƶ���һ��Բ�α�ʾ
		}

		if (control_points.size() == 4)
		{
			native_bezier(control_points, window);
			bezier(control_points, window);

			imshow("Bezier Curve", window);//չʾ����
			imwrite("my_bezier_curve.png", window);//����һ��ͼƬ
			key = cv::waitKey(0);

			return 0;
		}

		imshow("Bezier Curve", window);
		key = waitKey(20);
	}

	return 0;
}
