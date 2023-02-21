#include <iostream>
#include <Eigen/Core>
#include <cmath>

using namespace std;

#define PI 3.1415926

using namespace Eigen;

int main()
{
	Vector3f p(2.0f, 1.0f, 1.0f);
	Vector3f result(1.0f, 1.0f, 1.0f);
	//三角函数中的参数是弧度制，角度乘上π/180=弧度
	double x = 45.0 * PI / 180.0;
	

	Matrix3f rotate, translate,transform;
	rotate << cos(x), -sin(x), 0.0, sin(x), cos(x), 0.0, 0.0, 0.0, 1.0;
	translate << 1.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0;
	transform<< 1.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0;

	transform = translate * rotate;

	cout << transform << endl;
	result = transform * p;

	cout << result << endl;


	return 0;
}
