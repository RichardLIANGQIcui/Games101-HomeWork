#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <Eigen/Eigen>

#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "global.hpp"

//ע��OpenCV���������⣬������ò������޷����г���ġ�opencv��64λ������vs�����Debugƽ̨ҲҪ�ĳ�64��������x86

//�ܽ�����ҵ2����ҵĿ����Ϊ��ʵ��ʵ�������εĻ��ƣ���ԭ����ǹ�դ��
//��դ����ԭ����Ǳ�����Ļ��ÿ�����ص����ĵ㣬����õ����������ھͽ���������ɫ��
//�ж�һ�����ڲ����������������������ԭ���õ��������������������ɵ�������������������ˣ�����һ��������������
//����Ŀ��������MSAA���п���ݣ�ԭ����ǽ�ÿ��������ϸ�ָ��ӣ�Ȼ���ÿ�����Ӳ���������ƽ�����õ��Ľ�����ƽ��
//������Ŀ���ںڱ����⣬����Ϊ���Ƶڶ���������ʱ���ڲ����׶Σ��ر������������εı߽�
//������ֵ㲻���������ڣ�Ĭ�ϸõ��Ǳ�����ɫ����ʵ���ϸõ����ɫ�ǵ�һ�������ε���ɫ
//����ڱ��������Ҫ��������ȫ�ֱ������ֱ��¼�����Ϣ��������ɫ������������ȫ�ֱ����������������β����»���

using namespace std;
using namespace Eigen;
using namespace rst;

constexpr double MY_PI = 3.1415926;

Matrix4f get_view_matrix(Vector3f eye_pos)
{
	//����view�����õ�ԭ����ǽ������е�������ƶ���ԭ�㣬ͬʱ����ϵ�����ϵ��Ӧ������������ת
	//��������Ҳ�������һ���ƶ�
	//Ȼ�������ת���������ԭ���Ƕ�����ϵ��ת���������ϵ�ı�õ������Ҫ��ľ��󣬶����ھ�����
	//�����������Ը�������������Ǹþ����ת�þ���
	//����û������ת����������Ϊ�������ʱ������Լ�������ϵ���Ǻ�������һ�µ�
	Matrix4f view = Matrix4f::Identity();//Identity()��ʾ�õ�λ����Ա������г�ʼ��

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

//����������ת����ת�������ﺯ��ʵ���ϸ�ͨ�ã����õ����޵����˹��ʽ
//cosa*I + (1-cosa)*(axis*axis.transpose()) + sina*nhat,I�ǵ�λ����axis����
Matrix4f get_rotation(Vector3f axis, float angle)
{
	//����������ʽһ�������ÿ������������I
	Matrix3f I = Matrix3f::Identity();
	//����Ƕ�
	float a = angle / 180.0 * MY_PI;
	float cosa = cos(a);
	float sina = sin(a);
	//������й�һ��
	axis = axis.normalized();
	//��nhat����
	Matrix3f nhat;
	nhat << 0, -axis.z(), axis.y(),
		axis.z(), 0, -axis.x(),
		-axis.y(), axis.x(), 0;

	return toMatrix4f(cosa * I + (1 - cosa) * (axis * axis.transpose()) + sina * nhat);

}

Matrix4f get_model_matrix(float rotation_angle)
{
	Matrix4f model = Matrix4f::Identity();;

	return model;
}



//��ֱ�Ŀ��ӽǶ�fov,�����aspect
Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	Matrix4f projection = Matrix4f::Identity();

	//ͶӰ�����ԭ���ǽ��ռ�ѹ���ɽ�ƽ���С�������壨ÿ���߳�Ϊ2������������ͶӰ

	//��һ������������ͶӰ����ѹ������������ľ���
	Matrix4f pToO;
	pToO << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -zNear * zFar, 0, 0, 1, 0;

	//������ͶӰ
	//������Ҫ��һЩ����������Ĭ���˳��������ֱ��ˮƽ��������Ķ���ԭ�㣬λ��ʱֻ�ı�Զ��

	//Զ������ı߳�
	float a = zNear - zFar;//������Ĭ�ϳ�z�Ḻ���򿴣���������zNear - zFar

	//��ֱ����߳�
	float fovY = eye_fov / 180.0 * MY_PI;
	float b = -abs(zNear) * tan(fovY / 2)*2;//��Ϊ��ܽ�z�����˷�ת����������Ҫ�Ӹ���

	//ˮƽ����߳�
	float c = aspect_ratio * b;

	//����ͶӰ����λ��������
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
	//����������������
//1������һЩ��������������Ƕȡ����ƶ�����bool�������ļ���
//2�������û����룬����Ƕȣ�����Ƚ����⣬����cmd�����
//3���趨һ���г���Ĺ�դ�����������λ�ã������ζ������ݡ�����ͼ�����������
//4�����ù�դ�����������ζ��������������Ӧ��ŷ���
//5�������������Ͷ���֡������
//6����������п���Ϊ������ʼ��ͼ����
//7���ȳ�ʼ�������֡�����е����ݣ����ⲿ������mvp����ͨ����դ�����룬
//���ݴ������ù�դ���Ļ�ͼ��������ʱ���е����ݶ��Ѵ����դ����֡������
//8������opencv����ͼ�񣬲��ù�դ���е�֡��������ݳ�ʼ��ͼ�񣬵���ͼ���ת����д�뺯�������浽�ʼ������ļ�����
//9���������whileѭ�������û�û�а���esc��������ô�ͼ����ظ����������һ�����̣�ÿ��ѭ������ÿһ֡�Ļ���


	float angle = 0;
	bool command_line = false;
	string filename = "output.png";//Ĭ������ļ���


	//�� cmd��������ʾ�����򣩿�ʼ�����û�������,����Ƕȵ����룬argc ��ʾ���ݵ��ַ�������Ŀ
	if (argc == 2) {// ���յ��Ĳ�������2��������⵽ͨ�������д������ʱ,
		command_line = true;//�������п��ر�־Ϊ��
		filename = string(argv[1]);
		
	}

	rasterizer r(700, 700);//�趨��ɺ���ӿ�

	Vector3f eye_pos = { 0,0,5 };//�����������λ��

	vector<Vector3f> pos{ //���������������λ��
			   {2, 0, -2},
				{0, 2, -2},
				{-2, 0, -2},
				{3.5, -1, -5},
				{2.5, 1.5, -5},
				{-1, 0.5, -5}
	};

	vector<Vector3i> ind{ //������ͼ�εĶ�������
					{0, 1, 2},
					{3, 4, 5} };

	vector<Vector3f> cols//����ÿ��������ɫ
	{
				    {217.0, 238.0, 185.0},
					{217.0, 238.0, 185.0},
					{217.0, 238.0, 185.0},
					{185.0, 217.0, 238.0},
					{185.0, 217.0, 238.0},
					{185.0, 217.0, 238.0}
	};

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);
	auto col_id = r.load_colors(cols);

	int key = 0;//��������
	int frame_count = 0;//һ�����ɵ�֡��

	if (command_line)//��������п��ر�־Ϊ������һ��if������Ϊ��Ӧ�������д���Ĳ����������ʼ�ǶȺ��ļ�����
	{
		r.clear(Buffers::Color | Buffers::Depth);//��ʼ��֡�������Ȼ��棨������ҵ������ҵֻ�漰һ��ͼ�Σ����Բ��漰��ȣ����Բ��ܣ�

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
		

		//��դ��
		r.draw(pos_id, ind_id,col_id, Primitive::Triangle);

		//����һ��mat�����ڴ洢ͼ�����ݣ�CV_32FC3 32��ʾһ�����ص�ռ32λ F��ʾ������ C3��ʾRGB��ɫͼ��(��ͨ��)
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//data()����һ��ָ���ڴ������ָ��
		image.convertTo(image, CV_8UC3, 1.0f);//��һ�������һ����������ת������һ����������
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);//��rgb�ռ�ת����OpenCV��bgr�ռ�

		cv::imwrite(filename, image);//����ͼ���ļ���

		return 0;
	}

	while (key != 27)////ֻҪû�м�⵽����ESC��ѭ��(ESC��ASCII����27)
	{
		r.clear(Buffers::Color | Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id,col_id, Primitive::Triangle);
		
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		cv::imshow("image", image);//�ú�����ͼ�����ض��Ĵ�����ʾ
		key = cv::waitKey(10);// ÿ��10msˢ��һ��ͼ��
		cout << "frame count:" << frame_count++ << endl;

	}

	return 0;
}

