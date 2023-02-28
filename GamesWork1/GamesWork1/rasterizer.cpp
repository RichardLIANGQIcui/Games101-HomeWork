#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>


using namespace std;
using namespace Eigen;
using namespace rst;

pos_buf_id rasterizer::load_positions(const vector<Vector3f>& positions)
{
	auto id = get_next_id();
	//emplaceΪmap�ṹ�еĲ��뺯����Ч�ʻ��insert�ߣ���Ϊʹ�� insert() �� map �����в����ֵ�ԵĹ����ǣ�
	//�ȴ����ü�ֵ�ԣ�Ȼ���ٽ��ü�ֵ�Ը��ƻ����ƶ��� map �����е�ָ��λ��
	//ʹ�� emplace()ֱ���� map �����е�ָ��λ�ù���ü�ֵ��
	pos_buf.emplace(id, positions);

	return { id };//��Ϊ����ֵ�Ǹ��ṹ�壬�ҽṹ����ֻ��intһ�������������������ֱ��ʹ�ô����ű�ʾ
}

ind_buf_id rasterizer::load_indices(const vector<Vector3i>& indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

//������������֮������ߣ��õ���Bresenham��ֱ���㷨https://blog.csdn.net/qq_41883085/article/details/102706471
void rasterizer::draw_line(Vector3f begin, Vector3f end)
{
	auto x1 = begin.x();
	auto y1 = begin.y();
	auto x2 = end.x();
	auto y2 = end.y();

	Vector3f line_color = { 255,255,255 };

	int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

	dx = x2 - x1;
	dy = y2 - y1;
	dx1 = fabs(dx);
	dy1 = fabs(dy);
	px = 2 * dy1 - dx1;//��������ʼֵ
	py = 2 * dx1 - dy1;

	if (dy1 <= dx1)//��һ���ж��ǳ�x�����ǳ�y�����������
	{
		if (dx >= 0)//��һ����Ϊ���ҳ������ĳ�ʼ��
		{
			x = x1;
			y = y1;
			xe = x2;
		}
		else
		{
			x = x2;
			y = y2;
			xe = x1;
		}
		Vector3f point = Vector3f(x, y, 1.0f);//���õ�һ�������ɫ
		set_pixel(point, line_color);
		for (i = 0;x < xe;i++)//��x������������ĵ㣬������Ϊ���ص���ɫ
		{
			x = x + 1;
			if (px < 0)//�������ж�
			{
				px = px + 2 * dy1;//��һ��y�ǲ����
			}
			else
			{
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))//��һ�����ж�����Ϊy=y+1�ĳ��ִ���2��ͬ�����
				{
					y = y + 1;
				}
				else//�Լ���ͼ��֪��dx��dy��ͬ�����ŵ�����£�ֱ���������ߵ�
				{
					y = y - 1;
				}
				px = px + 2 * (dy1 - dx1);//�������ж�
			}

			Vector3f point = Vector3f(x, y, 1.0f);//����x��y���ҵ�Ҫ���Ƶĵ㣬����ֱ�ӻ��Ƹõ�
			set_pixel(point, line_color);
		}
	}
	else
	{
		if (dy >= 0)
		{
			x = x1;
			y = y1;
			ye = y2;
		}
		else
		{
			x = x2;
			y = y2;
			ye = y1;
		}
		Vector3f point = Vector3f(x, y, 1.0f);
		set_pixel(point, line_color);
		for (i = 0;y < ye;i++)
		{
			y = y + 1;
			if (py <= 0)
			{
				py = py + 2 * dx1;
			}
			else
			{
			if ((dx < 0 && dy < 0) || dx > 0 && dy > 0)
			{
				x = x + 1;
			}
			else
			{
				x = x - 1;
			}
			py = py + 2 * (dx1 - dy1);
			}
			Vector3f point = Vector3f(x, y, 1.0f);
			set_pixel(point, line_color);
		}
	}

}

auto to_vec4(const Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//�����еĵ㾭��mvp��ú��ӿڱ�ã�ת����w*h��С����Ļ����ʾ��
void rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type)
{
	if (type != Primitive::Triangle)
	{
		throw runtime_error("Drawing primitives other than triangle is not implemented yet!");
	}

	auto& buf = pos_buf[pos_buffer.pos_id];//�õ�һ�����飬���������������������λ��
	auto& ind = ind_buf[ind_buffer.ind_id];//ͬ��õ����������ζ������������

	float f1 = (100 - 0.1) / 2.0;
	float f2 = (100 + 0.1) / 2.0;

	Matrix4f mvp = projection * view * model;//ע���������˷���ʵ�ʱ�õ�˳�����෴��
	for (auto& i : ind)
	{
		Triangle t;
		//mvp��������ת��Ϊ��������
		Vector4f v[] = {
			mvp * to_vec4(buf[i[0]],1.0f),
			mvp * to_vec4(buf[i[1]],1.0f),
			mvp * to_vec4(buf[i[2]],1.0f)
		};

		//��һ��
		for (auto& vec : v)
		{
			vec /= vec.w();//���������ĸ�����Ϊ1
		}

		//�ӿڱ仯���������δ��������Ϊ�ռ�����Ļ�ĵ�,����������λ�ö�Ӧ
		//���������ÿһ��������vert.x() + 1.0�ѵ�ķ�Χ��-1~1�䵽��0~2
		//ǰ��0.5��ϵ���ǽ���Χ��0~2�䵽0~1��������width��height���������α仯����Ļ��Χ��
		//������z�Ƚ����⣬û��Ҫ��
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);//��x�����[-1,1]��Ϊ[0,width]
			vert.y() = 0.5 * height * (vert.y() + 1.0);//y�����[-1,1]�䵽[0,height]
			vert.z() = vert.z() * f1 + f2;//z������[-1,1]�䵽[0,100]����
		}

		for (int i = 0;i < 3;++i)
		{
			t.setVertex(i, v[i].head<3>());//head<n>��ʾ��ȡǰn��Ԫ��
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		t.setColor(0, 255.0, 0.0, 0.0);//��ɫû������Ҫ������һ��255����
		t.setColor(1, 0.0, 255.0, 0.0);
		t.setColor(2, 0.0, 0.0, 255.0);

		rasterize_wireframe(t);//����drawline����������
	}

}

void rasterizer::rasterize_wireframe(const Triangle& t)
{
	draw_line(t.c(), t.a());
	draw_line(t.c(), t.b());
	draw_line(t.b(), t.a());
}

void rasterizer::set_model(const Matrix4f& m)
{
	model = m;
}
void rasterizer::set_view(const Matrix4f& v)
{
	view = v;
}

void rasterizer::set_projection(const Matrix4f& p)
{
	projection = p;
}

void rasterizer::clear(Buffers buff)//���������ʾ��Ļ���������������鶼��Ϊ0
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		fill(frame_buf.begin(), frame_buf.end(), Vector3f{ 0,0,0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		//numeric_limits<float>::infinity())��ʾ�������
		fill(depth_buf.begin(), depth_buf.end(), numeric_limits<float>::infinity());
	}
}

rasterizer::rasterizer(int w, int h) :width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
}

int rasterizer::get_index(int x, int y)//���������������ڻ����������
{
	return (height - y) * width + x;//height-y����ΪOpenCV������ϵ�Ƿ�������
}

void rasterizer::set_pixel(const Vector3f& point, const Vector3f& color)
{
	if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
	{
		return;
	}
	//������ú���λ�þ������ص�λ�ã�����ĵ������ֵ���������Ϊ��ά�����еĵڼ�������Ҳ���Ƕ�ά�����еĵڼ�������
	auto ind = (height - point.y()) * width + point.x();
	frame_buf[ind] = color;
}