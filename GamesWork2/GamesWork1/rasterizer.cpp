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

col_buf_id rasterizer::load_colors(const vector<Vector3f>& colors)
{
	auto id = get_next_id();
	col_buf.emplace(id, colors);

	return { id };
}


auto to_vec4(const Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//insideTriangle��x, y, v.t�� ��Ҫ���е�������ܸ��Ĳ�����int,����ձ�������Ӧ��int����Ϊfloat����Ȼ���Զ�ȡ��û��Ч��
static bool insideTriangle(float x, float y, const Vector3f* _v)//
{
	if (_v == nullptr)
	{
		return false;
	}

	Vector3f point(x+0.5, y+0.5,0);
	//����������˵Ĺ�ʽ�ڿɼ�����
	Vector3f v01 = _v[1] - _v[0];
	Vector3f v12 = _v[2] - _v[1];
	Vector3f v20 = _v[0] - _v[2];

	Vector3f v0p = point - _v[0];
	Vector3f v1p = point - _v[1];
	Vector3f v2p = point - _v[2];

	vector<Vector3f> t;
	t.push_back(v01);
	t.push_back(v12);
	t.push_back(v20);

	vector<Vector3f> g;
	g.push_back(v0p);
	g.push_back(v1p);
	g.push_back(v2p);

	vector<Vector3f> k;
	Vector3f tmp;
	for (int i=0;i<3;i++)
	{
		tmp = t[i].cross(g[i]);
		k.push_back(tmp);
	}

	if ((k[0].z() > 0 && k[1].z() > 0 && k[2].z() > 0) ||
		(k[0].z() < 0 && k[1].z() < 0 && k[2].z() < 0))
	{
		return true;
	}
	else
	{
		return false;
	}

}

//���ֵ��������Ĳ�����c1��c2��c3�ֱ�Ϊ�������������һ�㣨x,y)�Ĺ��׳̶�
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

//�����еĵ㾭��mvp��ú��ӿڱ�ã�ת����w*h��С����Ļ����ʾ��
void rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer,Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];//�õ�һ�����飬���������������������λ��
	auto& ind = ind_buf[ind_buffer.ind_id];//ͬ��õ����������ζ������������
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

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
			vert.z() = vert.z() * f1 + f2;//z������[-1,1]�䵽[0.1,50]����,z�ռ�����ȡֵ��������
		}

		for (int i = 0;i < 3;++i)
		{
			t.setVertex(i, v[i].head<3>());//head<n>��ʾ��ȡǰn��Ԫ��
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);//����drawline����������
	}

}
//��Ļ�ռ��դ��
void rasterizer::rasterize_triangle(const Triangle& t)
{
	auto v = t.toVector4();
	int left_bound = min(v[2].x(), min(v[0].x(), v[1].x()));

	int right_bound = max(max(v[0].x(), v[1].x()), v[2].x());

	int up_bound = min(min(v[0].y(), v[1].y()), v[2].y());

	int down_bound = max(max(v[0].y(), v[1].y()), v[2].y());


	float X[4] = { -0.25,0.25,-0.25,0.25 };
	float Y[4] = { -0.25,-0.25,0.25,0.25 };
	for (int x = left_bound;x <=right_bound;x++)
	{
		for (int y = up_bound;y <down_bound;y++)
		{
			float x1 = x + 0.5;
			float y1 = y + 0.5;
			int count = 0;

			for (int i = 0;i < 4;i++)
			{
				float x2 = x1 + X[i];
				float y2 = y1 + Y[i];

				if (insideTriangle(x2, y2, t.v))
				{
					count++;
				}

			}
	
				if (count>0)
				{
					float alpha=0, beta=0, gamma=0;
					auto tup1 = computeBarycentric2D(x1, y1, t.v);
					tie(alpha, beta, gamma) = tup1;
					//����ʹ�õĲ�ֵ������һ��Ĳ�ֵ���������Ǹ���һ���õ���͸��У��������鿴[GAMES101]�ִ������ͼ��ѧ�γ��ܽ�4���������꣬��ҵ2��https://happyfire.blog.csdn.net/article/details/100148540
					// 
					//��ԭ�������ֻ����xy2dƽ��Ĳ�ֵ���õ�����ֵ������ά�ռ��еõ�����������ֵ��ͬ��������Ҫ͸��У��

					//w_reciprocal�Ǿ���͸��У��������ֵ������������ҵ�������viewport transformʱ����zֵӳ�䵽��near,far֮�䣬������һ����ã����ڶ��β�ֵ
					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					
					z_interpolated *= w_reciprocal;
					
					if (z_interpolated < depth_buf[get_index(x, y)])
					{
						depth_buf[get_index(x, y)] = z_interpolated;
						Vector3f point(x, y, z_interpolated);
						set_pixel(point, t.getColor()*(float)count/4.0);
					}
				}
		}
	}

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
	return (height -1- y) * width + x;//height-y����ΪOpenCV������ϵ�Ƿ�������
}

void rasterizer::set_pixel(const Vector3f& point, const Vector3f& color)
{
	if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
	{
		return;
	}
	//������ú���λ�þ������ص�λ�ã�����ĵ������ֵ���������Ϊ��ά�����еĵڼ�������Ҳ���Ƕ�ά�����еĵڼ�������
	auto ind = (height -1- point.y()) * width + point.x();
	frame_buf[ind] = color;
}