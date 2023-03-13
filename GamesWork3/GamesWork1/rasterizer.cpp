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

col_buf_id rasterizer::load_normals(const vector<Vector3f>& normals)
{
	auto id = get_next_id();
	nor_buf.emplace(id, normals);

	normal_id = id;

	return { id };
}

void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
	auto x1 = begin.x();
	auto y1 = begin.y();
	auto x2 = end.x();
	auto y2 = end.y();

	Eigen::Vector3f line_color = { 255, 255, 255 };

	int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

	dx = x2 - x1;
	dy = y2 - y1;
	dx1 = fabs(dx);
	dy1 = fabs(dy);
	px = 2 * dy1 - dx1;
	py = 2 * dx1 - dy1;

	if (dy1 <= dx1)
	{
		if (dx >= 0)
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
		Eigen::Vector2i point = Eigen::Vector2i(x, y);
		set_pixel(point, line_color);
		for (i = 0;x < xe;i++)
		{
			x = x + 1;
			if (px < 0)
			{
				px = px + 2 * dy1;
			}
			else
			{
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
				{
					y = y + 1;
				}
				else
				{
					y = y - 1;
				}
				px = px + 2 * (dy1 - dx1);
			}
			//            delay(0);
			Eigen::Vector2i point = Eigen::Vector2i(x, y);
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
		Eigen::Vector2i point = Eigen::Vector2i(x, y);
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
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
				{
					x = x + 1;
				}
				else
				{
					x = x - 1;
				}
				py = py + 2 * (dx1 - dy1);
			}
			//            delay(0);
			Eigen::Vector2i point = Eigen::Vector2i(x, y);
			set_pixel(point, line_color);
		}
	}
}

auto to_vec4(const Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//insideTriangle��x, y, v.t�� ��Ҫ���е�������ܸ��Ĳ�����int,����ձ�������Ӧ��int����Ϊfloat����Ȼ���Զ�ȡ��û��Ч��
static bool insideTriangle(float x, float y, const Vector4f* _v)//
{
	if (_v == nullptr)
	{
		return false;
	}

	Vector3f v[3];
	for (int i = 0;i < 3;i++)
	{
		v[i] = { _v[i].x(),_v[i].y(),_v[i].z() };
	}

	Vector3f point(x, y,0);
	//����������˵Ĺ�ʽ�ڿɼ�����
	Vector3f v01 = v[1] - v[0];
	Vector3f v12 = v[2] - v[1];
	Vector3f v20 = v[0] - v[2];

	Vector3f v0p = point - v[0];
	Vector3f v1p = point - v[1];
	Vector3f v2p = point - v[2];

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
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

//�����еĵ㾭��mvp��ú��ӿڱ�ã�ת����w*h��С����Ļ����ʾ��
void rasterizer::draw(vector<Triangle*> &TriangleList)
{
	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Matrix4f mvp = projection * view * model;//ע���������˷���ʵ�ʱ�õ�˳�����෴��
	for (auto& t : TriangleList)
	{
		Triangle newtri = *t;
		//����õ�view�ռ��µĵ��λ��
		array<Vector4f, 3> mm{
			(view * model * t->v[0]),
			(view * model * t->v[1]),
			(view * model * t->v[2])
		};

		array<Vector3f, 3> viewspace_pos;
		//����ͼ�ռ��µ�������������4fת����3f
		transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
			return v.template head<3>();
			});

		//mvp��������ת��Ϊ׶����
		Vector4f v[] = {
			mvp*t->v[0],
			mvp* t->v[1],
			mvp* t->v[2]
		};

		for (auto& vec : v)
		{
			vec.x() /= vec.w();//���������ν��γ�����Ҳ������͸�ӳ�����������γ�������׶��ᱻ�任��һ���������У���һ�����豸����
			vec.y() /= vec.w();//����v�������w������������ֵ�����������w���������w
			vec.z() /= vec.w();
		}

		//����view�ռ��µķ��ߣ��Ƶ��������https://blog.csdn.net/Q_pril/article/details/123598746
		Matrix4f inv_trans = (view * model).inverse().transpose();
		Vector4f n[] = {
			inv_trans * to_vec4(t->normal[0],0.0f),
			inv_trans * to_vec4(t->normal[1],0.0f),
			inv_trans * to_vec4(t->normal[2],0.0f)
		};

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
			newtri.setVertex(i, v[i]);
		}

		for (int i = 0;i < 3;i++)
		{
			newtri.setNormal(i, n[i].head<3>());
		}

		newtri.setColor(0, 35, 220.0, 200.0);
		newtri.setColor(1, 164, 113.0, 200.0);
		newtri.setColor(2, 150, 230.0, 210.0);

		rasterize_triangle(newtri,viewspace_pos);//
	}

}

//�����ֵ
static Vector3f interpolate(float alpha, float beta, float gamma, const Vector3f& vert1, const Vector3f& vert2, const Vector3f& vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}
//���������ֵ
static Vector2f interpolate(float alpha, float beta, float gamma, const Vector2f& vert1, const Vector2f& vert2, const Vector2f& vert3, float weight)
{
	auto u = alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0];
	auto v = alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1];

	return Vector2f(u, v);
}

//��Ļ�ռ��դ��,���ó�����MSAA
void rasterizer::rasterize_triangle(const Triangle& t,const array<Vector3f,3>& view_pos)
{
	auto v = t.toVector4();
	int left_bound = min(v[2].x(), min(v[0].x(), v[1].x()));

	int right_bound = max(max(v[0].x(), v[1].x()), v[2].x());

	int up_bound = min(min(v[0].y(), v[1].y()), v[2].y());

	int down_bound = max(max(v[0].y(), v[1].y()), v[2].y());


	for (int x = left_bound;x <=right_bound;x++)
	{
		for (int y = up_bound;y <down_bound;y++)
		{
			//��ʼ���������
			float x1 = x + 0.5;
			float y1 = y + 0.5;
			if (insideTriangle(x1, y1, t.v))
			{
				//����ʹ�õĲ�ֵ������һ��Ĳ�ֵ���������Ǹ���һ���õ���͸��У��������鿴[GAMES101]�ִ������ͼ��ѧ�γ��ܽ�4���������꣬��ҵ2��https://happyfire.blog.csdn.net/article/details/100148540
				// 
				//��ԭ�������ֻ����xy2dƽ��Ĳ�ֵ���õ�����ֵ������ά�ռ��еõ�����������ֵ��ͬ��������Ҫ͸��У��

				//w_reciprocal�Ǿ���͸��У��������ֵ������������ҵ�������viewport transformʱ����zֵӳ�䵽��near,far֮�䣬������һ����ã����ڶ��β�ֵ
				auto [alpha, beta, gamma] = computeBarycentric2D(x1, y1, t.v);

				float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());

				float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				zp *= Z;
				//�����õ�����ǳ�������������꣬��Ϊ��������ϵÿ����������2���������������2
				//i��ȡֵ�������Ϊ���ص��ܱ���������4�������㣬��Ȼ���ĸ������������Ҫӳ�䵽�Ŵ�������
				//��i=0��ʱ��������ϵĲ����㣬i=1ʱ�������ϲ�����
				if (zp < depth_buf[get_index(x, y)])
				{

					depth_buf[get_index(x, y)] = zp;

					//��ֵ��ɫ��Ϊ���ַ����ģ�͵�������ϵ��kd��kd�������Ϊģ�������һ������
					auto interpolated_color = (alpha * t.color[0] / v[0].w() + beta * t.color[1] / v[1].w() + gamma * t.color[2] / v[2].w()) * Z;
					auto interpolated_normal = (alpha * t.normal[0] / v[0].w() + beta * t.normal[1] / v[1].w() + gamma * t.normal[2] / v[2].w()) * Z;
					auto interpolated_texcoords = (alpha * t.tex_coords[0] / v[0].w() + beta * t.tex_coords[1] / v[1].w() + gamma * t.tex_coords[2] / v[2].w()) * Z;
					auto interpolated_shadingcoords = (alpha * view_pos[0] / v[0].w() + beta * view_pos[1] / v[1].w() + gamma * view_pos[2] / v[2].w()) * Z;
					//������ֵ����ɫֻ�����屾�����ɫ�����￼�ǹ���Ӱ�죬
					//������ɫ������ɫ������ģ����Ի�Ҫ����ֵ��ɫ������ɫ�����й��մ���
					fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);//ע������&*��ʾ�Ƚ�������ȡ��ַ����Ϊ����ͬһ������
					payload.view_pos = interpolated_shadingcoords;

					//����õ�������ɫ
					auto pixel_color = fragment_shader(payload);

					frame_buf[get_index(x, y)] = pixel_color;

					Vector2i point(x, y);

					set_pixel(point, pixel_color);

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
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
		std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		//ע�����ﳬ��������Ȼ�������ȫ����ʼ����0
		std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
	}

}

rasterizer::rasterizer(int w, int h) :width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
	
	//������
	frame_sample.resize(w * h * 4);
	depth_sample.resize(w * h * 4);

	texture = nullopt;//��ʼ�������󲻰����κ�ֵ
}

int rasterizer::get_index(int x, int y)//���������������ڻ����������
{
	return (height - y) * width + x;//height-y����ΪOpenCV������ϵ�Ƿ�������
}

//ȡ�����������������
int rasterizer::get_super_index(int x, int y)
{
	return (height * 2 - 1 - y) * width * 2 + x;
}

void rasterizer::set_pixel(const Vector2i& point, const Vector3f& color)
{
	if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
	{
		return;
	}
	//������ú���λ�þ������ص�λ�ã�����ĵ������ֵ���������Ϊ��ά�����еĵڼ�������Ҳ���Ƕ�ά�����еĵڼ�������
	auto ind = (height - point.y()) * width + point.x();
	frame_buf[ind] = color;
}

void rasterizer::set_vertex_shader(function<Vector3f(vertex_shader_payload)> vert_shader)
{
	vertex_shader = vert_shader;
}

void rasterizer::set_fragment_shader(function<Vector3f(fragment_shader_payload)> frag_shader)
{
	fragment_shader = frag_shader;
}