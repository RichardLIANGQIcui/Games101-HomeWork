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
	//emplace为map结构中的插入函数，效率会比insert高，因为使用 insert() 向 map 容器中插入键值对的过程是，
	//先创建该键值对，然后再将该键值对复制或者移动到 map 容器中的指定位置
	//使用 emplace()直接在 map 容器中的指定位置构造该键值对
	pos_buf.emplace(id, positions);

	return { id };//因为返回值是个结构体，且结构体内只有int一个变量，所以这里可以直接使用大括号表示
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

//insideTriangle（x, y, v.t） 需要进行调整，框架给的参数是int,其接收变量类型应由int设置为float！不然会自动取整没有效果
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
	//两个向量叉乘的公式在可见中有
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

//求插值计算所需的参数，c1、c2、c3分别为三个顶点对任意一点（x,y)的贡献程度
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

//把所有的点经过mvp变幻和视口变幻，转换成w*h大小的屏幕上显示点
void rasterizer::draw(vector<Triangle*> &TriangleList)
{
	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Matrix4f mvp = projection * view * model;//注意这里矩阵乘法和实际变幻的顺序是相反的
	for (auto& t : TriangleList)
	{
		Triangle newtri = *t;
		//计算得到view空间下的点的位置
		array<Vector4f, 3> mm{
			(view * model * t->v[0]),
			(view * model * t->v[1]),
			(view * model * t->v[2])
		};

		array<Vector3f, 3> viewspace_pos;
		//把视图空间下的三角形坐标由4f转换成3f
		transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
			return v.template head<3>();
			});

		//mvp将三角形转换为锥体中
		Vector4f v[] = {
			mvp*t->v[0],
			mvp* t->v[1],
			mvp* t->v[2]
		};

		for (auto& vec : v)
		{
			vec.x() /= vec.w();//这里便是所谓齐次除法，也被叫做透视除法，经过齐次除法后，视锥体会被变换到一个立方体中，归一化的设备坐标
			vec.y() /= vec.w();//这里v【】里的w分量存的是深度值，所以这里的w分量无需除w
			vec.z() /= vec.w();
		}

		//计算view空间下的法线，推导具体详见https://blog.csdn.net/Q_pril/article/details/123598746
		Matrix4f inv_trans = (view * model).inverse().transpose();
		Vector4f n[] = {
			inv_trans * to_vec4(t->normal[0],0.0f),
			inv_trans * to_vec4(t->normal[1],0.0f),
			inv_trans * to_vec4(t->normal[2],0.0f)
		};

		//视口变化，将三角形从立方体变为空间中屏幕的点,最终与像素位置对应
		//这里解释下每一步，首先vert.x() + 1.0把点的范围从-1~1变到了0~2
		//前面0.5的系数是将范围从0~2变到0~1，最后乘上width或height都把三角形变化到屏幕范围内
		//最后这个z比较特殊，没有要求
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);//将x方向从[-1,1]变为[0,width]
			vert.y() = 0.5 * height * (vert.y() + 1.0);//y方向从[-1,1]变到[0,height]
			vert.z() = vert.z() * f1 + f2;//z方向由[-1,1]变到[0.1,50]左右,z空间任意取值都是正数
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

//顶点插值
static Vector3f interpolate(float alpha, float beta, float gamma, const Vector3f& vert1, const Vector3f& vert2, const Vector3f& vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}
//纹理坐标插值
static Vector2f interpolate(float alpha, float beta, float gamma, const Vector2f& vert1, const Vector2f& vert2, const Vector2f& vert3, float weight)
{
	auto u = alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0];
	auto v = alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1];

	return Vector2f(u, v);
}

//屏幕空间光栅化,采用超采样MSAA
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
			//开始采样点遍历
			float x1 = x + 0.5;
			float y1 = y + 0.5;
			if (insideTriangle(x1, y1, t.v))
			{
				//这里使用的插值还不是一般的插值方法，而是更近一步用到了透视校正（具体查看[GAMES101]现代计算机图形学课程总结4：重心坐标，作业2；https://happyfire.blog.csdn.net/article/details/100148540
				// 
				//其原理就是在只考虑xy2d平面的插值，得到属性值和在三维空间中得到的真正属性值不同，所以需要透视校正

				//w_reciprocal是经过透视校正后的深度值，但是由于作业框架中在viewport transform时，将z值映射到了near,far之间，还需做一步变幻，即第二次插值
				auto [alpha, beta, gamma] = computeBarycentric2D(x1, y1, t.v);

				float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());

				float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				zp *= Z;
				//这里用的深度是超采样的深度坐标，因为整个坐标系每个方向都扩大2倍，所以这里乘上2
				//i的取值可以理解为像素点周边上下左右4个采样点，自然这四个采样点的坐标要映射到放大坐标上
				//当i=0的时候计算左上的采样点，i=1时计算右上采样点
				if (zp < depth_buf[get_index(x, y)])
				{

					depth_buf[get_index(x, y)] = zp;

					//插值颜色作为布林冯光照模型的漫反射系数kd，kd可以理解为模型自身的一种属性
					auto interpolated_color = (alpha * t.color[0] / v[0].w() + beta * t.color[1] / v[1].w() + gamma * t.color[2] / v[2].w()) * Z;
					auto interpolated_normal = (alpha * t.normal[0] / v[0].w() + beta * t.normal[1] / v[1].w() + gamma * t.normal[2] / v[2].w()) * Z;
					auto interpolated_texcoords = (alpha * t.tex_coords[0] / v[0].w() + beta * t.tex_coords[1] / v[1].w() + gamma * t.tex_coords[2] / v[2].w()) * Z;
					auto interpolated_shadingcoords = (alpha * view_pos[0] / v[0].w() + beta * view_pos[1] / v[1].w() + gamma * view_pos[2] / v[2].w()) * Z;
					//上述插值的颜色只是物体本身的颜色，这里考虑光照影响，
					//最终颜色是由着色器输出的，所以还要将插值颜色传入着色器进行光照处理
					fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);//注意这里&*表示先解引用在取地址，因为不是同一个类型
					payload.view_pos = interpolated_shadingcoords;

					//这里得到最终颜色
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

void rasterizer::clear(Buffers buff)//清楚整个显示屏幕，把两个缓存数组都设为0
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
		std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		//注意这里超采样的深度缓存数组全部初始化成0
		std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
	}

}

rasterizer::rasterizer(int w, int h) :width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
	
	//超采样
	frame_sample.resize(w * h * 4);
	depth_sample.resize(w * h * 4);

	texture = nullopt;//初始化，对象不包含任何值
}

int rasterizer::get_index(int x, int y)//根据坐标求像素在缓冲区的序号
{
	return (height - y) * width + x;//height-y是因为OpenCV的坐标系是反过来的
}

//取超采样数组里的索引
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
	//经过变幻后点的位置就是像素的位置，求传入的点的索引值，可以理解为二维数组中的第几个数，也就是二维像素中的第几个像素
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