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

//绘制两个顶点之间的连线，用到了Bresenham的直线算法https://blog.csdn.net/qq_41883085/article/details/102706471
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
	px = 2 * dy1 - dx1;//决策数初始值
	py = 2 * dx1 - dy1;

	if (dy1 <= dx1)//这一步判断是朝x方向还是朝y方向递增分量
	{
		if (dx >= 0)//这一步是为了找出递增的初始点
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
		Vector3f point = Vector3f(x, y, 1.0f);//设置第一个点的颜色
		set_pixel(point, line_color);
		for (i = 0;x < xe;i++)//朝x方向遍历后续的点，并设置为像素的颜色
		{
			x = x + 1;
			if (px < 0)//决策树判断
			{
				px = px + 2 * dy1;//这一步y是不变的
			}
			else
			{
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))//这一步的判断是因为y=y+1的出现存在2种同号情况
				{
					y = y + 1;
				}
				else//自己画图可知，dx和dy不同正负号的情况下，直线是向下走的
				{
					y = y - 1;
				}
				px = px + 2 * (dy1 - dx1);//决策树判断
			}

			Vector3f point = Vector3f(x, y, 1.0f);//上述x，y已找到要绘制的点，这里直接绘制该点
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

//把所有的点经过mvp变幻和视口变幻，转换成w*h大小的屏幕上显示点
void rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type)
{
	if (type != Primitive::Triangle)
	{
		throw runtime_error("Drawing primitives other than triangle is not implemented yet!");
	}

	auto& buf = pos_buf[pos_buffer.pos_id];//得到一个数组，里面是三角形三个顶点的位置
	auto& ind = ind_buf[ind_buffer.ind_id];//同理得到的是三角形顶点的索引数组

	float f1 = (100 - 0.1) / 2.0;
	float f2 = (100 + 0.1) / 2.0;

	Matrix4f mvp = projection * view * model;//注意这里矩阵乘法和实际变幻的顺序是相反的
	for (auto& i : ind)
	{
		Triangle t;
		//mvp将三角形转换为立方体中
		Vector4f v[] = {
			mvp * to_vec4(buf[i[0]],1.0f),
			mvp * to_vec4(buf[i[1]],1.0f),
			mvp * to_vec4(buf[i[2]],1.0f)
		};

		//归一化
		for (auto& vec : v)
		{
			vec /= vec.w();//把向量第四个数变为1
		}

		//视口变化，将三角形从立方体变为空间中屏幕的点,最终与像素位置对应
		//这里解释下每一步，首先vert.x() + 1.0把点的范围从-1~1变到了0~2
		//前面0.5的系数是将范围从0~2变到0~1，最后乘上width或height都把三角形变化到屏幕范围内
		//最后这个z比较特殊，没有要求
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);//将x方向从[-1,1]变为[0,width]
			vert.y() = 0.5 * height * (vert.y() + 1.0);//y方向从[-1,1]变到[0,height]
			vert.z() = vert.z() * f1 + f2;//z方向由[-1,1]变到[0,100]左右
		}

		for (int i = 0;i < 3;++i)
		{
			t.setVertex(i, v[i].head<3>());//head<n>表示提取前n个元素
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		t.setColor(0, 255.0, 0.0, 0.0);//颜色没有特殊要求，设置一个255即可
		t.setColor(1, 0.0, 255.0, 0.0);
		t.setColor(2, 0.0, 0.0, 255.0);

		rasterize_wireframe(t);//领用drawline绘制三角形
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

void rasterizer::clear(Buffers buff)//清楚整个显示屏幕，把两个缓存数组都设为0
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		fill(frame_buf.begin(), frame_buf.end(), Vector3f{ 0,0,0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		//numeric_limits<float>::infinity())表示正无穷大
		fill(depth_buf.begin(), depth_buf.end(), numeric_limits<float>::infinity());
	}
}

rasterizer::rasterizer(int w, int h) :width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
}

int rasterizer::get_index(int x, int y)//根据坐标求像素在缓冲区的序号
{
	return (height - y) * width + x;//height-y是因为OpenCV的坐标系是反过来的
}

void rasterizer::set_pixel(const Vector3f& point, const Vector3f& color)
{
	if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
	{
		return;
	}
	//经过变幻后点的位置就是像素的位置，求传入的点的索引值，可以理解为二维数组中的第几个数，也就是二维像素中的第几个像素
	auto ind = (height - point.y()) * width + point.x();
	frame_buf[ind] = color;
}