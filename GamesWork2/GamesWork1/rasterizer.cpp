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


auto to_vec4(const Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//insideTriangle（x, y, v.t） 需要进行调整，框架给的参数是int,其接收变量类型应由int设置为float！不然会自动取整没有效果
static bool insideTriangle(float x, float y, const Vector3f* _v)//
{
	if (_v == nullptr)
	{
		return false;
	}

	Vector3f point(x+0.5, y+0.5,0);
	//两个向量叉乘的公式在可见中有
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

//求插值计算所需的参数，c1、c2、c3分别为三个顶点对任意一点（x,y)的贡献程度
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

//把所有的点经过mvp变幻和视口变幻，转换成w*h大小的屏幕上显示点
void rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer,Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];//得到一个数组，里面是三角形三个顶点的位置
	auto& ind = ind_buf[ind_buffer.ind_id];//同理得到的是三角形顶点的索引数组
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

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
			vert.z() = vert.z() * f1 + f2;//z方向由[-1,1]变到[0.1,50]左右,z空间任意取值都是正数
		}

		for (int i = 0;i < 3;++i)
		{
			t.setVertex(i, v[i].head<3>());//head<n>表示提取前n个元素
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);//领用drawline绘制三角形
	}

}
//屏幕空间光栅化
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
					//这里使用的插值还不是一般的插值方法，而是更近一步用到了透视校正（具体查看[GAMES101]现代计算机图形学课程总结4：重心坐标，作业2；https://happyfire.blog.csdn.net/article/details/100148540
					// 
					//其原理就是在只考虑xy2d平面的插值，得到属性值和在三维空间中得到的真正属性值不同，所以需要透视校正

					//w_reciprocal是经过透视校正后的深度值，但是由于作业框架中在viewport transform时，将z值映射到了near,far之间，还需做一步变幻，即第二次插值
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
	return (height -1- y) * width + x;//height-y是因为OpenCV的坐标系是反过来的
}

void rasterizer::set_pixel(const Vector3f& point, const Vector3f& color)
{
	if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
	{
		return;
	}
	//经过变幻后点的位置就是像素的位置，求传入的点的索引值，可以理解为二维数组中的第几个数，也就是二维像素中的第几个像素
	auto ind = (height -1- point.y()) * width + point.x();
	frame_buf[ind] = color;
}