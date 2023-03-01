#pragma once

#include "Triangle.hpp"
#include <algorithm>
#include <Eigen/Eigen>
#include <map>


using namespace std;
using namespace Eigen;

//以下用到了namespace的命名空间包括多个类
namespace rst {

	//定义一个保存缓冲的枚举类型
	enum class Buffers {
		Color = 1,
		Depth = 2
	};

	//使用内敛函数，编译时展开，增加效率
	inline Buffers operator|(Buffers a, Buffers b)
	{
		return Buffers((int)a | (int)b);
	}

	inline Buffers operator&(Buffers a, Buffers b)
	{
		return Buffers((int)a & (int)b);
	}

	enum class Primitive
	{
		Line,
		Triangle
	};

	//注意使用以下结构体的目的是将不同属性的id封装起来，避免使用时弄混

	struct pos_buf_id
	{
		int pos_id = 0;
	};

	struct ind_buf_id
	{
		int ind_id = 0;
	};

	struct col_buf_id
	{
		int col_id = 0;
	};

	class rasterizer
	{
	public:
		rasterizer(int w, int h);
		pos_buf_id load_positions(const vector<Vector3f>& positions);
		ind_buf_id load_indices(const vector<Vector3i>& indices);
		col_buf_id load_colors(const vector<Vector3f>& colors);

		void set_model(const Matrix4f& m);
		void set_view(const Matrix4f& v);
		void set_projection(const Matrix4f& p);

		void set_pixel(const Vector3f& point, const Vector3f& color);

		void clear(Buffers buff);

		void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer,Primitive type);

		vector<Vector3f>& frame_buffer() { return frame_buf; }

	private:
		void rasterize_triangle(const Triangle& t);

	private:
		Matrix4f model;
		Matrix4f view;
		Matrix4f projection;

		map<int, vector<Vector3f>> pos_buf;//以一个三角形为基本单元
		map<int, vector<Vector3i>> ind_buf;
		map<int, vector<Vector3f>> col_buf;

		vector<Vector3f> frame_buf;//存的是像素的颜色
		vector<float> depth_buf;
		int get_index(int x, int y);

		int width, height;

		int next_id = 0;
		int get_next_id() { return next_id++; }

	};
}
