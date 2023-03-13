#pragma once

#include <algorithm>
#include <Eigen/Eigen>
#include <map>
#include <optional>
#include "Triangle.hpp"
#include "Shader.hpp"
#include "global.hpp"

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
		col_buf_id load_normals(const vector<Vector3f>& normals);

		void set_model(const Matrix4f& m);
		void set_view(const Matrix4f& v);
		void set_projection(const Matrix4f& p);

		void set_texture(Texture tex) { texture = tex; }

		//function是个函数容器，Vector3f表示返回值，vertex_shader_payload表示参数
		//这里解释下为什么用function，因为function可以调用形参和返回值相同的任意函数，而每个着色器都是function形式，也就是个函数
		//为了调用不同的着色器才使用function形式
		void set_vertex_shader(function<Vector3f(vertex_shader_payload)> vert_shader);
		void set_fragment_shader(function<Vector3f(fragment_shader_payload)> frag_shader);

		void set_pixel(const Vector2i& point, const Vector3f& color);

		void clear(Buffers buff);

		void draw(vector<Triangle*>& TriangleList);


		vector<Vector3f>& frame_buffer() { return frame_buf; }

	private:
		void rasterize_triangle(const Triangle& t,const array<Vector3f,3>& world_pos);
		void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);

	private:
		Matrix4f model;
		Matrix4f view;
		Matrix4f projection;

		int normal_id = -1;

		map<int, vector<Vector3f>> pos_buf;//以一个三角形为基本单元
		map<int, vector<Vector3i>> ind_buf;
		map<int, vector<Vector3f>> col_buf;
		map<int, vector<Vector3f>> nor_buf;

		//optional用途如其名，它可以容纳一个对象值或是为空。典型的应用情景是函数调用时，
		//如需根据条件返回一个对象（有效）或默认对象（无效），
		//若该对象构造成本很高（资源分配等），可用optional返回一个空对象，提高效率
		std::optional<Texture> texture;


		function<Vector3f(fragment_shader_payload)> fragment_shader;
		function<Vector3f(vertex_shader_payload)> vertex_shader;

		vector<Vector3f> frame_buf;//存的是像素的颜色
		vector<float> depth_buf;
		//超采样使用存储数据结构
		std::vector<Eigen::Vector3f> frame_sample;
		std::vector<float> depth_sample;


		int get_index(int x, int y);

		int get_super_index(int x, int y);

		int width, height;

		int next_id = 0;
		int get_next_id() { return next_id++; }

	};
}
