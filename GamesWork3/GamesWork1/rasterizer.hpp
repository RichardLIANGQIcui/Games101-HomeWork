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

//�����õ���namespace�������ռ���������
namespace rst {

	//����һ�����滺���ö������
	enum class Buffers {
		Color = 1,
		Depth = 2
	};

	//ʹ����������������ʱչ��������Ч��
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

	//ע��ʹ�����½ṹ���Ŀ���ǽ���ͬ���Ե�id��װ����������ʹ��ʱŪ��

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

		//function�Ǹ�����������Vector3f��ʾ����ֵ��vertex_shader_payload��ʾ����
		//���������Ϊʲô��function����Ϊfunction���Ե����βκͷ���ֵ��ͬ�����⺯������ÿ����ɫ������function��ʽ��Ҳ���Ǹ�����
		//Ϊ�˵��ò�ͬ����ɫ����ʹ��function��ʽ
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

		map<int, vector<Vector3f>> pos_buf;//��һ��������Ϊ������Ԫ
		map<int, vector<Vector3i>> ind_buf;
		map<int, vector<Vector3f>> col_buf;
		map<int, vector<Vector3f>> nor_buf;

		//optional��;������������������һ������ֵ����Ϊ�ա����͵�Ӧ���龰�Ǻ�������ʱ��
		//���������������һ��������Ч����Ĭ�϶�����Ч����
		//���ö�����ɱ��ܸߣ���Դ����ȣ�������optional����һ���ն������Ч��
		std::optional<Texture> texture;


		function<Vector3f(fragment_shader_payload)> fragment_shader;
		function<Vector3f(vertex_shader_payload)> vertex_shader;

		vector<Vector3f> frame_buf;//��������ص���ɫ
		vector<float> depth_buf;
		//������ʹ�ô洢���ݽṹ
		std::vector<Eigen::Vector3f> frame_sample;
		std::vector<float> depth_sample;


		int get_index(int x, int y);

		int get_super_index(int x, int y);

		int width, height;

		int next_id = 0;
		int get_next_id() { return next_id++; }

	};
}
