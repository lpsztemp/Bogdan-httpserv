#include "face.h"

face_t& face_t::operator=(const face_t& right)
{
	if (this != &right)
	{
		if (m_vertex_cnt <= VERTICES_IN_STACK)
		{
			if (right.m_vertex_cnt <= VERTICES_IN_STACK)
				std::copy(right.begin(), right.end(), m_pVertices);
			else
			{
				m_vertices_big = new point_t [right.m_vertex_cnt];
				std::copy(right.begin(), right.end(), m_vertices_big);
				m_pVertices = m_vertices_big;
			}
		}else if (right.m_vertex_cnt <= VERTICES_IN_STACK)
		{
			delete [] m_vertices_big;
			std::copy(right.begin(), right.end(), m_vertices_small);
			m_pVertices = m_vertices_small;
		}else
		{
			auto buf = std::make_unique<point_t[]>(right.m_vertex_cnt);
			std::copy(right.begin(), right.end(), &buf[0]);
			delete [] m_vertices_big;
			m_pVertices = buf.release();
		}
		m_vertex_cnt = right.m_vertex_cnt;
	}
	return *this;
}

face_t& face_t::operator=(face_t&& right) noexcept
{
	if (this != &right)
	{
		if (m_vertex_cnt <= VERTICES_IN_STACK)
		{
			if (right.m_vertex_cnt <= VERTICES_IN_STACK)
				std::copy(right.begin(), right.end(), m_pVertices);
			else
			{
				m_pVertices = m_vertices_big = right.m_vertices_big;
				right.m_pVertices = right.m_vertices_small;
			}
		}else if (right.m_vertex_cnt <= VERTICES_IN_STACK)
		{
			delete [] m_vertices_big;
			std::copy(right.begin(), right.end(), m_vertices_small);
			m_pVertices = m_vertices_small;
		}else
		{
			delete [] m_vertices_big;
			m_pVertices = m_vertices_big = right.m_vertices_big;
			right.m_pVertices = right.m_vertices_small;
		}
		m_vertex_cnt = right.m_vertex_cnt;
		right.m_vertex_cnt = 0;
	}
	return *this;
}
