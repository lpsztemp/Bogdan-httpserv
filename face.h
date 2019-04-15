#include "basedefs.h"
#include "point.h"
#include <iterator>
#include <type_traits>
#include <cstdint>

#ifndef CONVERTERS_FACE_H
#define CONVERTERS_FACE_H

struct face_t
{
	typedef point_t value_type;
	typedef std::size_t size_type;
	typedef std::ptrdiff_t difference_type;
	typedef point_t& reference;
	typedef const point_t& const_reference;
	typedef point_t* pointer;
	typedef const point_t* const_pointer;
	typedef point_t* iterator;
	typedef const point_t* const_iterator;
	typedef std::reverse_iterator<iterator> reverse_iterator;
	typedef std::reverse_iterator<const_iterator> const_reverse_iterator;


	template <class VertexRandomAccessIterator,
		class = std::enable_if_t<std::is_base_of_v<typename std::iterator_traits<VertexRandomAccessIterator>::iterator_category, std::random_access_iterator_tag>>>
	face_t(VertexRandomAccessIterator itBegin, VertexRandomAccessIterator itEnd):face_t(itBegin, itEnd, size_type(itEnd - itBegin)) {}
	template <class Container, class = std::enable_if_t<std::is_constructible_v<face_t,
		decltype(std::begin(std::declval<const Container&>())),
		decltype(std::end(std::declval<const Container&>())),
		decltype(std::declval<const Container&>().size())>>>
	face_t(const Container& cont):face_t(std::begin(cont), std::end(cont), cont.size()) {}
	template <class T, std::size_t N>
	face_t(const T (&arr)[N]):face_t(&arr[0], &arr[1], N) {}
	template <class T>
	face_t(std::initializer_list<T> lst):face_t(std::begin(lst), std::end(lst), lst.size()) {}
	template <class VertexInputIterator, class = std::enable_if_t<std::is_convertible_v<typename std::iterator_traits<VertexInputIterator>::value_type, point_t>>>
	face_t(VertexInputIterator itBegin, VertexInputIterator itEnd, size_type size):m_vertex_cnt(size)
	{
		if (m_vertex_cnt <= VERTICES_IN_STACK)
			m_pVertices = &m_vertices_small[0];
		else
		{
			m_vertices_big = new point_t[m_vertex_cnt];
			m_pVertices = m_vertices_big;
		}
		std::copy(itBegin, itEnd, &m_pVertices[0]);
	}
	face_t() = default;
	inline face_t(const face_t& right)
	{
		*this = right;
	}
	inline face_t(face_t&& right) noexcept
	{
		*this = std::move(right);
	}
	face_t& operator=(const face_t& right);
	face_t& operator=(face_t&& right) noexcept;
	inline const point_t& operator[](size_type n) const noexcept
	{
		return m_pVertices[n];
	}
	inline point_t& operator[](size_type n) noexcept
	{
		return m_pVertices[n];
	}
	inline const point_t& at(size_type n) const noexcept
	{
		return m_pVertices[n];
	}
	inline point_t& at(size_type n) noexcept
	{
		return m_pVertices[n];
	}
	inline size_type size() const noexcept
	{
		return m_vertex_cnt;
	}
	inline bool empty() const noexcept
	{
		return this->size() == 0;
	}
	inline iterator begin() noexcept
	{
		return &m_pVertices[0];
	}
	inline const_iterator cbegin() const noexcept
	{
		return &m_pVertices[0];
	}
	inline iterator end() noexcept
	{
		return &m_pVertices[this->size()];
	}
	inline const_iterator cend() const noexcept
	{
		return &m_pVertices[this->size()];
	}

	inline const_iterator begin() const noexcept
	{
		return this->cbegin();
	}
	inline const_iterator end() const noexcept
	{
		return this->cend();
	}
	inline reverse_iterator rbegin() noexcept
	{
		return std::make_reverse_iterator(this->end());
	}
	inline const_reverse_iterator rbegin() const noexcept
	{
		return std::make_reverse_iterator(this->end());
	}
	inline const_reverse_iterator crbegin() const noexcept
	{
		return std::make_reverse_iterator(this->cend());
	}
	inline reverse_iterator rend() noexcept
	{
		return std::make_reverse_iterator(this->begin());
	}
	inline const_reverse_iterator rend() const noexcept
	{
		return std::make_reverse_iterator(this->begin());
	}
	inline const_reverse_iterator crend() const noexcept
	{
		return std::make_reverse_iterator(this->cbegin());
	}
	inline ~face_t() noexcept
	{
		if (m_vertex_cnt > VERTICES_IN_STACK)
			delete [] m_vertices_big;
	}
private:
	size_type m_vertex_cnt = 0;
	constexpr static size_type VERTICES_IN_STACK = 3;
	union
	{
		point_t* m_vertices_big;
		point_t m_vertices_small[VERTICES_IN_STACK];
	};
	point_t* m_pVertices = m_vertices_small;
};

#endif //CONVERTERS_FACE_H
