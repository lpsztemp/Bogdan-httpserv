#include <map>
#include <string>
#include <istream>
#include <stdexcept>
#include <sstream>
#include <tuple>
#include <algorithm>
#include <iterator>
#include <locale>
#include "text_streams.h"
#include <cwctype>
#include "xml_exceptions.h"

#ifndef IMPL_XML_PARSER_H_
#define IMPL_XML_PARSER_H_

namespace xml
{
	namespace Implementation
	{
		template <int (*is_f)(std::wint_t /*char, const std::locale&*/)>
		struct is_functor
		{
			inline bool operator()(std::wint_t ch) const
			{
				return is_f(ch);
			}
		};
	}
	typedef Implementation::is_functor<&std::iswspace> isspace_t;
	typedef Implementation::is_functor<&std::iswdigit> isdigit_t;
	typedef Implementation::is_functor<&std::iswprint> isprint_t;

	text_istream& skip_whitespace(text_istream& is);

	class tag
	{
		std::wstring m_strTag;
		std::map<std::wstring, std::wstring> m_mpAttributes;
		bool m_fIsComment = false;
		bool m_fIsClosing = false;
		bool m_fIsUnary = false;
		bool m_fIsHeader = false;
	public:
		tag() = default;
		//On input: "is" corresponds to a position before or equal to the position of the opening angle bracket '<'.
		// If there are whitespace symbols before the bracket, they are skipped. If a non-whitespace symbol, that is not the left angle bracket,
		// is encountered, xml_invalid_syntax exception is generated.
		//On output "is" corresponds to the first symbol following the closing angle bracket '>'
		//If a comment is encountered, on output "is" is associated with the first symbol after the "-->" sequence
		explicit tag(text_istream& is);
	private:
		enum class nothrow_t {nothrow};
	public:
		static constexpr nothrow_t nothrow = nothrow_t::nothrow;
		tag(text_istream& is, nothrow_t);
		inline bool is_closing_tag() const
		{
			return m_fIsClosing;
		}
		inline bool is_unary_tag() const
		{
			return m_fIsUnary;
		}
		inline bool is_comment() const
		{
			return m_fIsComment;
		}
		//returns tag string
		//for comment returns the "<comment>" string
		inline const std::wstring& name() const
		{
			return m_strTag;
		}
		inline bool is_header() const
		{
			return m_fIsHeader;
		}
		template <class AttributeName>
		const std::wstring& attribute(const AttributeName& attr) const
		{
			static const std::wstring strDefault;
			auto it = m_mpAttributes.find(attr);
			if (it == m_mpAttributes.end())
				return strDefault;
			return it->second;
		}
	private:
		static std::wstring get_xml_word(text_istream& is);
		static std::wstring get_string_in_quotes(text_istream& is); //quotes are extracted from the stream and discarded

		class istream_state
		{
			std::wistream *m_pIs;
			std::wistream::iostate m_prev;
		public:
			inline istream_state(std::wistream& is, std::wistream::iostate newmask = std::wistream::iostate()):m_pIs(&is), m_prev(m_pIs->exceptions())
			{
				m_pIs->exceptions(newmask);
			}
			inline ~istream_state()
			{
				m_pIs->exceptions(m_prev);
			}
		};
	};

	//is must correspond to a position to read from. Leading white spaces will be ignored. After the value is read, the following closing XML
	//tag will be read, ignoring any preceding white space characters, and, if tag_name is not empty, will be checked against tag_name to match it.
	template <class T>
	auto get_tag_value(text_istream& is, std::wstring_view tag_name = std::string_view())
	-> std::enable_if_t<std::is_arithmetic_v<T>, T>
	{
		T val;
		is >> val;
		auto cl_tag = tag(is);
		if (!cl_tag.is_closing_tag() || (!tag_name.empty() && tag_name != cl_tag.name()))
			throw improper_xml_tag(is.get_resource_locator(), cl_tag.name());
		return val;
	}

	template <class T>
	auto get_tag_value(text_istream& is, std::wstring_view tag_name = std::wstring_view(), bool fDiscardBoundingSpaces = true)
	-> std::enable_if_t<std::is_same_v<std::basic_string<text_istream::char_type, text_istream::traits_type, typename T::allocator_type>, T>, T>
	{
		std::basic_string<text_istream::char_type, text_istream::traits_type, typename T::allocator_type> str;
		if (fDiscardBoundingSpaces)
			xml::skip_whitespace(is);
		text_istream::traits_type::int_type curr;
		while ((curr = is.get()) != text_istream::traits_type::eof())
		{
			if (curr == text_istream::traits_type::to_int_type(L'<'))
			{
				is.putback(text_istream::traits_type::to_char_type(curr));
				if (fDiscardBoundingSpaces)
				{
					auto it = std::find_if_not(str.rbegin(), str.rend(), xml::isspace_t());
					str.erase(it.base(), str.end());
				}
				auto cl_tag = tag(is);
				if (!cl_tag.is_closing_tag() || (!tag_name.empty() && tag_name != cl_tag.name()))
					throw improper_xml_tag(is.get_resource_locator(), cl_tag.name());
				return str;
			}
			str.append(1, text_istream::traits_type::to_char_type(curr));
		}
		throw xml_invalid_syntax(is.get_resource_locator());
	}

	template <class T>
	auto get_tag_value(text_istream& is, const tag& xml_tag)
	{
		if (xml_tag.is_closing_tag() || xml_tag.is_unary_tag())
			throw improper_xml_tag(is.get_resource_locator(), xml_tag.name());
		return get_tag_value<T>(is, xml_tag.name());
	}

} //namespace xml

#endif //IMPL_XML_PARSER_H_
