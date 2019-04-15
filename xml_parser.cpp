#include "xml_parser.h"

namespace xml
{
	text_istream& skip_whitespace(text_istream& is)
	{
		text_istream::int_type ch;
		while ((ch = is.get()) != text_istream::traits_type::eof())
		{
			if (std::iswspace(ch) == 0)
			{
				is.putback(text_istream::traits_type::to_int_type(ch));
				return is;
			}
		};
		return is;
	}
	tag::tag(text_istream& is)
	{
		auto is_state = istream_state(is);
		auto locale = is.getloc();
		if (xml::skip_whitespace(is).get() != L'<')
			throw xml_invalid_syntax(is.get_resource_locator());
		switch (auto curr = is.get())
		{
		case L'?':
			{
			const wchar_t header_xml[] = {L'x', L'm', L'l'};
				auto it_is = std::istreambuf_iterator<wchar_t>(is);
				for (auto val:header_xml)
					if (val != *(it_is++))
						throw xml_invalid_syntax(is.get_resource_locator());
				if (xml::skip_whitespace(is).eof())
					throw xml_invalid_syntax(is.get_resource_locator());
				while (true)
				{
					curr = is.get();
					if (curr == text_istream::traits_type::eof())
						throw xml_invalid_syntax(is.get_resource_locator());
					if (curr == L'?')
					{
						curr = is.get();
						if (curr != L'>')
							throw xml_invalid_syntax(is.get_resource_locator());
						break;
					}
					is.putback(char(curr));
					auto attribute = get_xml_word(is);
					if (is.eof() || (attribute != L"version" && attribute != L"encoding"))
						throw xml_invalid_syntax(is.get_resource_locator());
					if (xml::skip_whitespace(is).eof() || is.get() != L'=')
						throw xml_invalid_syntax(is.get_resource_locator());
					auto value = get_string_in_quotes(is);
					if (!m_mpAttributes.emplace(attribute, value).second)
						throw xml_attribute_already_specified(is.get_resource_locator(), attribute);
				}
				m_fIsHeader = true;
				return;
			}
		case L'/':
		{
			m_strTag = get_xml_word(is);
			if (is.eof() || m_strTag.empty())
				throw xml_invalid_syntax(is.get_resource_locator());
			if (xml::skip_whitespace(is).get() != L'>')
				throw xml_invalid_syntax(is.get_resource_locator());
			m_fIsClosing = true;
			return;
		}
		case L'!':
			if (is.get() != L'-' || is.get() != L'-')
				throw xml_invalid_syntax(is.get_resource_locator());
			while (true)
			{
				while (is.get() != L'-')
				{
					if (is.eof())
						throw xml_invalid_syntax(is.get_resource_locator());
				}
				if (is.get() == L'-' && is.get() == L'>')
					break;
				if (is.eof())
					throw xml_invalid_syntax(is.get_resource_locator());
			}
			m_strTag = std::wstring(L"<comment>");
			m_fIsComment = true;
			return;
		case text_istream::traits_type::eof():
			throw xml_invalid_syntax(is.get_resource_locator());
		default:
			is.putback(curr);
			m_strTag = get_xml_word(is);
			if (is.eof() || m_strTag.empty())
				throw xml_invalid_syntax(is.get_resource_locator());
			while ((curr = xml::skip_whitespace(is).peek()) != L'>')
			{
				if (curr == text_istream::traits_type::eof())
					throw xml_invalid_syntax(is.get_resource_locator());
				if (curr == L'/')
				{
					is.get();
					if (xml::skip_whitespace(is).get() != L'>')
						throw xml_invalid_syntax(is.get_resource_locator());
					m_fIsUnary = true;
					return;
				}
				auto attribute = get_xml_word(is);
				if (xml::skip_whitespace(is).get() != L'=')
					throw xml_invalid_syntax(is.get_resource_locator());
				auto value = get_string_in_quotes(is);
				if (!m_mpAttributes.emplace(attribute, value).second)
					throw xml_attribute_already_specified(is.get_resource_locator(), attribute);
			}
			is.get();
			return;
		};
	}
	tag::tag(text_istream& is, nothrow_t)
	{
		try
		{
			*this = tag(is);
		}catch(xml_error&)
		{
			is.setstate(is.rdstate() | std::ios_base::failbit);
			if (is.exceptions() & std::ios_base::failbit)
				throw std::ios_base::failure("tag::tag");
		}
	}
	std::wstring tag::get_xml_word(text_istream& is)
	{
		std::wostringstream os;
		text_istream::traits_type::int_type chCurrent;
		xml::skip_whitespace(is);
		while ((chCurrent = is.get()) != text_istream::traits_type::eof() && (std::iswalnum(chCurrent) || chCurrent == text_istream::traits_type::to_int_type(L'_')))
			os.put(text_istream::traits_type::to_char_type(chCurrent));
		if (chCurrent != text_istream::traits_type::eof())
			is.putback(text_istream::traits_type::to_char_type(chCurrent));
		return os.str();
	}
	std::wstring tag::get_string_in_quotes(text_istream& is)
	{
		std::wostringstream os;
		text_istream::traits_type::int_type chCurrent;
		if ((chCurrent = xml::skip_whitespace(is).get()) == text_istream::traits_type::eof() || chCurrent != text_istream::traits_type::to_int_type(L'\"'))
			throw xml_invalid_syntax(is.get_resource_locator());
		while ((chCurrent = is.get()) != text_istream::traits_type::eof() && chCurrent != text_istream::traits_type::to_int_type(L'\"'))
			os.put(text_istream::traits_type::to_char_type(chCurrent));
		return os.str();
	}
} //namespace xml
