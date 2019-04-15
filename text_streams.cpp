#ifdef _MSC_VER
#define _SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING
#endif

#include "text_streams.h"
#include <cwchar>
#include <cassert>
#include <stdexcept>
#include <cstring>
#include <codecvt>
#include <sstream>

template <class Facet = std::codecvt<wchar_t, char, std::mbstate_t>>
struct codecvt:Facet
{
	using Facet::Facet;
	~codecvt() {}
};

typedef codecvt<std::codecvt_byname<wchar_t, char, std::mbstate_t>> codecvt_byname;

const std::wstring text_istream::m_strUnknownResourceId = std::wstring(L"");

const std::wstring& text_istream::get_resource_id() const
{
	return m_strUnknownResourceId;
}

text_istream::int_type text_istream::streambuf::readone()
{
	const auto& conv = std::use_facet<std::codecvt<wchar_t, char, std::mbstate_t>>(this->getloc());
	wchar_t result;
	const char* from_end;
	wchar_t* to_end;

	auto cbTail = std::size_t(m_mb_buf_len - m_mb_buf_off);
	if (cbTail == 0)
	{
		if (m_mb_buf_len < sizeof(m_mb_buf))
			return traits_type::eof();
		m_mb_buf_off = std::size_t();
	}else
	{
		auto state_old = m_conv_state;
		auto status = conv.in(m_conv_state, &m_mb_buf[m_mb_buf_off], &m_mb_buf[m_mb_buf_len], from_end, &result, &result + 1, to_end);
		switch (status)
		{
		case std::codecvt_base::partial:
		case std::codecvt_base::ok:
			if (std::size_t(from_end - &m_mb_buf[0]) == m_mb_buf_off || to_end == &result)
			//see https://bugs.llvm.org/show_bug.cgi?id=13759
			{
				assert(m_mb_buf_off > 0);
				std::memmove(&m_mb_buf[0], &m_mb_buf[m_mb_buf_off], m_mb_buf_len - m_mb_buf_off);
				m_mb_buf_off = m_mb_buf_len - m_mb_buf_off;
				m_conv_state = state_old;
				break;
			}
			m_mb_buf_off = from_end - &m_mb_buf[0];
			return traits_type::to_int_type(result);
		case std::codecvt_base::error:
		case std::codecvt_base::noconv:
			return traits_type::eof();
		default:
			throw std::logic_error("Unexpected return value of codecvt::in");
		}
	}
	m_mb_buf_len = m_mb_buf_off + std::size_t(m_pBufImpl->sgetn(&m_mb_buf[m_mb_buf_off], std::streamsize(sizeof(m_mb_buf) - m_mb_buf_off)));
	auto status = conv.in(m_conv_state, &m_mb_buf[0], &m_mb_buf[m_mb_buf_len], from_end, &result, &result + 1, to_end);
	switch (status)
	{
	case std::codecvt_base::partial:
		if (from_end == &m_mb_buf[0])
			return traits_type::eof();
	case std::codecvt_base::ok:
		m_mb_buf_off = from_end - &m_mb_buf[0];
		return traits_type::to_int_type(result);
	case std::codecvt_base::error:
	case std::codecvt_base::noconv:
		return traits_type::eof();
	default:
		throw std::logic_error("Unexpected return value of codecvt::in");
	}
}

text_istream::pos_type text_istream::streambuf::seekoff(text_istream::off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which)
{
	if (off != 0)
		return pos_type(off_type(-1));
	if (dir == std::ios_base::cur)
		return off_type(impl_off_type(m_pBufImpl->pubseekoff(0, std::ios_base::cur, which))) - off_type(m_mb_buf_len - m_mb_buf_off);
	m_mb_buf_off = sizeof(m_mb_buf);
	m_mb_buf_len = sizeof(m_mb_buf);
	m_conv_state = std::mbstate_t();
	m_chLast = char_type();
	m_locator = locator{0u, 1u};
	m_prev_col = 0u;
	this->setg(&m_chLast, &m_chLast + 1, &m_chLast + 1);
	return pos_type(off_type(impl_off_type(m_pBufImpl->pubseekoff(0, dir, which))));
}

text_istream::pos_type text_istream::streambuf::seekpos(text_istream::pos_type pos, std::ios_base::openmode which)
{
	return pos == pos_type()?this->seekoff(off_type(), std::ios_base::beg, which):pos_type(off_type(-1));
}

text_istream::streambuf::int_type text_istream::streambuf::underflow()
{
	auto ch = readone();
	if (ch == traits_type::eof())
		return ch;
	else if (ch == traits_type::to_int_type(L'\n'))
	{
		m_prev_col = m_locator.col;
		m_locator.col = 0u;
		++m_locator.row;
	}else
		++m_locator.col;
	m_chLast = traits_type::to_char_type(ch);
	this->setg(&m_chLast, &m_chLast, &m_chLast + 1);
	return ch;
}

text_istream::streambuf::streambuf(std::streambuf* pBuf, TextEncoding encoding):m_pBufImpl(pBuf)
{
	this->setg(&m_chLast, &m_chLast + 1, &m_chLast + 1);
	this->set_encoding(encoding);
}

void text_istream::streambuf::set_encoding(TextEncoding encoding)
{
	switch (encoding)
	{
	case TextEncoding::UTF8:
#ifndef _WIN32
		try
		{
			this->pubimbue(std::locale(this->getloc(), "en_US.UTF-8", std::locale::ctype));
		}catch (std::runtime_error&)
		{
			throw std::invalid_argument("Locale \"en_US.UTF-8\" is not known to the system.");
		}
#else
		this->pubimbue(std::locale(this->getloc(), new std::codecvt_utf8<wchar_t>()));
#endif
		break;
	case TextEncoding::UTF16LE:
		this->pubimbue(std::locale(this->getloc(), new std::codecvt_utf16<wchar_t, 0x10ffff, std::codecvt_mode::little_endian>()));
		break;
	case TextEncoding::UTF16BE:
		this->pubimbue(std::locale(this->getloc(), new std::codecvt_utf16<wchar_t, 0x10ffff, std::codecvt_mode(0)>()));
		break;
	case TextEncoding::Default:
		this->pubimbue(std::locale(this->getloc(), "", std::locale::ctype));
		break;
	case TextEncoding::ANSI:
		this->pubimbue(std::locale(this->getloc(), std::locale::classic(), std::locale::ctype));
		break;
	case TextEncoding::Windows_1251:
		try
		{
			this->pubimbue(std::locale(this->getloc(), ".1251", std::locale::ctype));
		}catch (std::runtime_error&)
		{
			throw std::invalid_argument("Locale \".1251\" is not known to the system.");
		}
		break;
	}
}

text_istream::text_istream(std::istream& is, use_bom_t):std::wistream(nullptr), m_buf(is.rdbuf())
{
	if (is.fail())
	{
		this->setstate(std::ios_base::failbit);
		return;
	}
	auto pos_old = is.tellg();
	is.seekg(0, std::ios_base::end);
	auto cb = std::size_t(std::istream::off_type(is.tellg()));
	is.seekg(pos_old);
	if (cb > 0)
	{
		char ch0 = is.get();
		switch (ch0)
		{
		case '\xff':
			if (cb == 1)
				this->setfail();
			else
			{
				char ch1 = is.get();
				switch (ch1)
				{
				case '\xfe':
					if (cb == 2)
						m_buf.set_encoding(TextEncoding::UTF16LE);
					else
					{
						char ch2 = is.get();
						switch (ch2)
						{
						case 0: //UTF-32LE
							this->setfail();
							break;
						default:
							is.putback(ch2);
							m_buf.set_encoding(TextEncoding::UTF16LE);
						}
					}
					break;
				default:
					this->setfail();
				}
			}
			break;
		case '\xfe': //UTF-16BE
			if (cb < 1)
				this->setfail();
			else
			{
				char ch1 = is.get();
				switch (ch1)
				{
				case '\xff':
					m_buf.set_encoding(TextEncoding::UTF16BE);
					break;
				default:
					this->setfail();
				}
			}
			this->setfail();
			break;
		case '\xef':
			if (cb < 3)
				this->setfail();
			else
			{
				char ch1 = is.get();
				switch (ch1)
				{
				case '\xbb':
				{
					char ch2 = is.get();
					switch (ch2)
					{
					case '\xbf':
						break;
					default:
						this->setfail();
					}
					break;
				}
				default:
					this->setfail();
				}
			}
			break;
		case '0': //UTF-32BE ?
			this->setfail();
			break;
		default:
			is.putback(ch0);
		}
	}
	this->rdbuf(&m_buf);
}

text_istream& text_istream::operator=(text_istream&& right)
{
	m_buf = std::move(right.m_buf);
	this->std::wistream::operator=(std::move(right));
	auto state = this->rdstate();
	this->rdbuf(&m_buf);
	this->setstate(state);
	return *this;
}

resource_locator text_istream::get_resource_locator() const
{
	return resource_locator{
		this->get_locator().col,
		this->get_locator().row,
		std::wstring_convert<codecvt_byname>(new codecvt_byname("")).to_bytes(this->get_resource_id())};
}

const std::wstring& text_ifstream::get_resource_id() const
{
	return m_path;
}

text_ifstream& text_ifstream::operator=(text_ifstream&& right)
{
	this->text_istream::operator=(std::move(right));
	m_path = std::move(right.m_path);
	m_is = std::move(right.m_is);
	this->rdbuf()->rdbuf(m_is.rdbuf());
	return *this;
}

std::wstring text_ifstream::path_init(std::string_view path)
{
	return std::wstring_convert<codecvt_byname>(new codecvt_byname("")).from_bytes(path.data(), path.data() + path.size());
}

std::ifstream text_ifstream::stream_init(std::wstring_view path)
{
	return std::ifstream{std::wstring_convert<codecvt_byname>(new codecvt_byname("")
	).to_bytes(path.data(), path.data() + path.size()), std::ios_base::in};
}

