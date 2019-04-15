#include "xml_exceptions.h"
#include <codecvt>
#include <locale>

template <class Facet = std::codecvt<wchar_t, char, std::mbstate_t>>
struct codecvt:Facet
{
	using Facet::Facet;
	~codecvt() {}
};
typedef codecvt<std::codecvt_byname<wchar_t, char, std::mbstate_t>> codecvt_byname;

std::string make_exception_message(const std::wstring& what)
{
	return std::wstring_convert<codecvt_byname>(new codecvt_byname("")).to_bytes(what);
}
