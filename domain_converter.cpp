#include "domain_converter.h"
#include "xml_parser.h"

void skip_xml_domain_data(text_istream& is) //"is" is associated with the first character after the closing '>'
{
	int level = 1;
	wchar_t linebuf[100];
	do
	{
		auto p0 = text_istream::off_type(is.tellg());
		is.get(linebuf, sizeof(linebuf), L'<');
		if (is.eof())
			throw xml_invalid_syntax(is.get_resource_locator());
		auto cb = text_istream::off_type(is.tellg()) - p0;
		if (cb == sizeof(linebuf) - sizeof(wchar_t))
			continue;
		auto tag = xml::tag(is);
		if (tag.name() != L"domain")
			continue;
		if (!tag.is_closing_tag())
		{
			++level;
			continue;
		}
		--level;
	}while (level != 0);
}