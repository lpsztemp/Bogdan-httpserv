#include <stdexcept>
#include <sstream>
#include <utility>
#include <string_view>

#ifndef XML2BIN_EXCEPTIONS_H_
#define XML2BIN_EXCEPTIONS_H_

inline std::string make_exception_message(const std::string& what)
{
	return what;
}

inline std::string make_exception_message(std::string&& what)
{
	return std::move(what);
}

std::string make_exception_message(const std::wstring& what);

inline std::string make_exception_message(std::string_view what)
{
	return std::string(what);
}

inline std::string make_exception_message(std::wstring_view what)
{
	return make_exception_message(std::wstring(what));
}

inline std::string make_exception_message(const char* what)
{
	return std::string(what);
}

inline std::string make_exception_message(const wchar_t* what)
{
	return make_exception_message(std::wstring(what));
}

template <class Locator, class WhatString>
std::string make_exception_message(const Locator& locator, WhatString&& what)
{
	std::ostringstream os;
	os << locator << ": " << make_exception_message(std::forward<WhatString>(what));
	return os.str();
}

struct xml_error:std::runtime_error
{
	using std::runtime_error::runtime_error;
};

struct xml_attribute_already_specified:xml_error
{
	inline xml_attribute_already_specified():xml_error(make_exception_message("Attribute is not unique")) {}
	inline xml_attribute_already_specified(std::wstring_view attr):xml_error(make_exception_message(form_what(attr))) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline xml_attribute_already_specified(const ResourceLocatorT& resource):xml_error(make_exception_message(resource, "Attribute is not unique")) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline xml_attribute_already_specified(const ResourceLocatorT& resource, std::wstring_view attr):xml_error(make_exception_message(resource, form_what(attr))) {}
private:
	inline static std::wstring form_what(std::wstring_view attr)
	{
		std::wostringstream os;
		os << L"Attribute " << attr << L" was not unique";
		return os.str();
	}
};

struct xml_invalid_syntax:xml_error
{
	inline xml_invalid_syntax():xml_error(make_exception_message("Invalid xml syntax")) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline xml_invalid_syntax(const ResourceLocatorT& resource):xml_error(make_exception_message(resource, "Invalid xml syntax")) {}
};

struct xml_model_error:xml_error
{
	using xml_error::xml_error;
};

struct improper_xml_tag:xml_model_error
{
	inline improper_xml_tag():xml_model_error(make_exception_message("Improper XML tag")) {}
	inline improper_xml_tag(std::wstring_view tag):xml_model_error(make_exception_message(form_what(tag))) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline improper_xml_tag(const ResourceLocatorT& resource):xml_model_error(make_exception_message(resource, "Improper XML tag")) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline improper_xml_tag(const ResourceLocatorT& resource, std::wstring_view tag):xml_model_error(make_exception_message(resource, form_what(tag))) {}
private:
	inline static std::wstring form_what(std::wstring_view tag)
	{
		std::wostringstream ss;
		ss << L"Improper XML tag \"" << tag << L"\"";
		return ss.str();
	}
};

struct xml_attribute_not_found:xml_model_error
{
	inline xml_attribute_not_found():xml_model_error(make_exception_message("Attribute is not found")) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline xml_attribute_not_found(const ResourceLocatorT& resource):xml_model_error(make_exception_message(resource, "Attribute is not found")) {}
	inline xml_attribute_not_found(std::wstring_view attr):xml_model_error(make_exception_message(form_what(attr))) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline xml_attribute_not_found(const ResourceLocatorT& resource, std::wstring_view attr):xml_model_error(make_exception_message(resource, form_what(attr))) {}
private:
	inline static std::wstring form_what(std::wstring_view attr)
	{
		std::wostringstream os;
		os << L"Attribute " << attr << L" was not found";
		return os.str();
	}
};

struct xml_tag_not_found:xml_model_error
{
	inline xml_tag_not_found():xml_model_error(make_exception_message("Tag is not found")) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline xml_tag_not_found(const ResourceLocatorT& resource):xml_model_error(make_exception_message(resource, "Tag is not found")) {}
	inline xml_tag_not_found(std::wstring_view tag):xml_model_error(make_exception_message(form_what(tag))) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline xml_tag_not_found(const ResourceLocatorT& resource, std::wstring_view tag):xml_model_error(make_exception_message(resource, form_what(tag))) {}
private:
	inline static std::wstring form_what(std::wstring_view attr)
	{
		std::wostringstream os;
		os << L"Tag " << attr << L" was not found";
		return os.str();
	}
};

struct invalid_xml_model:xml_model_error
{
	inline invalid_xml_model():xml_model_error(make_exception_message("Invalid xml data")) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline invalid_xml_model(const ResourceLocatorT& resource):xml_model_error(make_exception_message(resource, "Invalid xml data")) {}
	inline invalid_xml_model(std::wstring_view strWhat):xml_model_error(make_exception_message(strWhat)) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline invalid_xml_model(const ResourceLocatorT& resource, std::wstring_view strWhat):xml_model_error(make_exception_message(resource, strWhat)) {}
};

struct ambiguous_specification:xml_model_error
{
	inline ambiguous_specification():xml_model_error(make_exception_message("Ambiguous model specification")) {}
	inline ambiguous_specification(std::wstring_view xml_entity):xml_model_error(make_exception_message(form_what(xml_entity))) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline ambiguous_specification(const ResourceLocatorT& resource):xml_model_error(make_exception_message(resource, "Ambiguous model specification")) {}
	template <class ResourceLocatorT, class = std::enable_if_t<!std::is_convertible_v<const ResourceLocatorT&, std::wstring_view>>>
	inline ambiguous_specification(const ResourceLocatorT& resource, std::wstring_view xml_entity):xml_model_error(make_exception_message(resource, form_what(xml_entity))) {}
private:
	template <class XmlEntityString>
	inline static std::wstring form_what(const XmlEntityString& xml_entity)
	{
		std::wostringstream ss;
		ss << L"Ambiguous " << xml_entity;
		return ss.str();
	}
};

#endif //XML2BIN_EXCEPTIONS_H_
