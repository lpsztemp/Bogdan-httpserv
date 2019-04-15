#include <type_traits>
#include <utility>
#include <string_view>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <optional>
#include "basedefs.h"
#include "binary_streams.h"
#include "xml_parser.h"

#ifndef XML2BIN_DOMAIN_CONVERTER_H_
#define XML2BIN_DOMAIN_CONVERTER_H_

struct domain_datum
{
	typedef std::uint8_t value_type, *pointer, &reference;
	typedef const std::uint8_t *const_pointer, &const_reference;
	typedef std::size_t size_type;
	typedef std::ptrdiff_t difference_type;
	typedef std::vector<std::uint8_t>::iterator iterator;
	typedef std::vector<std::uint8_t>::const_iterator const_iterator;
	typedef std::vector<std::uint8_t>::reverse_iterator reverse_iterator;
	typedef std::vector<std::uint8_t>::const_reverse_iterator const_reverse_iterator;

	inline pointer data() {return m_strg.data();}
	inline const_pointer data() const {return m_strg.data();}
	inline size_type size() const {return m_strg.size();}
	inline bool empty() const {return m_strg.empty();}

	domain_datum() = default;
	domain_datum(const domain_datum&) = default;
	domain_datum(domain_datum&&) = default;
	domain_datum& operator=(const domain_datum&) = default;
	domain_datum& operator=(domain_datum&&) = default;
	inline domain_datum(const std::vector<std::uint8_t>& vec):m_strg(vec) {}
	inline domain_datum(std::vector<std::uint8_t>&& vec):m_strg(std::move(vec)) {}
private:
	std::vector<std::uint8_t> m_strg;
};

typedef std::map<std::string, domain_datum> domain_data_map;

struct IDomainConverter
{
	virtual bool model_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) = 0;
	virtual bool poly_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) = 0;
	virtual bool face_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) = 0;
	virtual bool source_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) = 0;
	virtual bool plain_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) = 0;

	virtual std::optional<domain_datum> constant_face_domain_data(std::string_view DomainName, ConstantDomainDataId id) = 0;
	virtual domain_data_map constant_face_domain_data(ConstantDomainDataId id) = 0;
	virtual std::optional<domain_datum> constant_poly_domain_data(std::string_view DomainName, ConstantDomainDataId id) = 0;
	virtual domain_data_map constant_poly_domain_data(ConstantDomainDataId id) = 0;
	virtual std::optional<domain_datum> constant_source_domain_data(std::string_view DomainName, ConstantDomainDataId id) = 0;
	virtual domain_data_map constant_source_domain_data(ConstantDomainDataId id) = 0;
	virtual std::optional<domain_datum> constant_plain_domain_data(std::string_view DomainName, ConstantDomainDataId id) = 0;
	virtual domain_data_map constant_plain_domain_data(ConstantDomainDataId id) = 0;

	virtual ~IDomainConverter() {}
};

//converter for a domain data must have the interface:
/*
struct
{
	//MANDATORY
	static const std::string& domain_name();

	//OPTIONAL
	void model_domain_data(tag, is, os);
	void poly_domain_data(tag, is, os);
	void face_domain_data(tag, is, os);
	void source_domain_data(tag, is, os);
	void plain_domain_data(tag, is, os);

	//OPTIONAL-HGT
	void constant_face_domain_data(ConstantDomainDataId id, binary_ostream& os);
	void constant_poly_domain_data(ConstantDomainDataId id, binary_ostream& os);
};
*/

void skip_xml_domain_data(text_istream& is);

template <class T, class = void> struct has_model_domain_data:std::false_type {};
template <class T> struct has_model_domain_data<T, std::void_t<decltype(std::declval<T>().model_domain_data(std::declval<const xml::tag&>(), std::declval<text_istream&>(), std::declval<binary_ostream&>()))>>:std::true_type {};
template <class T, class = void> struct has_poly_domain_data:std::false_type {};
template <class T> struct has_poly_domain_data<T, std::void_t<decltype(std::declval<T>().poly_domain_data(std::declval<const xml::tag&>(), std::declval<text_istream&>(), std::declval<binary_ostream&>()))>>:std::true_type {};
template <class T, class = void> struct has_face_domain_data:std::false_type {};
template <class T> struct has_face_domain_data<T, std::void_t<decltype(std::declval<T>().face_domain_data(std::declval<const xml::tag&>(), std::declval<text_istream&>(), std::declval<binary_ostream&>()))>>:std::true_type {};
template <class T, class = void> struct has_source_domain_data:std::false_type {};
template <class T> struct has_source_domain_data<T, std::void_t<decltype(std::declval<T>().source_domain_data(std::declval<const xml::tag&>(), std::declval<text_istream&>(), std::declval<binary_ostream&>()))>>:std::true_type {};
template <class T, class = void> struct has_plain_domain_data:std::false_type {};
template <class T> struct has_plain_domain_data<T, std::void_t<decltype(std::declval<T>().plain_domain_data(std::declval<const xml::tag&>(), std::declval<text_istream&>(), std::declval<binary_ostream&>()))>>:std::true_type {};
template <class T, class = void> struct has_constant_face_domain_data:std::false_type {};
template <class T> struct has_constant_face_domain_data<T, std::void_t<decltype(std::declval<T>().constant_face_domain_data(std::declval<ConstantDomainDataId>(), std::declval<binary_ostream&>()))>>:std::true_type {};
template <class T, class = void> struct has_constant_poly_domain_data:std::false_type {};
template <class T> struct has_constant_poly_domain_data<T, std::void_t<decltype(std::declval<T>().constant_poly_domain_data(std::declval<ConstantDomainDataId>(), std::declval<binary_ostream&>()))>>:std::true_type {};
template <class T, class = void> struct has_constant_source_domain_data:std::false_type {};
template <class T> struct has_constant_source_domain_data<T, std::void_t<decltype(std::declval<T>().constant_source_domain_data(std::declval<ConstantDomainDataId>(), std::declval<binary_ostream&>()))>>:std::true_type {};
template <class T, class = void> struct has_constant_plain_domain_data:std::false_type {};
template <class T> struct has_constant_plain_domain_data<T, std::void_t<decltype(std::declval<T>().constant_plain_domain_data(std::declval<ConstantDomainDataId>(), std::declval<binary_ostream&>()))>>:std::true_type {};

template <class Convert>
auto convert_model_domain_data(Convert& conv, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) -> std::enable_if_t<has_model_domain_data<Convert&>::value, bool>
{
	conv.model_domain_data(domain_opening_tag, is, os);
	return true;
}
template <class Convert>
auto convert_model_domain_data(Convert&, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream&) -> std::enable_if_t<!has_model_domain_data<Convert&>::value, bool>
{
	skip_xml_domain_data(is);
	return false;
}
template <class Convert>
auto convert_poly_domain_data(Convert& conv, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) -> std::enable_if_t<has_poly_domain_data<Convert&>::value, bool>
{
	conv.poly_domain_data(domain_opening_tag, is, os);
	return true;
}
template <class Convert>
auto convert_poly_domain_data(Convert&, const xml::tag&, text_istream& is, binary_ostream&) -> std::enable_if_t<!has_poly_domain_data<Convert&>::value, bool>
{
	skip_xml_domain_data(is);
	return false;
}
template <class Convert>
auto convert_face_domain_data(Convert& conv, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) -> std::enable_if_t<has_face_domain_data<Convert&>::value, bool>
{
	conv.face_domain_data(domain_opening_tag, is, os);
	return true;
}
template <class Convert>
auto convert_face_domain_data(Convert&, const xml::tag&, text_istream& is, binary_ostream&) -> std::enable_if_t<!has_face_domain_data<Convert&>::value, bool>
{
	skip_xml_domain_data(is);
	return false;
}

template <class Convert>
auto convert_source_domain_data(Convert& conv, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) -> std::enable_if_t<has_source_domain_data<Convert&>::value, bool>
{
	conv.source_domain_data(domain_opening_tag, is, os);
	return true;
}
template <class Convert>
auto convert_source_domain_data(Convert&, const xml::tag&, text_istream& is, binary_ostream&) -> std::enable_if_t<!has_source_domain_data<Convert&>::value, bool>
{
	skip_xml_domain_data(is);
	return false;
}

template <class Convert>
auto convert_plain_domain_data(Convert& conv, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os) -> std::enable_if_t<has_plain_domain_data<Convert&>::value, bool>
{
	conv.plain_domain_data(domain_opening_tag, is, os);
	return true;
}
template <class Convert>
auto convert_plain_domain_data(Convert&, const xml::tag&, text_istream& is, binary_ostream&) -> std::enable_if_t<!has_plain_domain_data<Convert&>::value, bool>
{
	skip_xml_domain_data(is);
	return false;
}

template <class Convert>
auto write_constant_face_domain_data(Convert& conv, ConstantDomainDataId data_id, binary_ostream& os) -> std::enable_if_t<has_constant_face_domain_data<Convert&>::value, bool>
{
	conv.constant_face_domain_data(data_id, os);
	return true;
}
template <class Convert>
auto write_constant_face_domain_data(Convert&, ConstantDomainDataId, binary_ostream&) -> std::enable_if_t<!has_constant_face_domain_data<Convert&>::value, bool>
{
	return false;
}

template <class Convert>
auto write_constant_poly_domain_data(Convert& conv, ConstantDomainDataId data_id, binary_ostream& os) -> std::enable_if_t<has_constant_poly_domain_data<Convert&>::value, bool>
{
	conv.constant_poly_domain_data(data_id, os);
	return true;
}
template <class Convert>
auto write_constant_poly_domain_data(Convert&, ConstantDomainDataId, binary_ostream&) -> std::enable_if_t<!has_constant_poly_domain_data<Convert&>::value, bool>
{
	return false;
}

template <class Convert>
auto write_constant_source_domain_data(Convert& conv, ConstantDomainDataId data_id, binary_ostream& os) -> std::enable_if_t<has_constant_source_domain_data<Convert&>::value, bool>
{
	conv.constant_source_domain_data(data_id, os);
	return true;
}
template <class Convert>
auto write_constant_source_domain_data(Convert&, ConstantDomainDataId, binary_ostream&) -> std::enable_if_t<!has_constant_source_domain_data<Convert&>::value, bool>
{
	return false;
}

template <class Convert>
auto write_constant_plain_domain_data(Convert& conv, ConstantDomainDataId data_id, binary_ostream& os) -> std::enable_if_t<has_constant_plain_domain_data<Convert&>::value, bool>
{
	conv.constant_plain_domain_data(data_id, os);
	return true;
}
template <class Convert>
auto write_constant_plain_domain_data(Convert&, ConstantDomainDataId, binary_ostream&) -> std::enable_if_t<!has_constant_plain_domain_data<Convert&>::value, bool>
{
	return false;
}

template <class Converter>
struct ConverterImpl:IDomainConverter
{
	virtual bool model_domain_data(std::string_view strDomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		if (strDomainName == Converter::domain_name())
			return convert_model_domain_data(m_conv, domain_opening_tag, is, os);
		skip_xml_domain_data(is);
		return false;
	}
	virtual bool poly_domain_data(std::string_view strDomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		if (strDomainName == Converter::domain_name())
			return convert_poly_domain_data(m_conv, domain_opening_tag, is, os);
		skip_xml_domain_data(is);
		return false;
	}
	virtual bool face_domain_data(std::string_view strDomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		if (strDomainName == Converter::domain_name())
			return convert_face_domain_data(m_conv, domain_opening_tag, is, os);
		skip_xml_domain_data(is);
		return false;
	}
	virtual bool source_domain_data(std::string_view strDomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		if (strDomainName == Converter::domain_name())
			return convert_source_domain_data(m_conv, domain_opening_tag, is, os);
		skip_xml_domain_data(is);
		return false;
	}
	virtual bool plain_domain_data(std::string_view strDomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		if (strDomainName == Converter::domain_name())
			return convert_plain_domain_data(m_conv, domain_opening_tag, is, os);
		skip_xml_domain_data(is);
		return false;
	}
	virtual std::optional<domain_datum> constant_face_domain_data(std::string_view, ConstantDomainDataId id)
	{
		buf_ostream os;
		if (!write_constant_face_domain_data(m_conv, id, os))
			return std::optional<domain_datum>();
		return std::move(os.get_vector());
	}
	virtual domain_data_map constant_face_domain_data(ConstantDomainDataId id)
	{
		auto domain_data = this->constant_face_domain_data(Converter::domain_name(), id);
		return domain_data.has_value()?domain_data_map{{Converter::domain_name(), std::move(domain_data.value())}}:domain_data_map{};
	}
	virtual std::optional<domain_datum> constant_poly_domain_data(std::string_view, ConstantDomainDataId id)
	{
		buf_ostream os;
		if (!write_constant_poly_domain_data(m_conv, id, os))
			return std::optional<domain_datum>();
		return std::move(os.get_vector());
	}
	virtual domain_data_map constant_poly_domain_data(ConstantDomainDataId id)
	{
		auto domain_data = this->constant_poly_domain_data(Converter::domain_name(), id);
		return domain_data.has_value()?domain_data_map{{Converter::domain_name(), std::move(domain_data.value())}}:domain_data_map{};
	}
	virtual std::optional<domain_datum> constant_source_domain_data(std::string_view, ConstantDomainDataId id)
	{
		buf_ostream os;
		if (!write_constant_source_domain_data(m_conv, id, os))
			return std::optional<domain_datum>();
		return std::move(os.get_vector());
	}
	virtual domain_data_map constant_source_domain_data(ConstantDomainDataId id)
	{
		auto domain_data = this->constant_source_domain_data(Converter::domain_name(), id);
		return domain_data.has_value()?domain_data_map{{Converter::domain_name(), std::move(domain_data.value())}}:domain_data_map{};
	}
	virtual std::optional<domain_datum> constant_plain_domain_data(std::string_view, ConstantDomainDataId id)
	{
		buf_ostream os;
		if (!write_constant_plain_domain_data(m_conv, id, os))
			return std::optional<domain_datum>();
		return std::move(os.get_vector());
	}
	virtual domain_data_map constant_plain_domain_data(ConstantDomainDataId id)
	{
		auto domain_data = this->constant_plain_domain_data(Converter::domain_name(), id);
		return domain_data.has_value()?domain_data_map{{Converter::domain_name(), std::move(domain_data.value())}}:domain_data_map{};
	}

	template <class ... ConverterParams, class = std::enable_if_t<std::is_constructible_v<Converter, ConverterParams&& ... >>>
	ConverterImpl(ConverterParams&& ... conv):m_conv(std::forward<ConverterParams>(conv) ... ) {}
private:
	Converter m_conv;
};

struct generalized_converter:IDomainConverter
{
	template <class NameConverterTuple>
	generalized_converter(NameConverterTuple&& name_conv_tpl):m_mpConv(create_map(std::forward<NameConverterTuple>(name_conv_tpl)))
	{}
	virtual bool model_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
			return convert_model_domain_data(*itConv->second, domain_opening_tag, is, os);
		else
		{
			skip_xml_domain_data(is);
			return false;
		}
	}
	virtual bool poly_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
			return convert_poly_domain_data(*itConv->second, domain_opening_tag, is, os);
		else
		{
			skip_xml_domain_data(is);
			return false;
		}
	}
	virtual bool face_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
			return convert_face_domain_data(*itConv->second, domain_opening_tag, is, os);
		else
		{
			skip_xml_domain_data(is);
			return false;
		}
	}
	virtual bool source_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
			return convert_source_domain_data(*itConv->second, domain_opening_tag, is, os);
		else
		{
			skip_xml_domain_data(is);
			return false;
		}
	}
	virtual bool plain_domain_data(std::string_view DomainName, const xml::tag& domain_opening_tag, text_istream& is, binary_ostream& os)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
			return convert_plain_domain_data(*itConv->second, domain_opening_tag, is, os);
		else
		{
			skip_xml_domain_data(is);
			return false;
		}
	}
	virtual std::optional<domain_datum> constant_face_domain_data(std::string_view DomainName, ConstantDomainDataId id)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
		{
			buf_ostream os;
			if (write_constant_face_domain_data(itConv->second, id, os))
				return std::optional<domain_datum>{std::in_place_t(), std::move(os.get_vector())};
		}
		return std::optional<domain_datum>();
	}
	virtual domain_data_map constant_face_domain_data(ConstantDomainDataId id)
	{
		domain_data_map mpRet;
		for (auto& conv:m_mpConv)
		{
			buf_ostream os;
			if (write_constant_face_domain_data(conv.second, id, os))
				mpRet.emplace(conv.first, std::move(os.get_vector()));
		}
		return mpRet;
	}
	virtual std::optional<domain_datum> constant_poly_domain_data(std::string_view DomainName, ConstantDomainDataId id)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
		{
			buf_ostream os;
			if (write_constant_poly_domain_data(itConv->second, id, os))
				return std::optional<domain_datum>{std::in_place_t(), std::move(os.get_vector())};
		}
		return std::optional<domain_datum>();
	}
	virtual domain_data_map constant_poly_domain_data(ConstantDomainDataId id)
	{
		domain_data_map mpRet;
		for (auto& conv:m_mpConv)
		{
			buf_ostream os;
			if (write_constant_poly_domain_data(conv.second, id, os))
				mpRet.emplace(conv.first, std::move(os.get_vector()));
		}
		return mpRet;
	}
	virtual std::optional<domain_datum> constant_source_domain_data(std::string_view DomainName, ConstantDomainDataId id)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
		{
			buf_ostream os;
			if (write_constant_source_domain_data(itConv->second, id, os))
				return std::optional<domain_datum>{std::in_place_t(), std::move(os.get_vector())};
		}
		return std::optional<domain_datum>();
	}
	virtual domain_data_map constant_source_domain_data(ConstantDomainDataId id)
	{
		domain_data_map mpRet;
		for (auto& conv:m_mpConv)
		{
			buf_ostream os;
			if (write_constant_source_domain_data(conv.second, id, os))
				mpRet.emplace(conv.first, std::move(os.get_vector()));
		}
		return mpRet;
	}
	virtual std::optional<domain_datum> constant_plain_domain_data(std::string_view DomainName, ConstantDomainDataId id)
	{
		auto itConv = m_mpConv.find(DomainName);
		if (itConv != m_mpConv.end())
		{
			buf_ostream os;
			if (write_constant_plain_domain_data(itConv->second, id, os))
				std::optional<domain_datum>{std::in_place_t(), std::move(os.get_vector())};
		}
		return std::optional<domain_datum>();
	}
	virtual domain_data_map constant_plain_domain_data(ConstantDomainDataId id)
	{
		domain_data_map mpRet;
		for (auto& conv:m_mpConv)
		{
			buf_ostream os;
			if (write_constant_plain_domain_data(conv.second, id, os))
				mpRet.emplace(conv.first, std::move(os.get_vector()));
		}
		return mpRet;
	}
private:
	struct less_transparent
	{
		bool operator()(std::string_view lhs, std::string_view rhs) const
		{
			return lhs < rhs;
		}
		typedef void is_transparent;
	};
	typedef std::map<std::string, std::unique_ptr<IDomainConverter>, less_transparent> map_type;
	map_type m_mpConv;
	template <std::size_t I = 0, class ... Init>
	static auto create_map(const std::tuple<Init...>& tpl) -> std::enable_if_t<(I + 1 < sizeof ... (Init)), map_type>
	{
		map_type mp = create_map<I + 2>(tpl);
		mp.emplace(std::get<I>(tpl), std::unique_ptr<IDomainConverter>(new ConverterImpl<std::tuple_element_t<I + 1, std::tuple<Init...>>>(std::get<I + 1>(tpl))));
		return mp;
	}
	template <std::size_t I = 0, class ... Init>
	static auto create_map(std::tuple<Init...>&& tpl) -> std::enable_if_t<(I + 1 < sizeof ... (Init)), map_type>
	{
		map_type mp = create_map<I + 2>(std::move(tpl));
		mp.emplace(std::get<I>(tpl), std::unique_ptr<IDomainConverter>(new ConverterImpl<std::tuple_element_t<I + 1, std::tuple<Init...>>>(std::get<I + 1>(std::move(tpl)))));
		return mp;
	}
	template <std::size_t I = 0>
	static auto create_map(...)
	{
		return map_type();
	}
};

#endif // XML2BIN_DOMAIN_CONVERTER_H_
