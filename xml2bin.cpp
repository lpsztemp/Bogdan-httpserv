#include <algorithm>
#include <iterator>
#include <functional>
#include <list>
#include <fstream>
#include <optional>
#include <codecvt>
#include <locale>
#include "basedefs.h"
#include "xml2bin.h"
#include "radio_hf_domain_xml2bin.h"
#include "arch_ac_domain_xml2bin.h"
#include "domain_converter.h"
#include "hgt_optimizer.h"
#include "xml_parser.h"
#include "binary_streams.h"
#include "text_streams.h"
#include "point.h"
#if FILESYSTEM_CPP17
#include <filesystem>
#endif

struct input_not_ready:std::invalid_argument
{
	input_not_ready():std::invalid_argument("Input model is not fully specified") {}
};

static constexpr double unspecified_double()
{
	return std::numeric_limits<double>::infinity();
}

static constexpr bool is_specified(double val)
{
	return val != unspecified_double();
}

static constexpr point_t unspecified_point()
{
	return {unspecified_double(), unspecified_double(), unspecified_double()};
}

static constexpr bool is_specified(const point_t& val)
{
	return val.x != unspecified_double() && val.y != unspecified_double() && val.z != unspecified_double();
}

template <class T, class = void>
struct has_data_and_size:std::false_type {};
template <class T>
struct has_data_and_size<T, std::void_t<decltype(std::declval<T>().data()), decltype(std::declval<T>().size())>>:std::true_type {};

template <class T, class = void>
struct is_iteratable:std::false_type {};
template <class T>
struct is_iteratable<T, std::void_t<decltype(std::begin(std::declval<T>())), decltype(std::end(std::declval<T>()))>>:std::true_type {};

class conversion_state_impl:public Implementation::conversion_state
{
	binary_ostream* m_pOs = nullptr;
	std::istream* m_pHgt = nullptr;
	point_t m_size = unspecified_point();
	std::string m_strModelName;

	typedef std::map<std::string, std::vector<std::uint8_t>> domain_data_map;
	domain_data_map m_mapDomainData;

	struct poly_data
	{
		struct face_data
		{
			std::list<point_t> lstVertices;
			domain_data_map mapDomainData;
		};
		std::string name;
		std::list<face_data> lstFaces;
		domain_data_map mapDomainData;
	};
	std::map<std::string, poly_data> m_polyNamedMap;
	std::list<poly_data> m_polyUnnamedList;

	struct source_data
	{
		point_t pos = unspecified_point();
		point_t dir = unspecified_point();
		point_t top = unspecified_point();
		domain_data_map mapDomainData;
		std::string name;
	};
	std::map<std::string, source_data> m_srcNamedMap;
	std::list<source_data> m_srcUnnamedList;

	struct plain_data
	{
		point_t pos = unspecified_point();
		point_t v1 = unspecified_point();
		point_t v2 = unspecified_point();
		double ePlainGridStep = unspecified_double();
		domain_data_map mapDomainData;
		std::string name;
	};
	std::map<std::string, plain_data> m_plainNamedMap;
	std::list<plain_data> m_plainUnnamedList;
	std::unique_ptr<IDomainConverter> m_pConv;
public:
	conversion_state_impl(std::string_view domain, binary_ostream& os):m_pOs(std::addressof(os))
	{
		if (domain == radio_hf_convert::domain_name())
			m_pConv.reset(new ConverterImpl<radio_hf_convert>(radio_hf_convert()));
		else if (domain == arch_ac_convert::domain_name())
			m_pConv.reset(new ConverterImpl<arch_ac_convert>(arch_ac_convert()));
		else
			throw std::invalid_argument("Unknown domain name");
	}
	explicit conversion_state_impl(binary_ostream& os):m_pOs(std::addressof(os))
	{
		m_pConv.reset(new generalized_converter(std::make_tuple(
			radio_hf_convert::domain_name(),	radio_hf_convert(),
			arch_ac_convert::domain_name(),		arch_ac_convert()
		)));
	}
	conversion_state_impl(const conversion_state_impl&) = delete;
	conversion_state_impl& operator=(const conversion_state_impl&) = delete;

	void next_xml(text_istream& is)
	{
		using namespace std;
		xml::tag tag;
		const std::pair<TextEncoding, std::wstring> pTestEncoding[] =
		{
			{TextEncoding::UTF8, std::wstring(L"UTF-8")},
			{TextEncoding::UTF16LE, std::wstring(L"UTF-16LE")},
			{TextEncoding::UTF16BE, std::wstring(L"UTF-16BE")},
			{TextEncoding::Windows_1251, std::wstring(L"windows-1251")}
		};
		auto start_pos = is.tellg();
		int cur_enc;
		tag = xml::tag(is, xml::tag::nothrow);
		if (is.fail())
		{
			cur_enc = 0;
			do
			{
				is.clear();
				is.seekg(start_pos);
				if (is.fail())
					throw xml_invalid_syntax();
				if (!is.set_encoding(pTestEncoding[cur_enc++].first).fail())
				{
					tag = xml::tag(is, xml::tag::nothrow);
					if (!is.fail())
						break;
				}
			}while (cur_enc < int(sizeof(pTestEncoding) / sizeof(TextEncoding)));
		}
		while (tag.is_comment())
			tag = xml::tag(is);
		if (!tag.is_header())
			throw invalid_xml_model(L"XML header is not specified");
		for (cur_enc = 0; cur_enc < int(sizeof(pTestEncoding) / sizeof(TextEncoding)); ++cur_enc)
		{
			const auto& enc1 = tag.attribute(L"encoding");
			const auto& enc2 = pTestEncoding[cur_enc].second;
			if (std::equal(std::begin(enc1), std::end(enc1), std::begin(enc2), std::end(enc2),
				[](wchar_t chl, wchar_t chr) -> bool {return std::towupper(chl) == std::towupper(chr);}))
			{
				is.set_encoding(pTestEncoding[cur_enc].first);
				break;
			}
		}
		if (cur_enc == int(sizeof(pTestEncoding) / sizeof(TextEncoding)))
			throw invalid_xml_model(is.get_resource_locator(), L"Unknown or unspecified XML encoding");
		while ((tag = xml::tag(is)).is_comment()) continue;
		if (tag.name() != L"model")
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
		this->convert_model(tag, is);
	}
	void next_hgt(std::istream& is)
	{
		m_pHgt = std::addressof(is); //do it in finalize directly to pOs
	}
	void finalize()
	{
		std::unique_ptr<hgt_t> hgt;
		if (m_pHgt)
		{
			hgt = hgt_t::read(*m_pHgt, *m_pConv);
			const auto& hgt_attr = hgt->attributes();
			auto off = hgt_attr.height_offset;
			if (!is_specified(m_size))
			{
				m_size.x = hgt_attr.resolution.cColumns * hgt_attr.resolution.dx;
				m_size.y = hgt_attr.resolution.cRows * hgt_attr.resolution.dy;
				m_size.z = hgt_attr.max_height;
			}else if (off != 0)
				m_size.z += off;
			if (off != 0)
			{
				for (auto& poly:m_polyNamedMap)
					for (auto& face:poly.second.lstFaces)
						for (auto& vertex:face.lstVertices)
							vertex.z += hgt->attributes().height_offset;
				for (auto& poly:m_polyUnnamedList)
					for (auto& face:poly.lstFaces)
						for (auto& vertex:face.lstVertices)
							vertex.z += hgt->attributes().height_offset;
				for (auto& src:m_srcNamedMap)
				{
					src.second.pos.z += off;
					src.second.dir.z += off;
					src.second.top.z += off;
				}
				for (auto& src:m_srcUnnamedList)
				{
					src.pos.z += off;
					src.dir.z += off;
					src.top.z += off;
				}
				for (auto& plain:m_plainNamedMap)
				{
					plain.second.pos.z += off;
					plain.second.v1.z += off;
					plain.second.v2.z += off;
				}
				for (auto& plain:m_plainUnnamedList)
				{
					plain.pos.z += off;
					plain.v1.z += off;
					plain.v2.z += off;
				}
			}
		}
		if (!this->is_model_ready())
			throw input_not_ready();
		auto& os = *m_pOs;
		this->write_sequence(m_strModelName);
		this->write(CAMaaS::CHU_METERS);
		this->write(m_size);
		this->write(m_mapDomainData);
		auto object_count = std::uint32_t(this->poly_count() + this->source_count() + this->plain_count());
		auto object_count_pos = os.tellp();
		os << object_count;
		for (const auto& poly:m_polyNamedMap)
			this->write(poly.second);
		for (const auto& poly:m_polyUnnamedList)
			this->write(poly);
		for (const auto& src:m_srcNamedMap)
			this->write(src.second);
		for (const auto& src:m_srcUnnamedList)
			this->write(src);
		for (const auto& plain:m_plainNamedMap)
			this->write(plain.second);
		for (const auto& plain:m_plainUnnamedList)
			this->write(plain);
		if (bool(hgt))
		{
			auto hgt_stats = hgt->write(os);
			auto old_pos = os.tellp();
			os.seekp(object_count_pos);
			os << object_count + std::uint32_t(hgt_stats.poly_count);
			os.seekp(old_pos);
		}
	}
private:
	poly_data::face_data convert_face(const xml::tag& rTag, text_istream& is)
	{
		poly_data::face_data face;
		while (true)
		{
			xml::tag tag;
			while ((tag = xml::tag(is)).is_comment()) continue;
			if (tag.is_comment())
				continue;
			else if (tag.name() == L"domain")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto strDomain = encode_string(tag.attribute(L"name"));
				if (strDomain.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"name");
				buf_ostream os_buf;
				if (m_pConv->face_domain_data(strDomain, tag, is, os_buf)
					&& !face.mapDomainData.emplace(std::move(strDomain), std::move(os_buf.get_vector())).second)
					throw ambiguous_specification(is.get_resource_locator(), L"domain");
			}else if (tag.name() == L"vertex")
			{
				if (!tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				point_t v;
				const auto& x = tag.attribute(L"x");
				if (x.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"x");
				v.x = std::stod(x);
				const auto& y = tag.attribute(L"y");
				if (y.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"y");
				v.y = std::stod(y);
				const auto& z = tag.attribute(L"z");
				if (z.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"z");
				v.z = std::stod(z);
				face.lstVertices.emplace_back(std::move(v));
			}else if (tag.name() == L"face" && tag.is_closing_tag())
				break;
			else
				throw improper_xml_tag(is.get_resource_locator(), tag.name());
		};
		if (face.lstVertices.size() < 3)
			throw invalid_xml_model(is.get_resource_locator(), L"A face must have at least 3 vertices that do not appertain one line.");
		return face;
	}
	poly_data convert_poly(const xml::tag& rTag, text_istream& is)
	{
		poly_data poly;
		poly.name = encode_string(rTag.attribute(L"name"));
		while (true)
		{
			auto tag = xml::tag(is);
			if (tag.is_comment())
				continue;
			else if (tag.name() == L"domain")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto strDomain = encode_string(tag.attribute(L"name"));
				if (strDomain.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"name");
				buf_ostream os_buf;
				if (m_pConv->poly_domain_data(strDomain, tag, is, os_buf)
					&& !poly.mapDomainData.emplace(std::move(strDomain), std::move(os_buf.get_vector())).second)
					throw ambiguous_specification(is.get_resource_locator(), L"domain");
			}else if (tag.name() == L"face")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				poly.lstFaces.emplace_back(convert_face(tag, is));
			}else if (tag.name() == L"polyobject" && tag.is_closing_tag())
				break;
			else
				throw improper_xml_tag(is.get_resource_locator(), tag.name());
		};
		return poly;
	}
	source_data convert_source(const xml::tag& rTag, text_istream& is)
	{
		source_data source;
		source.name = encode_string(rTag.attribute(L"name"));
		while (true)
		{
			auto tag = xml::tag(is);
			if (tag.is_comment())
				continue;
			else if (tag.name() == L"domain")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto strDomain = encode_string(tag.attribute(L"name"));
				if (strDomain.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"name");
				buf_ostream os_buf;
				if (m_pConv->source_domain_data(strDomain, tag, is, os_buf)
					&& !source.mapDomainData.emplace(std::move(strDomain), std::move(os_buf.get_vector())).second)
					throw ambiguous_specification(is.get_resource_locator(), L"domain");
			}else if (tag.name() == L"position")
			{
				if (!tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto coord = tag.attribute(L"x");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"x");
				source.pos.x = std::stod(coord);
				coord = tag.attribute(L"y");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"y");
				source.pos.y = std::stod(coord);
				coord = tag.attribute(L"z");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"z");
				source.pos.z = std::stod(coord);
			}else if (tag.name() == L"direction")
			{
				if (!tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto coord = tag.attribute(L"x");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"x");
				source.dir.x = std::stod(coord);
				coord = tag.attribute(L"y");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"y");
				source.dir.y = std::stod(coord);
				coord = tag.attribute(L"z");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"z");
				source.dir.z = std::stod(coord);
			}else if (tag.name() == L"top")
			{
				if (!tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto coord = tag.attribute(L"x");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"x");
				source.top.x = std::stod(coord);
				coord = tag.attribute(L"y");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"y");
				source.top.y = std::stod(coord);
				coord = tag.attribute(L"z");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"z");
				source.top.z = std::stod(coord);
			}else if (tag.name() == L"sourceobject" && tag.is_closing_tag())
				break;
			else
				throw improper_xml_tag(is.get_resource_locator(), tag.name());
		};
		if (source.pos == unspecified_point())
			throw invalid_xml_model(is.get_resource_locator(), L"A source position is not specified.");
		if (source.dir == unspecified_point())
			throw invalid_xml_model(is.get_resource_locator(), L"A source main axis, aka direction, is not specified.");
		if (source.top == unspecified_point())
			throw invalid_xml_model(is.get_resource_locator(), L"A source top direction is not specified.");
		return source;
	}
	plain_data convert_plain(const xml::tag& rTag, text_istream& is)
	{
		plain_data plain;
		plain.name = encode_string(rTag.attribute(L"name"));
		while (true)
		{
			auto tag = xml::tag(is);
			if (tag.is_comment())
				continue;
			else if (tag.name() == L"domain")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto strDomain = encode_string(tag.attribute(L"name"));
				if (strDomain.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"name");
				buf_ostream os_buf;
				if (m_pConv->plain_domain_data(strDomain, tag, is, os_buf)
					&& !plain.mapDomainData.emplace(std::move(strDomain), std::move(os_buf.get_vector())).second)
					throw ambiguous_specification(is.get_resource_locator(), L"domain");
			}else if (tag.name() == L"position")
			{
				if (!tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto coord = tag.attribute(L"x");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"x");
				plain.pos.x = std::stod(coord);
				coord = tag.attribute(L"y");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"y");
				plain.pos.y = std::stod(coord);
				coord = tag.attribute(L"z");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"z");
				plain.pos.z = std::stod(coord);
			}else if (tag.name() == L"v1")
			{
				if (!tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto coord = tag.attribute(L"x");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"x");
				plain.v1.x = std::stod(coord);
				coord = tag.attribute(L"y");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"y");
				plain.v1.y = std::stod(coord);
				coord = tag.attribute(L"z");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"z");
				plain.v1.z = std::stod(coord);
			}else if (tag.name() == L"v2")
			{
				if (!tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto coord = tag.attribute(L"x");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"x");
				plain.v2.x = std::stod(coord);
				coord = tag.attribute(L"y");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"y");
				plain.v2.y = std::stod(coord);
				coord = tag.attribute(L"z");
				if (coord.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"z");
				plain.v2.z = std::stod(coord);
			}else if (tag.name() == L"grid")
			{
				if (!tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto val = tag.attribute(L"step");
				if (val.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"x");
				plain.ePlainGridStep = std::stod(val);
			}else if (tag.name() == L"plainobject" && tag.is_closing_tag())
				break;
			else
				throw improper_xml_tag(is.get_resource_locator(), tag.name());
		};
		if (plain.pos == unspecified_point())
			throw invalid_xml_model(is.get_resource_locator(), L"A point on a control point plain is not specified.");
		if (plain.v1 == unspecified_point())
			throw invalid_xml_model(is.get_resource_locator(), L"A vector \"v1\" on a control point plain is not specified.");
		if (plain.v1 == unspecified_point())
			throw invalid_xml_model(is.get_resource_locator(), L"A vector \"v2\" on a control point plain is not specified.");
		if (plain.v1 == unspecified_point())
			throw invalid_xml_model(is.get_resource_locator(), L"A control point plain grid step is not specified.");
		return plain;
	}
	void convert_model(const xml::tag& rModelTag, text_istream& is)
	{
		auto strAttr = rModelTag.attribute(L"cx");
		if (!strAttr.empty())
		{
			if (is_specified(m_size.x))
				throw ambiguous_specification(is.get_resource_locator(), L"cx");
			m_size.x = std::stod(strAttr);
		}
		strAttr = rModelTag.attribute(L"cy");
		if (!strAttr.empty())
		{
			if (is_specified(m_size.y))
				throw ambiguous_specification(is.get_resource_locator(), L"cy");
			m_size.y = std::stod(strAttr);
		}
		strAttr = rModelTag.attribute(L"cz");
		if (!strAttr.empty())
		{
			if (is_specified(m_size.z))
				throw ambiguous_specification(is.get_resource_locator(), L"cz");
			m_size.z = std::stod(strAttr);
		}
		strAttr = rModelTag.attribute(L"name");
		if (!strAttr.empty())
		{
			if (!m_strModelName.empty())
				throw ambiguous_specification(is.get_resource_locator(), L"name");
			m_strModelName = encode_string(strAttr);
		}
		while (true)
		{
			auto tag = xml::tag(is);
			if (tag.is_comment())
				continue;
			else if (tag.name() == L"domain")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto strDomain = encode_string(tag.attribute(L"name"));
				if (strDomain.empty())
					throw xml_attribute_not_found(is.get_resource_locator(), L"name");
				buf_ostream os_buf;
				if (m_pConv->model_domain_data(strDomain, tag, is, os_buf)
					&& !m_mapDomainData.emplace(std::move(strDomain), std::move(os_buf.get_vector())).second)
					throw ambiguous_specification(is.get_resource_locator(), L"domain");
			}else if (tag.name() == L"polyobject")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto poly = convert_poly(tag, is);
				if (poly.name.empty())
					m_polyUnnamedList.emplace_back(std::move(poly));
				else
				{
					auto name = poly.name;
					if (!m_polyNamedMap.emplace(std::move(name), std::move(poly)).second)
						throw ambiguous_specification(is.get_resource_locator(), tag.name());
				}
			}else if (tag.name() == L"sourceobject")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto source = convert_source(tag, is);
				if (source.name.empty())
					m_srcUnnamedList.emplace_back(std::move(source));
				else
				{
					auto name = source.name;
					if (!m_srcNamedMap.emplace(std::move(name), std::move(source)).second)
						throw ambiguous_specification(is.get_resource_locator(), tag.name());
				}
			}else if (tag.name() == L"plainobject")
			{
				if (tag.is_closing_tag() || tag.is_unary_tag())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
				auto plain = convert_plain(tag, is);
				if (plain.name.empty())
					m_plainUnnamedList.emplace_back(std::move(plain));
				else
				{
					auto name = plain.name;
					if (!m_plainNamedMap.emplace(std::move(name), std::move(plain)).second)
						throw ambiguous_specification(is.get_resource_locator(), tag.name());
				}
			}else if (tag.name() == L"model" && tag.is_closing_tag())
				break;
			else
				throw improper_xml_tag(is.get_resource_locator(), tag.name());
		};
	}
	template <class T>
	auto write(binary_ostream& os, const T& val) const -> std::enable_if_t<std::is_pod_v<std::decay_t<T>>>
	{
		os << val;
	}
	void write(binary_ostream& os, const point_t& pt) const
	{
		os << std::uint32_t(3) // dimensions
			<< pt.x << pt.y << pt.z;
	}
	template <class Cont>
	auto write_sequence(binary_ostream& os, const Cont& cont) const -> std::enable_if_t<has_data_and_size<Cont>::value>
	{
		std::size_t cb = cont.size() * sizeof(typename std::decay<Cont>::type::value_type);
		os << std::uint32_t(cb);
		os.write(reinterpret_cast<const char*>(cont.data()), cb);
	}
	template <class Cont>
	auto write_sequence(binary_ostream& os, const Cont& cont) const -> std::enable_if_t<!has_data_and_size<Cont>::value && is_iteratable<Cont>::value>
	{
		using std::begin;
		using std::end;
		os << std::uint32_t(cont.size());
		for (const auto& elem:cont)
			this->write(os, elem);
	}
	void write(binary_ostream& os, const domain_data_map& mpDomainData) const
	{
		os << std::uint32_t(mpDomainData.size());
		for (const auto& prDomainData:mpDomainData)
		{
			this->write_sequence(os, prDomainData.first);
			this->write_sequence(os, prDomainData.second);
		}
	}
	void write_object_generic_part(binary_ostream& os, std::string_view strObjectName, const domain_data_map& mpDomainData, CAMaaS::ObjectTypeId type_id) const
	{
		this->write_sequence(os, strObjectName);
		this->write(os, mpDomainData);
		os << type_id;
	}
	void write_object_generic_part(binary_ostream& os, const domain_data_map& mpDomainData, CAMaaS::ObjectTypeId type_id) const
	{
		write_object_generic_part(os, std::string_view(), mpDomainData, type_id);
	}
	void write(binary_ostream& os, const poly_data::face_data& face) const
	{
		this->write_sequence(os, face.lstVertices);
		this->write(os, face.mapDomainData);
	}
	void write(binary_ostream& os, const poly_data& poly) const
	{
		this->write_object_generic_part(os, poly.name, poly.mapDomainData, CAMaaS::ObjectPoly);
		this->write_sequence(os, poly.lstFaces);
	}
	void write(binary_ostream& os, const source_data& src) const
	{
		this->write_object_generic_part(os, src.name, src.mapDomainData, CAMaaS::ObjectSource);
		this->write(os, src.pos);
		this->write(os, std::uint32_t(2));
		this->write(os, src.dir);
		this->write(os, src.top);
	}
	void write(binary_ostream& os, const plain_data& plain) const
	{
		this->write_object_generic_part(os, plain.name, plain.mapDomainData, CAMaaS::ObjectPlain);
		this->write(os, plain.pos);
		this->write(os, std::uint32_t(2));
		this->write(os, plain.v1);
		this->write(os, plain.v2);
		this->write(os, plain.ePlainGridStep);
	}
	template <class T>
	void write(const T& elem)
	{
		this->write(*m_pOs, elem);
	}
	template <class T>
	void write_sequence(const T& container)
	{
		this->write_sequence(*m_pOs, container);
	}
	bool is_model_ready() const
	{
		return is_specified(m_size);
	}
	std::size_t poly_count() const
	{
		return m_polyNamedMap.size() + m_polyUnnamedList.size();
	}
	std::size_t source_count() const
	{
		return m_srcNamedMap.size() + m_srcUnnamedList.size();
	};
	std::size_t plain_count() const
	{
		return m_plainNamedMap.size() + m_plainUnnamedList.size();
	};
};

namespace Implementation
{
	std::unique_ptr<conversion_state> xml2bin_set(const std::string& domain, binary_ostream& os)
	{
		return std::unique_ptr<conversion_state>(new conversion_state_impl(domain, os));
	}
	void xml2bin_next_xml(const std::unique_ptr<conversion_state>& state, text_istream& is)
	{
		static_cast<conversion_state_impl*>(state.get())->next_xml(is);
	}
	void xml2bin_next_hgt(const std::unique_ptr<conversion_state>& state, std::istream& is)
	{
		static_cast<conversion_state_impl*>(state.get())->next_hgt(is);
	}
	void xml2bin_finalize(const std::unique_ptr<conversion_state>& state)
	{
		static_cast<conversion_state_impl*>(state.get())->finalize();
	}
} //Implementation
