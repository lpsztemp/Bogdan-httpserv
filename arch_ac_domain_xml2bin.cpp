#include "arch_ac_domain_xml2bin.h"
#include <map>
#include <tuple>
#include <memory>

const std::string& arch_ac_convert::domain_name()
{
	static const std::string name = "arch_ac";
	return name;
}

struct ModelDomainData
{
	double eAttenuation;
};

struct FaceDomainData
{
	constexpr static std::uint32_t domain_data_id_2 = 0x80000001u;
	std::map<double, double> absorption_map;
};

struct SourceDomainData
{
	constexpr static std::uint32_t version_2_domain_id = 0x80000001u;
	struct RadiationPatternData
	{
		virtual ~RadiationPatternData() {}
		virtual binary_ostream& write_to_stream(binary_ostream&) const = 0;
	};
	struct FrequencyResponseData
	{
		virtual ~FrequencyResponseData() {}
		virtual binary_ostream& write_to_stream(binary_ostream&) const = 0;
	};
	class ExpressionRadiationData: public RadiationPatternData
	{
		static constexpr std::uint32_t datum_id = 1u;
		std::string expression_value;
	public:
		ExpressionRadiationData() = default;
		ExpressionRadiationData(const std::wstring& expr):expression_value(encode_string(expr)) {}
		virtual binary_ostream& write_to_stream(binary_ostream& os) const
		{
			os << datum_id << std::uint32_t(expression_value.size());
			return os.write(expression_value.data(), expression_value.size());
		}
	};
	class TableRadiationData:public RadiationPatternData
	{
		static constexpr std::uint32_t datum_id = 2u;
		struct MapCmp
		{
			bool operator()(const std::tuple<double, double, double>& left, const std::tuple<double, double, double>& right) const
			{
				using std::get;
				return get<0>(left) < get<0>(right) || (get<0>(left) == get<0>(right) && (get<1>(left) < get<1>(right) || (get<1>(left) == get<1>(right) && get<2>(left) < get<2>(right))));
			}
		};
		typedef std::map<std::tuple<double, double, double>, double, MapCmp> map_type;
		map_type m_table;
	public:
		TableRadiationData() = default;
		bool emplace(double eF, double eAz, double eZn, double eVal)
		{
			return m_table.emplace(std::make_tuple(eF, eAz, eZn), eVal).second;
		}
		virtual binary_ostream& write_to_stream(binary_ostream& os) const
		{
			os << datum_id << std::uint32_t(m_table.size());
			for (const auto& pr:m_table)
				os << std::get<0>(pr.first) << std::get<1>(pr.first) << std::get<2>(pr.first) << pr.second;
			return os;
		}
	};
	class ExpressionFrequencyResponseData: public FrequencyResponseData
	{
		static constexpr std::uint32_t datum_id = 0x101u;
		std::string expression_value;
	public:
		ExpressionFrequencyResponseData() = default;
		ExpressionFrequencyResponseData(const std::wstring& expr):expression_value(encode_string(expr)) {}
		virtual binary_ostream& write_to_stream(binary_ostream& os) const
		{
			os << datum_id << std::uint32_t(expression_value.size());
			return os.write(expression_value.data(), expression_value.size());
		}
	};
	class TableFrequencyResponseData:public FrequencyResponseData
	{
		static constexpr std::uint32_t datum_id = 0x102u;
		typedef std::map<double, double> map_type;
		map_type m_table;
	public:
		TableFrequencyResponseData() = default;
		bool emplace(double eF, double eVal)
		{
			return m_table.emplace(eF, eVal).second;
		}
		virtual binary_ostream& write_to_stream(binary_ostream& os) const
		{
			os << datum_id << std::uint32_t(m_table.size());
			for (const auto& pr:m_table)
				os << pr.first << pr.second;
			return os;
		}
	};
private:
	std::unique_ptr<RadiationPatternData> m_pRp;
	std::unique_ptr<FrequencyResponseData> m_pFr;
public:
	SourceDomainData() = default;
	const FrequencyResponseData& GetFrequencyResponse() const
	{
		return *m_pFr;
	}
	template <class FrequencyResponseT>
	SourceDomainData& SetFrequencyResponse(FrequencyResponseT&& fr)
	{
		m_pFr.reset(new std::decay_t<FrequencyResponseT>(std::forward<FrequencyResponseT>(fr)));
		return *this;
	}
	const RadiationPatternData& GetRadiationPattern() const
	{
		return *m_pRp;
	}
	template <class RadiationPatternT>
	SourceDomainData& SetRadiationPattern(RadiationPatternT&& rp)
	{
		m_pRp.reset(new std::decay_t<RadiationPatternT>(std::forward<RadiationPatternT>(rp)));
		return *this;
	}
};

static FaceDomainData LoadFaceDomainData(text_istream& is, const xml::tag& opening_tag)
{
	bool fAbsorptionSpecified = false;
	FaceDomainData result;
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"absorption")
		{
			if (fAbsorptionSpecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			while (true)
			{
				tag = xml::tag(is);
				if (tag.is_comment())
					continue;
				else if (tag.name() == L"absorption_row")
				{
					auto freq = tag.attribute(L"frequency");
					if (freq.empty())
						throw xml_attribute_not_found(is.get_resource_locator(), L"frequency");
					if (!result.absorption_map.emplace(std::stod(freq), xml::get_tag_value<double>(is, tag)).second)
						throw ambiguous_specification(is.get_resource_locator(), L"absorption_row");
				}else if (tag.name() == L"absorption" && tag.is_closing_tag() && !tag.is_unary_tag())
					break;
				else
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
			}
			if (result.absorption_map.empty())
				throw xml_tag_not_found(is.get_resource_locator(), L"absorption_row");
			else if (result.absorption_map.size() != 6)
				throw invalid_xml_model(is.get_resource_locator(), L"Invalid arch_ac absorption specification");
			else
			{
				double frequency_set[] = {125, 250, 500, 1000, 2000, 4000};
				double* f = frequency_set;
				for (auto it = std::begin(result.absorption_map); it != std::end(result.absorption_map); ++it)
					if (it->first != *f++)
						throw invalid_xml_model(is.get_resource_locator(), L"Invalid arch_ac absorption specification");
			}
			fAbsorptionSpecified = true;
		}else if (tag.name() == opening_tag.name() && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	if (!fAbsorptionSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"absorption");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const FaceDomainData& data)
{
	os << FaceDomainData::domain_data_id_2 << std::uint32_t(data.absorption_map.size());
	for (const auto& pr:data.absorption_map)
		os << pr.second;
	return os;
}

static SourceDomainData LoadSourceDomainData(text_istream& is, const xml::tag& opening_tag)
{
	SourceDomainData result;
	bool fFrequencyResponseSpecified = false, fRadiationPatternSpecified = false;
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"afc")
		{
			if (fFrequencyResponseSpecified)
				throw ambiguous_specification(is.get_resource_locator(), L"afc");
			while (true)
			{
				tag = xml::tag(is);
				if (tag.is_comment())
					continue;
				else if (tag.name() == L"function")
				{
					if (fFrequencyResponseSpecified)
						throw ambiguous_specification(is.get_resource_locator(), L"afc");
					result.SetFrequencyResponse(SourceDomainData::ExpressionFrequencyResponseData(xml::get_tag_value<std::wstring>(is, tag)));
					fFrequencyResponseSpecified = true;
				}else if (tag.name() == L"afc_row")
				{
					if (fFrequencyResponseSpecified)
						throw ambiguous_specification(is.get_resource_locator(), L"afc");
					SourceDomainData::TableFrequencyResponseData fr;
					while (true)
					{
						auto freq = tag.attribute(L"frequency");
						if (freq.empty())
							throw xml_attribute_not_found(is.get_resource_locator(), L"frequency");
						auto value = xml::get_tag_value<double>(is, tag);
						if (!fr.emplace(std::stod(freq), value))
							throw ambiguous_specification(is.get_resource_locator(), L"afc_row");
						tag = xml::tag(is);
						if (tag.is_comment())
							continue;
						else if (tag.name() == L"afc_row")
							continue;
						else if (tag.name() == L"afc" && tag.is_closing_tag() && !tag.is_unary_tag())
							break;
						else
							throw improper_xml_tag(is.get_resource_locator(), tag.name());
					}
					result.SetFrequencyResponse(std::move(fr));
					fFrequencyResponseSpecified = true;
					break;
				}else if (tag.name() == L"afc" && tag.is_closing_tag() && !tag.is_unary_tag())
					break;
				else
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
			};
			if (!fFrequencyResponseSpecified)
				throw xml_tag_not_found(is.get_resource_locator(), L"afc_row or function");
		}else if (tag.name() == L"rp")
		{
			if (fRadiationPatternSpecified)
				throw ambiguous_specification(is.get_resource_locator(), L"rp");
			while (true)
			{
				tag = xml::tag(is);
				if (tag.is_comment())
					continue;
				else if (tag.name() == L"function")
				{
					if (fRadiationPatternSpecified)
						throw ambiguous_specification(is.get_resource_locator(), L"rp");
					result.SetRadiationPattern(SourceDomainData::ExpressionRadiationData(xml::get_tag_value<std::wstring>(is, tag)));
					fRadiationPatternSpecified = true;
				}else if (tag.name() == L"rp_row")
				{
					if (fRadiationPatternSpecified)
						throw ambiguous_specification(is.get_resource_locator(), L"radiation_pattern");
					SourceDomainData::TableRadiationData rp;
					while (true)
					{
						auto freq = tag.attribute(L"frequency");
						if (freq.empty())
							throw xml_attribute_not_found(is.get_resource_locator(), L"frequency");
						auto az = tag.attribute(L"azimuth");
						if (az.empty())
							throw xml_attribute_not_found(is.get_resource_locator(), L"azimuth");
						auto zn = tag.attribute(L"zenith");
						if (zn.empty())
							throw xml_attribute_not_found(is.get_resource_locator(), L"zenith");
						auto value = xml::get_tag_value<double>(is, tag);
						if (!rp.emplace(std::stod(freq), std::stod(az), std::stod(zn), value))
							throw ambiguous_specification(is.get_resource_locator(), L"rp_row");
						tag = xml::tag(is);
						if (tag.is_comment())
							continue;
						else if (tag.name() == L"rp_row")
							continue;
						else if (tag.name() == L"rp" && tag.is_closing_tag() && !tag.is_unary_tag())
							break;
						else
							throw improper_xml_tag(is.get_resource_locator(), tag.name());
					}
					result.SetRadiationPattern(std::move(rp));
					fRadiationPatternSpecified = true;
					break;
				}else if (tag.name() == L"rp" && tag.is_closing_tag() && !tag.is_unary_tag())
					break;
				else
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
			};
			if (!fRadiationPatternSpecified)
				throw xml_tag_not_found(is.get_resource_locator(), L"rp_row or function");
		}else if (tag.name() == opening_tag.name() && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	if (!fFrequencyResponseSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"afc");
	if (!fRadiationPatternSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"rp");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const SourceDomainData& data)
{
	os << SourceDomainData::version_2_domain_id;
	return data.GetFrequencyResponse().write_to_stream(data.GetRadiationPattern().write_to_stream(os));
}

static ModelDomainData LoadModelDomainData(text_istream& is, const xml::tag& opening_tag)
{
	bool fModelSpecified = false;
	ModelDomainData result;
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"attenuation")
		{
			if (fModelSpecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			result.eAttenuation = xml::get_tag_value<double>(is, tag);
			fModelSpecified = true;
		}else if (tag.name() == opening_tag.name() && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	if (!fModelSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"attenuation");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const ModelDomainData& data)
{
	return os << data.eAttenuation;
}

void arch_ac_convert::face_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os)
{
	os << LoadFaceDomainData(is, opening_tag);
}

void arch_ac_convert::source_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os)
{
	os << LoadSourceDomainData(is, opening_tag);
}

void arch_ac_convert::model_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os)
{
	os << LoadModelDomainData(is, opening_tag); //GCC: result.eAttenuation -Wmaybe-uninitialized  false positive
}
