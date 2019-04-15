#include "radio_hf_domain_xml2bin.h"
#include <list>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <tuple>
#include <cassert>
//#include <chsvlib/chsvmath.h>

const std::string& radio_hf_convert::domain_name()
{
	static const std::string name = "radio_hf";
	return name;
}

enum class SourceDatumTypeId:std::uint32_t
{
	ExpressionRadiationPattern = 1,
	TableRadiationPattern,

	ExpressionFrequencyResponse = 0x101,
	TableFrequencyResponse,

	AntennaGain = 0x201,
	Polarization = 0x301,
	InputPower = 0x401,
	FarField = 0x501
};

enum class MediumDatumTypeId:std::uint32_t
{
	Permittivity = 1,
	Permeability,
	Conductivity,
	ElectricLoss,
	MagneticLoss
};

enum class ModelMediumDatumTypeId:std::uint32_t
{
	Medium = 1,
	RefractionChange
};

enum class InputModelDatumTypeId:std::uint32_t
{
	ModelMedium = 1,
	MinimalFieldAmplitude = 0x10,
	FrequencyRange = 0x20,
	FrequencySet,
	IterationAverageResultingChange = 0x30,
	IterationAverageResultingStep,
	MaxIteration = 0x40
};

struct MediumDefinition
{
	constexpr static double default_permittivity = 8.854E-12;
	constexpr static double default_permeability = 4 * Chusov::Math::pi<double>() * 1E-7;
	constexpr static double default_coductivity = 0.;
	constexpr static double default_electric_loss = 0.;
	constexpr static double default_magnetic_loss = 0.;
	double ePermittivity;
	double ePermeability;
	double eConductivity;
	double eElectricLoss;
	double eMagneticLoss;
};

struct ModelMediumDefinition
{
	double eRefractionChange;
	MediumDefinition medium;
};

struct ModelDomainData
{
	struct FrequencySetGeneric
	{
		virtual ~FrequencySetGeneric() {}
	};
	struct FrequencySet:FrequencySetGeneric
	{
		std::list<double> m_container;
		FrequencySet() = default;
		explicit FrequencySet(std::list<double>&& cont):m_container(std::move(cont)) {}
	};
	struct FrequencyRange:FrequencySetGeneric
	{
		double eMin, eMax, eStep;
		FrequencyRange() = default;
		inline FrequencyRange(double min, double max, double step):eMin(min), eMax(max), eStep(step) {}
	};
private:
	ModelMediumDefinition m_ModelMedium;
	double m_eMinimalFieldAmplitude;
	std::unique_ptr<FrequencySetGeneric> m_pFrequencySet;
	double m_eIterationChange;
	unsigned m_eIterationStep;
	unsigned m_nMaxIteration = std::numeric_limits<unsigned>::max();
public:
	ModelDomainData() = default;
	const ModelMediumDefinition& GetModelDomainDefinition() const
	{
		return m_ModelMedium;
	}
	template <class ModelMediumDefinitionT>
	ModelDomainData& SetModelDomainDefinition(ModelMediumDefinitionT&& definition)
	{
		m_ModelMedium = std::forward<ModelMediumDefinitionT>(definition);
		return *this;
	}
	double GetMinimalFieldAmplitude() const
	{
		return m_eMinimalFieldAmplitude;
	}
	ModelDomainData& SetMinimalFieldAmplitude(double val)
	{
		m_eMinimalFieldAmplitude = val;
		return *this;
	}
	const FrequencySetGeneric& GetFrequencySet() const
	{
		return *m_pFrequencySet;
	}
	template <class FrequencySetT>
	ModelDomainData& SetFrequencySet(FrequencySetT&& set)
	{
		m_pFrequencySet.reset(new std::decay_t<FrequencySetT> (std::forward<FrequencySetT>(set)));
		return *this;
	}
	double GetIterationChange() const
	{
		return m_eIterationChange;
	}
	ModelDomainData& SetIterationChange(double val)
	{
		m_eIterationChange = val;
		return *this;
	}
	unsigned GetIterationStep() const
	{
		return m_eIterationStep;
	}
	ModelDomainData& SetIterationStep(unsigned val)
	{
		m_eIterationStep = val;
		return *this;
	}
	unsigned GetMaxIteration() const
	{
		return m_nMaxIteration;
	}
	ModelDomainData& SetMaxIteration(unsigned val)
	{
		m_nMaxIteration = val;
		return *this;
	}
	bool IsMaxIterationSpecified() const
	{
		return m_nMaxIteration != std::numeric_limits<unsigned>::max();
	}
};

struct AntennaTypeDefinition
{
	struct FrequencyResponseGeneric
	{
		virtual ~FrequencyResponseGeneric() {}
	};
	struct ExpressionFrequencyResponse:FrequencyResponseGeneric
	{
		std::string m_strExpr;
		ExpressionFrequencyResponse() = default;
		explicit ExpressionFrequencyResponse(const std::wstring& expr):m_strExpr(encode_string(expr)) {}
	};
	struct TableFrequencyResponse:FrequencyResponseGeneric
	{
		typedef std::map<double, double> map_type;
		map_type m_table;
		TableFrequencyResponse() = default;
		explicit TableFrequencyResponse(map_type&& fr):m_table(std::move(fr)) {}
	};
	struct RadiationPatternGeneric
	{
		virtual ~RadiationPatternGeneric() {}
	};
	struct ExpressionRadiationPattern:RadiationPatternGeneric
	{
		std::string m_strExpr;
		ExpressionRadiationPattern() = default;
		explicit ExpressionRadiationPattern(const std::wstring& expr):m_strExpr(encode_string(expr)) {}
	};
	struct TableRadiationPattern:RadiationPatternGeneric
	{
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
		TableRadiationPattern() = default;
		explicit TableRadiationPattern(map_type&& rp):m_table(std::move(rp)) {}
	};
private:
	std::unique_ptr<FrequencyResponseGeneric> m_pFr;
	std::unique_ptr<RadiationPatternGeneric> m_pRp;
	double m_eGainAmplitude, m_eGainPhase;
	double m_ePolarization;
	double m_eFarField = std::numeric_limits<double>::infinity();
public:
	AntennaTypeDefinition() = default;
	const FrequencyResponseGeneric& GetFrequencyResponse() const
	{
		return *m_pFr;
	}
	template <class FrequencyResponseT>
	AntennaTypeDefinition& SetFrequencyResponse(FrequencyResponseT&& fr)
	{
		m_pFr.reset(new std::decay_t<FrequencyResponseT>(std::forward<FrequencyResponseT>(fr)));
		return *this;
	}
	const RadiationPatternGeneric& GetRadiationPattern() const
	{
		return *m_pRp;
	}
	template <class RadiationPatternT>
	AntennaTypeDefinition& SetRadiationPattern(RadiationPatternT&& rp)
	{
		m_pRp.reset(new std::decay_t<RadiationPatternT>(std::forward<RadiationPatternT>(rp)));
		return *this;
	}
	double GetGainAmplitude() const
	{
		return m_eGainAmplitude;
	}
	AntennaTypeDefinition& SetGainAmplitude(double val)
	{
		m_eGainAmplitude = val;
		return *this;
	}
	double GetGainPhase() const
	{
		return m_eGainPhase;
	}
	AntennaTypeDefinition& SetGainPhase(double val)
	{
		m_eGainPhase = val;
		return *this;
	}
	double GetPolarization() const
	{
		return m_ePolarization;
	}
	AntennaTypeDefinition& SetPolarization(double val)
	{
		m_ePolarization = val * Chusov::Math::pi<double>() / 180.;
		return *this;
	}
	double GetFarField() const
	{
		return m_eFarField;
	}
	AntennaTypeDefinition& SetFarField(double val)
	{
		m_eFarField = val;
		return *this;
	}
	bool IsFarFieldSpecified() const
	{
		return m_eFarField != std::numeric_limits<double>::infinity();
	}
};

struct AntennaDefinition
{
	AntennaTypeDefinition m_type;
};

struct SourceDomainDataDefinition
{
	AntennaDefinition antenna;
	double eInputPower;
};

struct PolyDomainDataDefinition
{
	MediumDefinition medium;
};

static MediumDefinition LoadMediumData(text_istream& is, const xml::tag& opening_tag)
{
	MediumDefinition medium = {MediumDefinition::default_permittivity, MediumDefinition::default_permeability, MediumDefinition::default_coductivity,
		MediumDefinition::default_electric_loss, MediumDefinition::default_magnetic_loss};

	bool fPermittivity = false, fPermeability = false, fConductivity = false,
		fElectricLoss = false, fMagneticLoss = false;
	assert(opening_tag.name() == L"medium");
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"permittivity")
		{
			medium.ePermittivity = xml::get_tag_value<double>(is, tag);
			if (fPermittivity)
				throw ambiguous_specification(is.get_resource_locator(), L"permittivity");
			fPermittivity = true;
		}else if (tag.name() == L"permeability")
		{
			medium.ePermeability = xml::get_tag_value<double>(is, tag);
			if (fPermeability)
				throw ambiguous_specification(is.get_resource_locator(), L"permeability");
			fPermeability = true;
		}else if (tag.name() == L"conductivity")
		{
			medium.eConductivity = xml::get_tag_value<double>(is, tag);
			if (fConductivity)
				throw ambiguous_specification(is.get_resource_locator(), L"conductivity");
			fConductivity = true;
		}else if (tag.name() == L"electricLoss")
		{
			medium.eElectricLoss = xml::get_tag_value<double>(is, tag);
			if (fElectricLoss)
				throw ambiguous_specification(is.get_resource_locator(), L"electricLoss");
			fElectricLoss = true;
		}else if (tag.name() == L"magneticLoss")
		{
			medium.eMagneticLoss = xml::get_tag_value<double>(is, tag);
			if (fMagneticLoss)
				throw ambiguous_specification(is.get_resource_locator(), L"magneticLoss");
			fMagneticLoss = true;
		}else if (tag.name() == opening_tag.name() && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else if (tag.is_comment())
			continue;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	return medium;
}

static binary_ostream& operator<<(binary_ostream& os, const MediumDefinition& medium)
{
	return os << MediumDatumTypeId::Permittivity << medium.ePermittivity << MediumDatumTypeId::Permeability << medium.ePermeability <<
		MediumDatumTypeId::Conductivity << medium.eConductivity << MediumDatumTypeId::ElectricLoss << medium.eElectricLoss <<
		MediumDatumTypeId::MagneticLoss << medium.eMagneticLoss << std::uint32_t();
}

static ModelMediumDefinition LoadModelMediumData(text_istream& is, const xml::tag& opening_tag)
{
	bool fRefractionChange = false, fMedium = false;
	ModelMediumDefinition result;
	assert(opening_tag.name() == L"model_medium");
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"refraction_change")
		{
			result.eRefractionChange = xml::get_tag_value<double>(is, tag);
			if (fRefractionChange)
				throw ambiguous_specification(is.get_resource_locator(), L"refraction_change");
			fRefractionChange = true;
		}else if (tag.name() == L"medium")
		{
			if (fMedium)
				throw ambiguous_specification(is.get_resource_locator(), L"medium");
			result.medium = LoadMediumData(is, tag);
			fMedium = true;
		}else if (tag.name() == L"model_medium" && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else if (!tag.is_comment())
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	if (!fRefractionChange)
		throw xml_tag_not_found(is.get_resource_locator(), L"refraction_change");
	if (!fMedium)
		throw xml_tag_not_found(is.get_resource_locator(), L"medium");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const ModelMediumDefinition& def)
{
	return os << ModelMediumDatumTypeId::RefractionChange << def.eRefractionChange << ModelMediumDatumTypeId::Medium << def.medium
		<< std::uint32_t();
}

static ModelDomainData LoadModelDomainData(text_istream& is, const xml::tag& opening_tag)
{
	bool fMinimalFieldAmplitude = false, fAverageIterationChange = false, fIterationCheckStep = false, fSpectrum = false, fModelMedium = false, fMaxIteration = false;
	ModelDomainData result;
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"minimal_field_amplitude")
		{
			if (fMinimalFieldAmplitude)
				throw ambiguous_specification(is.get_resource_locator(), L"minimal_field_amplitude");
			result.SetMinimalFieldAmplitude(xml::get_tag_value<double>(is, tag));
			fMinimalFieldAmplitude = true;
		}else if (tag.name() == L"minimal_change_of_average_iteration_result")
		{
			if (fAverageIterationChange)
				throw ambiguous_specification(is.get_resource_locator(), L"minimal_change_of_average_iteration_result");
			result.SetIterationChange(xml::get_tag_value<double>(is, tag));
			fAverageIterationChange = true;
		}else if (tag.name() == L"step_for_iteration_result_check")
		{
			if (fIterationCheckStep)
				throw ambiguous_specification(is.get_resource_locator(), L"minimal_change_of_average_iteration_result");
			result.SetIterationStep(xml::get_tag_value<unsigned>(is, tag));
			fIterationCheckStep = true;
		}else if (tag.name() == L"max_iterations")
		{
			if (fMaxIteration)
				throw ambiguous_specification(is.get_resource_locator(), L"max_iterations");
			result.SetMaxIteration(xml::get_tag_value<unsigned>(is, tag));
			fMaxIteration = true;
		}else if (tag.name() == L"frequency_set")
		{
			if (tag.is_closing_tag() || tag.is_unary_tag())
				throw improper_xml_tag(is.get_resource_locator(), tag.name());
			if (fSpectrum)
				throw ambiguous_specification(is.get_resource_locator(), L"frequency_set");
			std::list<double> lstSpectrum;
			while (true)
			{
				tag = xml::tag(is);
				if (tag.is_comment())
					continue;
				else if (tag.name() == L"frequency")
					lstSpectrum.emplace_back(xml::get_tag_value<double>(is, tag));
				else if (tag.name() == L"frequency_set")
				{
					if (!tag.is_closing_tag())
						throw improper_xml_tag(is.get_resource_locator(), tag.name());
					break;
				}else if (!tag.is_comment())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
			}
			if (lstSpectrum.empty())
				throw improper_xml_tag(is.get_resource_locator(), L"frequency_set");
			result.SetFrequencySet(ModelDomainData::FrequencySet(std::move(lstSpectrum)));
			fSpectrum = true;
		}else if (tag.name() == L"frequency_range")
		{
			bool fRangeMin = false, fRangeMax = false, fRangeStep = false;
			double eRangeMin, eRangeMax, eRangeStep;
			if (tag.is_closing_tag() || tag.is_unary_tag())
				throw improper_xml_tag(is.get_resource_locator(), tag.name());
			if (fSpectrum)
				throw ambiguous_specification(is.get_resource_locator(), L"frequency_range");
			while (true)
			{
				tag = xml::tag(is);
				if (tag.is_comment())
					continue;
				else if (tag.name() == L"min")
				{
					if (fRangeMin)
						throw ambiguous_specification(is.get_resource_locator(), L"min");
					eRangeMin = xml::get_tag_value<double>(is, tag);
					fRangeMin = true;
				}else if (tag.name() == L"max")
				{
					if (fRangeMax)
						throw ambiguous_specification(is.get_resource_locator(), L"max");
					eRangeMax = xml::get_tag_value<double>(is, tag);
					fRangeMax = true;
				}else if (tag.name() == L"step")
				{
					if (fRangeStep)
						throw ambiguous_specification(is.get_resource_locator(), L"step");
					eRangeStep = xml::get_tag_value<double>(is, tag);
					fRangeStep = true;
				}else if (tag.name() == L"frequency_range" && tag.is_closing_tag() && !tag.is_unary_tag())
					break;
				else if (!tag.is_comment())
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
			}
			if (!fRangeMin)
				throw xml_tag_not_found(is.get_resource_locator(), L"min");
			if (!fRangeMax)
				throw xml_tag_not_found(is.get_resource_locator(), L"max");
			if (!fRangeStep)
				throw xml_tag_not_found(is.get_resource_locator(), L"step");
			result.SetFrequencySet(ModelDomainData::FrequencyRange{eRangeMin, eRangeMax, eRangeStep});
			fSpectrum = true;
		}else if (tag.name() == L"model_medium")
		{
			if (fModelMedium)
				throw ambiguous_specification(is.get_resource_locator(), L"model_medium");
			result.SetModelDomainDefinition(LoadModelMediumData(is, tag));
			fModelMedium = true;
		}else if (tag.name() == opening_tag.name() && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	};
	if (!fMinimalFieldAmplitude)
		throw xml_tag_not_found(is.get_resource_locator(), L"minimal_field_amplitude");
	if (!fAverageIterationChange)
		throw xml_tag_not_found(is.get_resource_locator(), L"minimal_change_of_average_iteration_result");
	if (!fIterationCheckStep)
		throw xml_tag_not_found(is.get_resource_locator(), L"step_for_iteration_result_check");
	if (!fSpectrum)
		throw xml_tag_not_found(is.get_resource_locator(), L"frequency_set or frequency_range");
	if (!fModelMedium)
		throw xml_tag_not_found(is.get_resource_locator(), L"model_medium");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const ModelDomainData& def)
{
	os << InputModelDatumTypeId::ModelMedium << def.GetModelDomainDefinition();
	os << InputModelDatumTypeId::MinimalFieldAmplitude << def.GetMinimalFieldAmplitude()
		<< InputModelDatumTypeId::IterationAverageResultingChange << def.GetIterationChange() << InputModelDatumTypeId::IterationAverageResultingStep << def.GetIterationStep();
	if (typeid(def.GetFrequencySet()) == typeid(ModelDomainData::FrequencyRange))
	{
		const auto& range = dynamic_cast<const ModelDomainData::FrequencyRange&>(def.GetFrequencySet());
		os << InputModelDatumTypeId::FrequencyRange << range.eMin << range.eMax << range.eStep;
	}else if (typeid(def.GetFrequencySet()) == typeid(ModelDomainData::FrequencySet))
	{
		const auto& set = dynamic_cast<const ModelDomainData::FrequencySet&>(def.GetFrequencySet());
		os << InputModelDatumTypeId::FrequencySet << std::uint32_t(set.m_container.size());
		for (auto f:set.m_container)
			os << f;
	}else
		throw std::logic_error("Unexpected data resulting from XML parsing");
	if (def.IsMaxIterationSpecified())
		os << InputModelDatumTypeId::MaxIteration << def.GetMaxIteration();
	return os << std::uint32_t();
}

static AntennaTypeDefinition LoadAntennaType(text_istream& is, const xml::tag& opening_tag)
{
	bool fFrequencyResponseSpecified = false, fRadiationPatternSpecified = false, fAntennaGainSpecified = false, fPolarizationSpecified = false, fFarFieldspecified = false;
	assert(opening_tag.name() == L"antenna_type");
	AntennaTypeDefinition result;
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"frequency_response")
		{
			if (fFrequencyResponseSpecified)
				throw ambiguous_specification(is.get_resource_locator(), L"frequency_response");
			while (true)
			{
				tag = xml::tag(is);
				if (tag.is_comment())
					continue;
				else if (tag.name() == L"expression")
				{
					if (fFrequencyResponseSpecified)
						throw ambiguous_specification(is.get_resource_locator(), L"frequency_response");
					result.SetFrequencyResponse(AntennaTypeDefinition::ExpressionFrequencyResponse(xml::get_tag_value<std::wstring>(is, tag)));
					fFrequencyResponseSpecified = true;
				}else if (tag.name() == L"table")
				{
					if (fFrequencyResponseSpecified)
						throw ambiguous_specification(is.get_resource_locator(), L"frequency_response");
					AntennaTypeDefinition::TableFrequencyResponse::map_type fr;
					while (true)
					{
						tag = xml::tag(is);
						if (tag.is_comment())
							continue;
						else if (tag.name() == L"fr_row")
						{
							auto freq = tag.attribute(L"frequency");
							if (freq.empty())
								throw xml_attribute_not_found(is.get_resource_locator(), L"frequency");
							auto value = xml::get_tag_value<double>(is, tag);
							if (!fr.emplace(std::stod(freq), value).second)
								throw ambiguous_specification(is.get_resource_locator(), L"fr_row");
						}else if (tag.name() == L"table" && tag.is_closing_tag() && !tag.is_unary_tag())
							break;
						else
							throw improper_xml_tag(is.get_resource_locator(), tag.name());
					}
					result.SetFrequencyResponse(AntennaTypeDefinition::TableFrequencyResponse(std::move(fr)));
					fFrequencyResponseSpecified = true;
				}else if (tag.name() == L"frequency_response" && tag.is_closing_tag() && !tag.is_unary_tag())
					break;
				else
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
			};
			if (!fFrequencyResponseSpecified)
				throw xml_tag_not_found(is.get_resource_locator(), L"expression or table");
		}else if (tag.name() == L"radiation_pattern")
		{
			if (fRadiationPatternSpecified)
				throw ambiguous_specification(is.get_resource_locator(), L"radiation_pattern");
			while (true)
			{
				tag = xml::tag(is);
				if (tag.is_comment())
					continue;
				else if (tag.name() == L"expression")
				{
					if (fRadiationPatternSpecified)
						throw ambiguous_specification(is.get_resource_locator(), L"radiation_pattern");
					result.SetRadiationPattern(AntennaTypeDefinition::ExpressionRadiationPattern(xml::get_tag_value<std::wstring>(is, tag)));
					fRadiationPatternSpecified = true;
				}else if (tag.name() == L"table")
				{
					if (fRadiationPatternSpecified)
						throw ambiguous_specification(is.get_resource_locator(), L"radiation_pattern");
					AntennaTypeDefinition::TableRadiationPattern::map_type rp;
					while (true)
					{
						tag = xml::tag(is);
						if (tag.is_comment())
							continue;
						else if (tag.name() == L"rp_row")
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
							if (!rp.emplace(std::make_tuple(std::stod(freq), std::stod(az), std::stod(zn)), value).second)
								throw ambiguous_specification(is.get_resource_locator(), L"rp_row");
						}else if (tag.name() == L"table" && tag.is_closing_tag() && !tag.is_unary_tag())
							break;
						else
							throw improper_xml_tag(is.get_resource_locator(), tag.name());
					}
					result.SetRadiationPattern(AntennaTypeDefinition::TableRadiationPattern(std::move(rp)));
					fRadiationPatternSpecified = true;
				}else if (tag.name() == L"radiation_pattern")
				{
					if (!tag.is_closing_tag() || tag.is_unary_tag())
						throw improper_xml_tag(is.get_resource_locator(), tag.name());
					break;
				}else
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
			};
			if (!fRadiationPatternSpecified)
				throw xml_tag_not_found(is.get_resource_locator(), L"expression or table");
		}else if (tag.name() == L"antenna_gain")
		{
			if (fAntennaGainSpecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			bool fMagnitudeSpecified = false, fPhaseSpecified = false;
			double eMagnitude = 1, ePhase = 0;
			while (true)
			{
				tag = xml::tag(is);
				if (tag.is_comment())
					continue;
				else if (tag.name() == L"magnitude")
				{
					if (fMagnitudeSpecified)
						throw ambiguous_specification(is.get_resource_locator(), tag.name());
					eMagnitude = xml::get_tag_value<double>(is, tag);
					fMagnitudeSpecified = true;
				}else if (tag.name() == L"phase")
				{
					if (fPhaseSpecified)
						throw ambiguous_specification(is.get_resource_locator(), tag.name());
					ePhase = xml::get_tag_value<double>(is, tag);
					fPhaseSpecified = true;
				}else if (tag.name() == L"antenna_gain" && tag.is_closing_tag() && !tag.is_unary_tag())
					break;
				else
					throw improper_xml_tag(is.get_resource_locator(), tag.name());
			};
			result.SetGainAmplitude(eMagnitude).SetGainPhase(ePhase);
			fAntennaGainSpecified = true;
		}else if (tag.name() == L"polarization_angle")
		{
			if (fPolarizationSpecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			result.SetPolarization(xml::get_tag_value<double>(is, tag));
			fPolarizationSpecified = true;
		}else if (tag.name() == L"far_field_distance")
		{
			if (fFarFieldspecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			result.SetFarField(xml::get_tag_value<double>(is, tag));
			fFarFieldspecified = true;
		}else if (tag.name() == L"antenna_type" && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	if (!fFrequencyResponseSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"frequency_response");
	if (!fRadiationPatternSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"radiation_pattern");
	if (!fAntennaGainSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"antenna_gain");
	if (!fPolarizationSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"polarization_angle");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const AntennaTypeDefinition& def)
{
	os << SourceDatumTypeId::AntennaGain << def.GetGainAmplitude() << def.GetGainPhase();
	os << SourceDatumTypeId::Polarization << def.GetPolarization();

	if (typeid(def.GetFrequencyResponse()) == typeid(AntennaTypeDefinition::ExpressionFrequencyResponse))
	{
		const auto& fr = dynamic_cast<const AntennaTypeDefinition::ExpressionFrequencyResponse&>(def.GetFrequencyResponse());
		os << SourceDatumTypeId::ExpressionFrequencyResponse << std::uint32_t(fr.m_strExpr.size());
		os.write(fr.m_strExpr.data(), fr.m_strExpr.size());
	}else if (typeid(def.GetFrequencyResponse()) == typeid(AntennaTypeDefinition::TableFrequencyResponse))
	{
		const auto& fr = dynamic_cast<const AntennaTypeDefinition::TableFrequencyResponse&>(def.GetFrequencyResponse());
		os << SourceDatumTypeId::TableFrequencyResponse << std::uint32_t(fr.m_table.size());
		for (auto pr:fr.m_table)
			os << pr.first << pr.second;
	}else
		throw std::logic_error("Unexpected data resulting from XML parsing");
	if (typeid(def.GetRadiationPattern()) == typeid(AntennaTypeDefinition::ExpressionRadiationPattern))
	{
		const auto& rp = dynamic_cast<const AntennaTypeDefinition::ExpressionRadiationPattern&>(def.GetRadiationPattern());
		os << SourceDatumTypeId::ExpressionRadiationPattern << std::uint32_t(rp.m_strExpr.size());
		os.write(rp.m_strExpr.data(), rp.m_strExpr.size());
	}else if (typeid(def.GetRadiationPattern()) == typeid(AntennaTypeDefinition::TableRadiationPattern))
	{
		using std::get;
		const auto& rp = dynamic_cast<const AntennaTypeDefinition::TableRadiationPattern&>(def.GetRadiationPattern());
		os << SourceDatumTypeId::TableRadiationPattern << std::uint32_t(rp.m_table.size());
		for (auto pr:rp.m_table)
			os << get<0>(pr.first) << get<1>(pr.first) << get<2>(pr.first) << pr.second;
	}else
		throw std::logic_error("Unexpected data resulting from XML parsing");
	if (def.IsFarFieldSpecified())
		os << SourceDatumTypeId::FarField << def.GetFarField();
	return os;
}

static AntennaDefinition LoadAntenna(text_istream& is, const xml::tag& opening_tag)
{
	AntennaDefinition result;
	bool fAntennaTypeSpecified = false;
	assert(opening_tag.name() == L"antenna");
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"antenna_type")
		{
			if (fAntennaTypeSpecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			result.m_type = LoadAntennaType(is, tag);
			fAntennaTypeSpecified = true;
		}else if (tag.name() == L"antenna" && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	if (!fAntennaTypeSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"antenna_type");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const AntennaDefinition& def)
{
	return os << def.m_type;
}

static SourceDomainDataDefinition LoadSourceDomainData(text_istream& is, const xml::tag& opening_tag)
{
	bool fAntennaSpecified = false, fPowerSpecified = false;
	SourceDomainDataDefinition result;
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"antenna")
		{
			if (fAntennaSpecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			result.antenna = LoadAntenna(is, tag);
			fAntennaSpecified = true;
		}else if (tag.name() == L"input_power")
		{
			if (fPowerSpecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			result.eInputPower = xml::get_tag_value<double>(is, tag);
			fPowerSpecified = true;
		}else if (tag.name() == opening_tag.name() && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	if (!fAntennaSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"antenna");
	if (!fPowerSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"input_power");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const SourceDomainDataDefinition& def)
{
	return os << SourceDatumTypeId::InputPower << def.eInputPower << def.antenna << std::uint32_t();
}

static PolyDomainDataDefinition LoadPolyDomainData(text_istream& is, const xml::tag& opening_tag)
{
	PolyDomainDataDefinition result;
	bool fMediumSpecified = false;
	while (true)
	{
		auto tag = xml::tag(is);
		if (tag.is_comment())
			continue;
		else if (tag.name() == L"medium")
		{
			if (fMediumSpecified)
				throw ambiguous_specification(is.get_resource_locator(), tag.name());
			result.medium = LoadMediumData(is, tag);
			fMediumSpecified = true;
		}else if (tag.name() == opening_tag.name() && tag.is_closing_tag() && !tag.is_unary_tag())
			break;
		else
			throw improper_xml_tag(is.get_resource_locator(), tag.name());
	}
	if (!fMediumSpecified)
		throw xml_tag_not_found(is.get_resource_locator(), L"medium");
	return result;
}

static binary_ostream& operator<<(binary_ostream& os, const PolyDomainDataDefinition& def)
{
	return os << def.medium;
}

void radio_hf_convert::model_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os)
{
	os << LoadModelDomainData(is, opening_tag);
}

void radio_hf_convert::poly_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os)
{
	os << LoadPolyDomainData(is, opening_tag);
}

void radio_hf_convert::source_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os)
{
	os << LoadSourceDomainData(is, opening_tag);
}

void radio_hf_convert::constant_poly_domain_data(ConstantDomainDataId id, binary_ostream& os)
{
	switch (id)
	{
	case ConstantDomainDataId::SurfaceLand:
		os << PolyDomainDataDefinition{{25, 0.01, 3E-3, 0, 0}};
		return;
	case ConstantDomainDataId::SurfaceWater:
		//seawater, temperature: 0 C, salinity: 35 promille, 1 atm
		os << PolyDomainDataDefinition{{
				78.39, //real part of permeability. https://agupubs.onlinelibrary.wiley.com/doi/full/10.1002/2015RS005776
				1.256627E-6,
				2.797264, //conductivity. https://agupubs.onlinelibrary.wiley.com/doi/pdf/10.1029/97RS02223
				0, //see http://www.dtic.mil/dtic/tr/fulltext/u2/a046687.pdf
				0
			}};
	};
}