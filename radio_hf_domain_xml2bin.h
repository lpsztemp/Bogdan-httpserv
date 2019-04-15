#include <list>
#include <algorithm>
#include "basedefs.h"
#include "binary_streams.h"
#include "xml_parser.h"

#ifndef IMPL_RADIO_HF_DOMAIN_XML2BIN_H_
#define IMPL_RADIO_HF_DOMAIN_XML2BIN_H_

struct radio_hf_convert
{
	//REQUIRED METHODS
	static const std::string& domain_name();

	//NON MANDATORY METHODS
	void model_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os);
	void poly_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os);
	void source_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os);

	//hgt
	//must be thread safe
	void constant_poly_domain_data(ConstantDomainDataId id, binary_ostream& os);
};

#endif //IMPL_RADIO_HF_DOMAIN_XML2BIN_H_
