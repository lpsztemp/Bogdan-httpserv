#include "xml_parser.h"
#include "text_streams.h"
#include "binary_streams.h"
#include <string>

#ifndef IMPL_ARCH_AC_DOMAIN_XML2BIN_H_
#define IMPL_ARCH_AC_DOMAIN_XML2BIN_H_

struct arch_ac_convert
{
	//REQUIRED METHODS
	static const std::string& domain_name();

	//NON MANDATORY METHODS
	void model_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os);
	void face_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os);
	void source_domain_data(const xml::tag& opening_tag, text_istream& is, binary_ostream& os);
};

#endif //IMPL_ARCH_AC_DOMAIN_XML2BIN_H_
