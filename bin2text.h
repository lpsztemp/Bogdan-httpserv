#include <cstdint>
#include <iostream>
#include <string>
#include <stdexcept>
#include "binary_streams.h"

#ifndef BIN2TEXT_H_
#define BIN2TEXT_H_

namespace Implementation
{
	void bin2text_arch_ac(std::istream& is, std::ostream& os);
	void bin2text_radio_hf(std::istream& is, std::ostream& os);
}

template <class DomainString>
void bin2text(const DomainString& strDomain, std::istream& is, std::ostream& os)
{
	using namespace Implementation;
	if (strDomain == "arch_ac")
		bin2text_arch_ac(is, os);
	else if (strDomain == "radio_hf")
		bin2text_radio_hf(is, os);
	else
		throw std::invalid_argument("Invalid domain name");
}

#endif //BIN2TEXT_H_
