#include "binary_streams.h"
#include "xml2bin.h"
#include "domain_converter.h"

#ifndef XML2BIN_HGTOPTIMIZER_H_
#define XML2BIN_HGTOPTIMIZER_H_

struct hgt_t
{
	virtual ~hgt_t() = default;

	struct attributes_t
	{
		HGT_RESOLUTION_DATA resolution;
		short min_height; //maximal height of the HGT after the shift by height_offset meters
		short max_height; //minimal height of the HGT after the shift by height_offset meters
		short height_offset; //a positive shift in heights required to move the input geometry model from negative area outside of the model boundaries
	};
	struct conversion_statistics_t
	{
		unsigned poly_count;
	};

	static std::unique_ptr<hgt_t> read(std::istream& is_data, IDomainConverter& converter);
	virtual const attributes_t& attributes() const = 0;
	virtual conversion_statistics_t write(binary_ostream& os) const = 0;
protected:
	hgt_t() = default;
	hgt_t(const hgt_t&) = default;
	hgt_t(hgt_t&&) = default;
	hgt_t& operator=(const hgt_t&) = default;
	hgt_t& operator=(hgt_t&&) = default;
};

#endif //XML2BIN_HGTOPTIMIZER_H_
