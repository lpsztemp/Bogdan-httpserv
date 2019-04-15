#include "bin2text.h"
#include <type_traits>

template <class T>
static auto read_as(std::istream& is) -> std::enable_if_t<std::is_pod_v<T>, T>
{
	T val;
	is.read(reinterpret_cast<char*>(std::addressof(val)), sizeof(T));
	return val;
}

template <class T>
static auto read_buf(std::istream& is, T* buf, std::size_t cBuf) -> std::enable_if_t<std::is_void_v<T> || std::is_pod_v<T>>
{
	is.read(reinterpret_cast<char*>(buf), cBuf * sizeof(T));
}

namespace Implementation
{

void bin2text_arch_ac(std::istream& is, std::ostream& os)
{
	typedef std::uint32_t size_type;
	auto cPlainCount = read_as<size_type>(is);
	for (size_type iPlain = 0; iPlain < cPlainCount; ++iPlain)
	{
		auto cbName = read_as<size_type>(is);
		auto strName = std::string(cbName, char());
		read_buf(is, strName.data(), cbName);
		os << "Plain name: " << strName << "\n";
		{
			os << "Control points:\n";
			auto cColumns = read_as<size_type>(is), cRows = read_as<size_type>(is);
			for (size_type iRow = 0; iRow < cRows; ++iRow)
			{
				auto x = read_as<double>(is);
				auto y = read_as<double>(is);
				auto z = read_as<double>(is);
				os << '{' << x << ", " << y << ", " << z << '}';
				for (size_type iCol = 1; iCol < cColumns; ++iCol)
				{
					auto x = read_as<double>(is);
					auto y = read_as<double>(is);
					auto z = read_as<double>(is);
					os << ",\t{" << x << ", " << y << ", " << z << "}";
				}
				os << '\n';
			}
		}
		for (auto iFreq = 0; iFreq < 6 /*frequencies*/; ++iFreq)
		{
			auto eF = read_as<double>(is);
			os << "Results at " << eF << "Hz:\n";
			auto cColumns = read_as<size_type>(is), cRows = read_as<size_type>(is);
			for (size_type iRow = 0; iRow < cRows; ++iRow)
			{
				os << read_as<double>(is);
				for (size_type iCol = 1; iCol < cColumns; ++iCol)
					os << "\t" << read_as<double>(is);
				os << "\n";
			}
		}
	}
}

void bin2text_radio_hf(std::istream& is, std::ostream& os)
{
	typedef std::uint32_t size_type;
	auto cPlainCount = read_as<size_type>(is);
	for (size_type iPlain = 0; iPlain < cPlainCount; ++iPlain)
	{
		auto cbName = read_as<size_type>(is);
		auto strName = std::string(cbName, char());
		read_buf(is, strName.data(), cbName);
		os << "Plain name: " << strName << "\n";
		auto cColumns = read_as<size_type>(is);
		auto cRows = read_as<size_type>(is);
		os << "Control points:\n";
		for (size_type iRow = 0; iRow < cRows; ++iRow)
		{
			os << '{' << read_as<double>(is);
			os << ", " << read_as<double>(is);
			os << ", " << read_as<double>(is) << '}';
			for (size_type iCol = 1; iCol < cColumns; ++iCol)
			{
				os << ",\t{" << read_as<double>(is);
				os << ", " << read_as<double>(is);
				os << ", " << read_as<double>(is) << '}';
			}
			os << "\n";
		}
		auto cFrequencies = read_as<size_type>(is);
		for (size_type iFreq = 0; iFreq < cFrequencies /*frequencies*/; ++iFreq)
		{
			auto eFrequency = read_as<double>(is);
			os << "Results at " << eFrequency << "Hz:\n";
			for (size_type iRow = 0; iRow < cRows; ++iRow)
			{
				auto eExAmp = read_as<double>(is);
				auto eExArg = read_as<double>(is);
				auto eEyAmp = read_as<double>(is);
				auto eEyArg = read_as<double>(is);
				auto eEzAmp = read_as<double>(is);
				auto eEzArg = read_as<double>(is);
				os << "({"
					<< eExAmp << ", " << eExArg << "}, {"
					<< eEyAmp << ", " << eEyArg << "}, {"
					<< eEzAmp << ", " << eEzArg << "})";
				for (size_type iCol = 1; iCol < cColumns; ++iCol)
				{
					auto eExAmp = read_as<double>(is);
					auto eExArg = read_as<double>(is);
					auto eEyAmp = read_as<double>(is);
					auto eEyArg = read_as<double>(is);
					auto eEzAmp = read_as<double>(is);
					auto eEzArg = read_as<double>(is);
					os << ",\t({"
						<< eExAmp << ", " << eExArg << "}, {"
						<< eEyAmp << ", " << eEyArg << "}, {"
						<< eEzAmp << ", " << eEzArg << "})";
				}
				os << "\n";
			}
		}
	}
}

} // Implementation
