#include <cstdint>
#include <iterator>
#include <memory>
//#include <camaas/base_config.h>

#ifndef XML2BIN_BASEDEFS_H
#define XML2BIN_BASEDEFS_H

enum class ConstantDomainDataId
{
	SurfaceLand, SurfaceWater
};

namespace CAMaaS 
{
typedef std::uint32_t size_type;
typedef std::int32_t return_code_t;
enum status_id_t:std::uint32_t
{
	status_not_found,
	status_running,
	status_idle
};
enum ObjectTypeId:std::uint32_t
{
	ObjectPoly,
	ObjectSource,
	ObjectPlain
};
typedef enum tag_Units:std::uint32_t
{
	CHU_MILLIMETERS,
	CHU_METERS, /*default*/
	CHU_KILOMETERS,
	CHU_INVALID_METRIC
} Units, MetricId;
}

namespace Chusov
{
namespace Math
{
template <class T>
constexpr T pi()
{
	return T(3.14);
}
}}

#ifdef _WIN32
#define LIBIMPORT __declspec(dllimport)
#define PLATFORM_NATIVE_CALLING_CONVENTION __stdcall
#endif //_WIN32

#define ChsvFailed(err) ((err) < 0)

#endif //XML2BIN_BASEDEFS_H
