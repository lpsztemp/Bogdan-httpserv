#include "xml2bin.h"
#include "bin2text.h"

#include <list>
#include <string>
#include <stdexcept>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <locale>
#include <codecvt>
#include <memory>

//#include <chsvlib/chsverr.h>
//#include <camaas/domain_interaction.h>

//commands
enum CallId
{
	CreateModelId,
	CreateObjectId,
	UploadModelId,
	StartSimulationId,
	GetProcessStatusId,
	GetSimulationResultsId,
	StopSimulationId, /*not implemented*/
	CloseSimulationId /*does not do anything*/
};

//control entry point
extern "C" /*LIBIMPORT*/ CAMaaS::return_code_t PLATFORM_NATIVE_CALLING_CONVENTION ControlSystemEntryPoint(std::uint32_t nFunctionId,
	_In_bytecount_(cbPackedInputParams) const void* pPackedInputParams, CAMaaS::size_type cbPackedInputParams,
	_Deref_post_count_(*pcbPackedOutputParams) void** pPackedOutputParams, _Out_ CAMaaS::size_type* pcbPackedOutputParams)
{
	return -1;
}

extern "C" /*LIBIMPORT*/ CAMaaS::return_code_t PLATFORM_NATIVE_CALLING_CONVENTION ControlSystemFreeData(_In_ void* pData)
{
	return -1;
}

extern "C" /*LIBIMPORT*/ _Ret_count_(*pSize) const char* PLATFORM_NATIVE_CALLING_CONVENTION ControlSystemGetErrorDescription(CAMaaS::return_code_t code,
	_Out_opt_ CAMaaS::size_type* pSize)
{
	return "Unknown error";
}

extern "C" /*LIBIMPORT*/ CAMaaS::return_code_t PLATFORM_NATIVE_CALLING_CONVENTION ControlSystemSetErrorLogFile(_In_z_count_(cchFileName) const char* pszFileName,
	CAMaaS::size_type cchFileName, bool fEraseContent)
{
	return -1;
}


struct constrol_system_free_data_t
{
	void operator()(void* pData) const
	{
		ControlSystemFreeData(pData);
	}
};

class coded_exception:public std::runtime_error
{
	CAMaaS::return_code_t m_code;
	static std::string form_what(CAMaaS::return_code_t err)
	{
		std::ostringstream os;
		os << "Error " << std::hex << err << ": " << ControlSystemGetErrorDescription(err, nullptr) << ".";
		return os.str();
	}
public:
	CAMaaS::return_code_t code() const {return m_code;}
	coded_exception(CAMaaS::return_code_t err):std::runtime_error(form_what(err)), m_code(err) {}
};

struct failed_to_open_a_file:std::runtime_error
{
	failed_to_open_a_file(const std::string& file):std::runtime_error(form_what(file)) {}
private:
	static std::string form_what(const std::string& file)
	{
		std::ostringstream os;
		os << "Failed to open the file \"" << file << "\".";
		return os.str();
	}
};

struct unexpected_hgt_size:std::runtime_error
{
	unexpected_hgt_size():std::runtime_error("Unexpected HGT size. Only SRTM 30m and SRTM 90m are supported.") {}
};

struct invalid_usage:std::runtime_error
{
	invalid_usage():std::runtime_error(form_what()) {}
	template <class Description>
	invalid_usage(const Description& description):std::runtime_error(form_what(description)) {}
private:
	static inline std::string form_what()
	{
		return std::string("Invalid program usage. Use cmd_client --help to get help.");
	}
	template <class Description>
	static inline std::string form_what(const Description& description)
	{
		std::ostringstream os;
		os << "Invalid program usage: " << description << " Use cmd_client --help to get help.";
		return os.str();
	}
};

/*
Starts simulation for a given model in a specified domain system.
*/
template <class DomainSystemId, class ModelId>
std::string StartSimulation(const DomainSystemId& domain_system_id, const ModelId& model_id)
{
	buf_ostream os_request;
	os_request << CAMaaS::size_type(domain_system_id.size());
	os_request.write(domain_system_id.data(), domain_system_id.size());
	os_request << CAMaaS::size_type(model_id.size());
	os_request.write(model_id.data(), model_id.size());
	char* pProcessName;
	CAMaaS::size_type cchProcessName;
	CAMaaS::return_code_t nErr = ControlSystemEntryPoint(StartSimulationId, os_request.data(), CAMaaS::size_type(os_request.size()), (void**) &pProcessName, &cchProcessName);
	if (ChsvFailed(nErr))
		throw coded_exception(nErr);
	auto ret = std::string(pProcessName, cchProcessName);
	ControlSystemFreeData(pProcessName);
	return ret;
}

template <class ProcessId>
bool IsProcessRunning(const ProcessId& process_id)
{
	buf_ostream os_request;
	os_request << CAMaaS::size_type(process_id.size());
	os_request.write(process_id.data(), process_id.size());
	CAMaaS::status_id_t* pStatus;
	CAMaaS::size_type cbStatus;

	CAMaaS::return_code_t nErr = ControlSystemEntryPoint(std::uint32_t(GetProcessStatusId), os_request.data(), CAMaaS::size_type(os_request.size()),
		(void**) &pStatus, &cbStatus);
	if (ChsvFailed(nErr))
		throw coded_exception(nErr);
	auto nStatus = *pStatus;
	ControlSystemFreeData(pStatus);
	switch (nStatus)
	{
	case CAMaaS::status_not_found:
		throw std::invalid_argument("Process was not found");
	case CAMaaS::status_running:
		return true;
	case CAMaaS::status_idle:
		return false;
	default:
		throw std::logic_error("Unexpected process status code");
	};

}

template <class ProcessId>
void CloseSimulation(const ProcessId& process_id)
{
	buf_ostream os_request;
	os_request << CAMaaS::size_type(process_id.size());
	os_request.write(process_id.data(), process_id.size());
	void* pOutput;
	CAMaaS::size_type cbOutput;
	CAMaaS::return_code_t nErr = ControlSystemEntryPoint(CloseSimulationId, os_request.data(), CAMaaS::size_type(os_request.size()), &pOutput, &cbOutput);
	if (nErr != 0)
		throw coded_exception(nErr);
	if (cbOutput != 0)
		throw std::logic_error("Unexpected size of results");
}

template <class ProcessId>
void OutputBinaryResults(std::ostream& os, const ProcessId& process_id)
{
	buf_ostream os_request;
	os_request << CAMaaS::size_type(process_id.size());
	os_request.write(process_id.data(), process_id.size());
	std::uint32_t cbRes;
	std::unique_ptr<std::uint8_t, constrol_system_free_data_t> pRes;

	{
		std::uint8_t* l_pRes;
		auto nErr = ControlSystemEntryPoint(std::uint32_t(GetSimulationResultsId), os_request.data(), CAMaaS::size_type(os_request.size()), (void**)&l_pRes, &cbRes);
		if (nErr != 0)
			throw coded_exception(nErr);
		pRes.reset(l_pRes);
	}
	os.write(reinterpret_cast<const char*>(pRes.get()), cbRes);
}

template <class ProcessId, class DomainSystemId>
void OutputTextualizedResults(std::ostream& os, const ProcessId& process_id, const DomainSystemId& domain_system_id)
{
	buf_ostream os_request;
	os_request << CAMaaS::size_type(process_id.size());
	os_request.write(process_id.data(), process_id.size());
	CAMaaS::size_type cbRes;
	std::unique_ptr<std::uint8_t, constrol_system_free_data_t> pRes;

	{
		std::uint8_t* l_pRes;
		auto nErr = ControlSystemEntryPoint(std::uint32_t(GetSimulationResultsId), os_request.data(), CAMaaS::size_type(os_request.size()), (void**) &l_pRes, &cbRes);
		if (nErr != 0)
			throw coded_exception(nErr);
		pRes.reset(l_pRes);
	}
	auto is_response = buf_istream(pRes.get(), cbRes);
	//os << "\xef\xbb\xbf"; //UTF-8 BOM
	bin2text(domain_system_id, is_response, os);
}

template <class DefinitionPath>
std::string UploadBinaryModel(DefinitionPath model_definition)
{
	std::ifstream isModel(model_definition, std::ios_base::in | std::ios_base::binary);
	if (isModel.fail())
		throw failed_to_open_a_file(model_definition);
	auto prev_pos = isModel.tellg();
	auto cb = std::size_t(isModel.seekg(0, std::ios_base::end).tellg() - prev_pos);
	isModel.seekg(prev_pos);
	auto pData = std::make_unique<char[]>(cb);
	isModel.read(pData.get(), std::streamsize(cb));

	std::unique_ptr<char, constrol_system_free_data_t> pModel;
	CAMaaS::size_type cchModel;
	{
		char* l_pModel;
		CAMaaS::return_code_t nErr = ControlSystemEntryPoint(UploadModelId, pData.get(), CAMaaS::size_type(cb), (void**) &l_pModel, &cchModel);
		if (ChsvFailed(nErr))
			throw coded_exception(nErr);
		pModel.reset(l_pModel);
	}
	return std::string(pModel.get(), cchModel);
}

//return model_id
template <class DomainSystemId, class InputIteratorXmlBegin, class InputIteratorXmlEnd>
std::string UploadXMLModel(const DomainSystemId& domain_id, InputIteratorXmlBegin xml_paths_begin, InputIteratorXmlEnd xml_paths_end)
{
	std::string ret;
	buf_ostream os_binary;
	std::list<text_ifstream> lst_xml;
	for (auto itXml = xml_paths_begin; itXml != xml_paths_end; ++itXml)
	{
		auto is = text_ifstream(std::string_view(*itXml));
		if (is.fail())
			throw failed_to_open_a_file(*itXml);
		lst_xml.emplace_back(std::move(is));
	}
	xml2bin(domain_id, std::begin(lst_xml), std::end(lst_xml), os_binary);
	std::unique_ptr<char, constrol_system_free_data_t> pModel;
	CAMaaS::size_type cchModel;
	{
		char* l_pModel;
		CAMaaS::return_code_t nErr = ControlSystemEntryPoint(UploadModelId, os_binary.data(), CAMaaS::size_type(os_binary.size()), (void**) &l_pModel, &cchModel);
		if (ChsvFailed(nErr))
			throw coded_exception(nErr);
		pModel.reset(l_pModel);
	}
	return std::string(pModel.get(), cchModel);
}

template <class DomainSystemId, class InputIteratorXmlBegin, class InputIteratorXmlEnd, class HgtDefinition>
std::string UploadXMLModel(const DomainSystemId& domain_id, InputIteratorXmlBegin xml_is_begin, InputIteratorXmlEnd xml_is_end, const HgtDefinition& hgt)
{
	std::string ret;
	buf_ostream os_binary;
	std::list<text_ifstream> lst_xml;
	std::ifstream is_hgt(hgt, std::ios_base::in | std::ios_base::binary);
	if (is_hgt.fail())
		throw failed_to_open_a_file(hgt);
	is_hgt.seekg(0, std::ios_base::end);
	auto cb = std::size_t(std::streamoff(is_hgt.tellg()));
	is_hgt.seekg(0, std::ios_base::beg);
	switch (cb)
	{
	case HGT_3.cColumns * HGT_3.cRows * sizeof(std::int16_t):
	case HGT_1.cColumns * HGT_1.cRows * sizeof(std::int16_t):
		break;
	default:
		throw unexpected_hgt_size();
	}
	for (auto itXml = xml_is_begin; itXml != xml_is_end; ++itXml)
	{
		auto is = text_ifstream(std::string_view(*itXml));
		if (is.fail())
			throw failed_to_open_a_file(*itXml);
		lst_xml.emplace_back(std::move(is));
	}
	hgtxml2bin(domain_id, is_hgt, std::begin(lst_xml), std::end(lst_xml), os_binary);
	std::unique_ptr<char, constrol_system_free_data_t> pModel;
	CAMaaS::size_type cchModel;
	{
		char* l_pModel;
		CAMaaS::return_code_t nErr = ControlSystemEntryPoint(UploadModelId, os_binary.data(), CAMaaS::size_type(os_binary.size()), (void**) &l_pModel, &cchModel);
		if (ChsvFailed(nErr))
			throw coded_exception(nErr);
		pModel.reset(l_pModel);
	}
	return std::string(pModel.get(), cchModel);
}

#include <clocale>
#include <thread> //for portable thread suspension
#include <iostream>

#define DEFAULT_LOG "./execution.log"
const char pDefaultLogFile[] = DEFAULT_LOG;

struct ISimulationResultsStorage
{
	virtual ~ISimulationResultsStorage() = default;
	virtual void do_output(const std::string& porcess_id, const std::string& domain_system_id) = 0;
};

struct TextualizedResultsStorage:ISimulationResultsStorage
{
	virtual void do_output(const std::string& process_id, const std::string& domain_system_id)
	{
		OutputTextualizedResults(m_os, process_id, domain_system_id);
		m_os.flush();
	}
	TextualizedResultsStorage(const std::string& path, bool fDiscardExisting)
		:m_os(path, std::ios_base::in | std::ios_base::out | std::ios_base::binary | (fDiscardExisting?std::ios_base::trunc:std::ios_base::ate))
	{
		if (m_os.fail() || m_os.tellp() != std::ofstream::pos_type())
			throw failed_to_open_a_file(path);
	}
private:
	std::ofstream m_os;
};

struct BinaryResultsStorage:ISimulationResultsStorage
{
	virtual void do_output(const std::string& process_id, const std::string&)
	{
		OutputBinaryResults(m_os, process_id);
		m_os.flush();
	}
	BinaryResultsStorage(const std::string& path, bool fDiscardExisting)
		:m_os(path, std::ios_base::in | std::ios_base::out | std::ios_base::binary | (fDiscardExisting?std::ios_base::trunc:std::ios_base::ate))
	{
		if (m_os.fail() || m_os.tellp() != std::ofstream::pos_type())
			throw failed_to_open_a_file(path);
	}
private:
	std::ofstream m_os;
};

template <class OutputType>
auto make_results_storage(OutputType&& output)
	-> std::enable_if_t<std::is_base_of_v<ISimulationResultsStorage, std::decay_t<OutputType>>, std::unique_ptr<ISimulationResultsStorage>>
{
	return std::make_unique<OutputType>(std::move(output));
}

struct CompositeResultsStorage:ISimulationResultsStorage
{
	virtual void do_output(const std::string& process_id, const std::string& domain_system_id)
	{
		m_pThisOutput->do_output(process_id, domain_system_id);
		m_pRestOutput->do_output(process_id, domain_system_id);
	}
	template <class OutputType0, class ... OutputTypeRest, class = std::enable_if_t<
		std::is_base_of_v<ISimulationResultsStorage, std::decay_t<OutputType0>>
		&& std::conjunction_v<std::is_base_of<ISimulationResultsStorage, std::decay_t<OutputTypeRest>>...>
		>>
	CompositeResultsStorage(OutputType0&& output_0, OutputTypeRest&& ... output_n);
private:
	std::unique_ptr<ISimulationResultsStorage> m_pThisOutput;
	std::unique_ptr<ISimulationResultsStorage> m_pRestOutput;
};

template <class OutputType, class ... OutputTypeRest>
auto make_results_storage(OutputType&& output, OutputTypeRest&& ... output_n) -> std::enable_if_t<
	std::is_base_of_v<ISimulationResultsStorage, std::decay_t<OutputType>>
	&& std::conjunction_v<std::is_base_of<ISimulationResultsStorage, std::decay_t<OutputTypeRest>>...>,
	std::unique_ptr<ISimulationResultsStorage>>
{
	return std::make_unique<CompositeResultsStorage>(std::forward<OutputType>(output), std::forward<OutputTypeRest>(output_n) ...);
}

template <class OutputType0, class ... OutputTypeRest, class>
CompositeResultsStorage::CompositeResultsStorage(OutputType0&& output_0, OutputTypeRest&& ... output_n)
	:m_pThisOutput(make_results_storage(std::forward<OutputType0>(output_0))),
	m_pRestOutput(make_results_storage(std::forward<OutputTypeRest>(output_n)...))
{
}

struct ISimulationInput
{
	virtual ~ISimulationInput() = default;
	virtual bool is_ready() const = 0;
	virtual std::string upload_model(const std::string& domain_id) = 0;
};

struct BinarySimulationInput:ISimulationInput
{
	template <class DefinitionPath>
	BinarySimulationInput(DefinitionPath&& path):m_path(std::forward<DefinitionPath>(path)) {}
	virtual bool is_ready() const
	{
		return !m_path.empty();
	}
	virtual std::string upload_model(const std::string&)
	{
		return UploadBinaryModel(m_path);
	}
private:
	std::string m_path;
};

struct XmlSimulationInput:ISimulationInput
{
	template <class InputIteratorXmlBegin, class InputIteratorXmlEnd, class HgtPath>
	XmlSimulationInput(InputIteratorXmlBegin xml_paths_begin, InputIteratorXmlEnd xml_paths_end, HgtPath&& strHgt)
	{
		for (auto it = xml_paths_begin; it != xml_paths_end; ++it)
			m_lstXml.emplace_back(*it);
		m_strHgt = std::forward<HgtPath>(strHgt);
	}
	template <class InputIteratorXmlBegin, class InputIteratorXmlEnd>
	XmlSimulationInput(InputIteratorXmlBegin xml_paths_begin, InputIteratorXmlEnd xml_paths_end)
	{
		for (auto it = xml_paths_begin; it != xml_paths_end; ++it)
			m_lstXml.emplace_back(*it);
	}
	virtual bool is_ready() const
	{
		return !m_lstXml.empty();
	}
	virtual std::string upload_model(const std::string& domain_id)
	{
		if (m_strHgt.empty())
			return UploadXMLModel(domain_id, std::begin(m_lstXml), std::end(m_lstXml));
		else
			return UploadXMLModel(domain_id, std::begin(m_lstXml), std::end(m_lstXml), m_strHgt);
	}
private:
	std::list<std::string> m_lstXml;
	std::string m_strHgt;
};

struct IMode
{
	~IMode() = default;
	virtual void Run() = 0;
};
struct DefaultMode:IMode
{
	template <class DomainId, class LogFile>
	DefaultMode(DomainId&& domain_id, LogFile&& log, std::unique_ptr<ISimulationInput>&& input,
		std::unique_ptr<ISimulationResultsStorage>&& output):m_domain(std::forward<DomainId>(domain_id)), m_log(std::forward<LogFile>(log)),
		m_input(std::move(input)), m_output(std::move(output)) {}
	template <class DomainId>
	DefaultMode(DomainId&& domain_id, std::unique_ptr<ISimulationInput>&& input,
		std::unique_ptr<ISimulationResultsStorage>&& output)
		:DefaultMode(std::forward<DomainId>(domain_id), std::string(DEFAULT_LOG), std::move(input), std::move(output)) {}
	virtual void Run()
	{
		using CAMaaS::size_type;
		if (!m_log.empty())
			ControlSystemSetErrorLogFile(m_log.c_str(), CAMaaS::size_type(m_log.size()), true);
		std::cout << "Initializing simulation.\n";
		std::cout << "Creating geometry model.\n";
		auto model_id = m_input->upload_model(m_domain);
		std::cout << "Starting simulation.\n";
		auto tm_0 = std::chrono::system_clock::now();
		auto process_id = StartSimulation(m_domain, model_id);
		std::string strPending;
		try
		{
			unsigned tm = 0;
			std::cout << "Started.\n";
			while (IsProcessRunning(process_id)) //waiting until the simulation is finished
			{
				std::cout << std::string(strPending.size(), '\b');
				strPending = std::string("Pending (");
				strPending.append(std::to_string(tm++)).append(")...");
				std::cout << strPending;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			auto dur = std::chrono::system_clock::now() - tm_0;
			std::cout << "\nSimulation complete in " <<
				std::chrono::duration_cast<std::chrono::minutes>(dur).count() <<
				":" << std::chrono::duration_cast<std::chrono::seconds>(dur % std::chrono::minutes(1)).count() <<
				"." << std::chrono::duration_cast<std::chrono::milliseconds>(dur % std::chrono::seconds(1)).count() << " seconds.\n";
			m_output->do_output(process_id, m_domain);
		}catch(...)
		{
			CloseSimulation(process_id); //Closing started simulation if an error on a cilent side or on a server side occurs
			throw;
		}
		CloseSimulation(process_id); //Closing the started simulation on success
	}
private:
	std::string m_domain;
	std::string m_log;
	std::unique_ptr<ISimulationInput> m_input;
	std::unique_ptr<ISimulationResultsStorage> m_output;
};

struct Inp2BinMode:IMode
{
	template <class DomainId, class XmlPathIteratorBegin, class XmlPathIteratorEnd, class HgtInput, class Output>
	Inp2BinMode(DomainId&& domain_id, bool fDiscardOutput, XmlPathIteratorBegin xml_begin, XmlPathIteratorEnd xml_end, HgtInput&& hgt, Output&& output_file)
		:m_domain(std::forward<DomainId>(domain_id))
	{
		for (auto it_xml = xml_begin; it_xml != xml_end; ++it_xml)
		{
			auto is_xml = text_ifstream(std::string_view(*it_xml));
			if (is_xml.fail())
				throw failed_to_open_a_file(*it_xml);
			m_lst_xml.emplace_back(std::move(is_xml));
		}
		if (!hgt.empty())
		{
			m_is_hgt = std::ifstream(hgt, std::ios_base::in | std::ios_base::binary);
			if (m_is_hgt.fail())
				throw failed_to_open_a_file(hgt);
			auto cb = std::size_t(std::ifstream::off_type(m_is_hgt.seekg(std::ifstream::off_type(), std::ios_base::end).tellg()));
			m_is_hgt.seekg(std::ifstream::pos_type());
			switch (cb)
			{
			case HGT_3.cColumns * HGT_3.cRows * sizeof(std::int16_t):
			case HGT_1.cColumns * HGT_1.cRows * sizeof(std::int16_t):
				break;
			default:
				throw unexpected_hgt_size();
			}
			m_fHgt = true;
		}
		m_os = binary_ofstream(std::string_view(output_file), fDiscardOutput);
		if (m_os.fail())
			throw failed_to_open_a_file(output_file);
	}
	template <class DomainId, class XmlPathIteratorBegin, class XmlPathIteratorEnd, class Output>
	Inp2BinMode(DomainId&& domain_id, bool fDiscardOutput, XmlPathIteratorBegin xml_begin, XmlPathIteratorEnd xml_end, Output&& output_file)
		:Inp2BinMode(std::forward<DomainId>(domain_id), fDiscardOutput, xml_begin, xml_end, std::string(), std::forward<Output>(output_file)) {}
	virtual void Run()
	{
		if (m_fHgt)
			hgtxml2bin(m_domain, m_is_hgt, std::begin(m_lst_xml), std::end(m_lst_xml), m_os);
		else
			xml2bin(m_domain, std::begin(m_lst_xml), std::end(m_lst_xml), m_os);
	}
private:
	std::string m_domain;
	std::list<text_ifstream> m_lst_xml;
	bool m_fHgt = false;
	std::ifstream m_is_hgt;
	binary_ofstream m_os;
};

struct OutConvMode:IMode
{
	template <class DomainId, class InputFile, class OutputFile>
	OutConvMode(DomainId&& domain_id, bool fDiscardOutput, InputFile&& input, OutputFile&& output)
		:m_domain(std::forward<DomainId>(domain_id)), m_binary_input(input, std::ios_base::in | std::ios_base::binary),
		m_text_output(output, std::ios_base::out | (fDiscardOutput?std::ios_base::trunc:std::ios_base::ate))
	{
		if (m_binary_input.fail())
			throw failed_to_open_a_file(input);
		if (m_text_output.fail())
			throw failed_to_open_a_file(output);
	}
	virtual void Run()
	{
		bin2text(m_domain, m_binary_input, m_text_output);
	}
private:
	std::string m_domain;
	std::ifstream m_binary_input;
	std::ofstream m_text_output;
};

struct ShowHelp:IMode
{
	virtual void Run()
	{
		std::cout <<
			"cmd_client --domain domain_name [--hgt path_to_hgt_file] [--discard_output] [--log log_file]\n"\
			"               <--binary_input binary_model_definition|--xml_input input_xml_file_1 [... input_xml_file_n]>\n"\
			"               <[--binary_output output_binary_file] [--text_output output_textualized_file]>|\n"\
			"           --inp2bin --domain domain_name [--hgt path_to_hgt_file] [--discard_output]\n"\
			"               input_xml_file_1 [... input_xml_file_n] output_binary_file|\n"\
			"           --outconv --domain domain_name [--discard_output] --bin input_binary_file --txt output_text_file|\n"\
			"           --help\n"\
			"DESCRIPTION\n"\
			"    Starts simulation for a specified input producing the specified output OR only performs conversion of\n"\
			"input or output data various between various formats. The program operates in one of three modes.\n"\
			"    In the default mode the program preprocesses an input given in thespecified format, then perfoms the\n"\
			"simulation and postprocess the output writing it to the specified file, also in the specified format.\n"\
			"    In the inp2bin mode the program only converts the specified input given in one format to the binary\n"\
			"format, which is raw data fed to the control system.\n"\
			"    In the outconv mode the program only converts the specified binary output to a textual form.\n"\
			"PARAMETERS\n"\
			" --domain specifies a domain system id for which the program should perform its action. In the default and\n"\
			"    inp2bin modes of operation the program discards any domain data for domain systems with ids different\n"\
			"    from the one specified by the --domain parameter.\n"\
			" --hgt specifies a path to a HGT file to be converted to a set of polygonal surfaces to specify, together\n"\
			"    with input XML files, the output binary model. Only SRTM 30m and SRTM 90m are supported. The parameter is\n"\
			"    optional. The parameter is not allowed in the default mode together with the --binary_input parameter.\n"\
			" --discard_output is a switch which allows to truncate the output file before writing to it, should the file\n"\
			"    exist. Otherwise, if the output file exists and not empty, the program will fail.\n"\
			" --log in default mode specifies a log file for the simulation. Defaults to \"" DEFAULT_LOG "\"\n"\
			" --binary_input, in the default mode, specifies a binary definition of the input model to be fed to the\n"\
			"    control subsystem as-is. That is, the file specified by this parameter must contain raw data defined as\n"\
			"    specified for the UploadModelId request in the wiki. The parameter must not be specified together with\n"\
			"    the --xml_input and/or --hgt parameters\n"\
			" --xml_input, in the default mode, specifies a set of XML-definitions of the input model. The set is\n"\
			"    specified by a sequence of following files paths. All the definitions are unified to a one data block\n"
			"    defining the input model. As such the complete input model may be separated to multiple XML definitions\n"\
			"    on the per-object basis. Only one definition of a model and one definition of a named object with a given\n"\
			"    name may be specified across all the files. The exception is the unnamed objects. The parameter can also\n"\
			"    be specified together with a HGT file given by the --hgt parameter, in which case the set of polygons\n"\
			"    obtained by parsing the HGT will be unified with the XML files to produce the complete model definition.\n"\
			" --binary_output, in the default mode, specifies an output file to accept raw binary output data produced by\n"\
			"    a single simulation.\n"\
			" --text_output, in the default mode, specifies an output file to accept textualized output data produced by a\n"\
			"    single simulation.\n"\
			" --inp2bin switches the program to operate as a converter from an input, given in a specified format, to the\n"\
			"    raw binary format accepted as an input parameter of the UploadModelId request to the control system.\n"\
			" --outconv switches the program to operate as a converter from an output, given in raw binary format, to the\n"\
			"    specified format.\n"\
			" --bin, in the outconv mode, specifies the input file with binary raw data with simulation results to convert to\n"\
			"    text.\n"\
			" --txt, in the outconv mode, specifies the text file to accept the textualized output of simulation results.\n"\
			" --help displays this message.\n";
	}
};

class Program
{
public:

	Program(int argc, char** argv)
	{
		int i;
		if (argc < 2)
			throw invalid_usage();
		if (std::string_view(argv[1]) == "--help")
		{
			if (argc > 2)
				throw invalid_usage();
			m_pMode = std::make_unique<ShowHelp>();
		}else if (std::string_view(argv[1]) == "--inp2bin")
		{
			std::string domain_id;
			std::string hgt;
			bool fDiscardOutput = false;
			std::list<std::string> lst_xml;
			std::string output;
			for (i = 2; i < argc; ++i)
			{
				if (std::string_view(argv[i]) == "--hgt")
				{
					if (!hgt.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					hgt = argv[i];
				}else if (std::string_view(argv[i]) == "--domain")
				{
					if (!domain_id.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					domain_id = argv[i];
				}else if (std::string_view(argv[i]) == "--discard_output")
				{
					if (fDiscardOutput)
						throw invalid_usage();
					fDiscardOutput = true;
				}else if (argv[i][0] == '-' && argv[i][1] == '-')
				{
					std::ostringstream os;
					os << "Unknown parameter " << argv[i] << " of the inp2bin mode.";
					throw invalid_usage(os.str());
				}else
					break;
			}
			for (; i < argc; ++i)
			{
				if (argv[i][0] == '-' && argv[i][1] == '-')
				{
					std::ostringstream os;
					os << "Unknown parameter " << argv[i] << " of the inp2bin mode.";
					throw invalid_usage(os.str());
				}
				if (!output.empty())
					lst_xml.emplace_back(std::move(output));
				output = std::string(argv[i]);
			}
			if (lst_xml.empty())
				throw invalid_usage("No input XML files are specified for the inp2bin mode.");
			if (output.empty())
				throw invalid_usage("No output file is specified for the inp2bin mode.");
			if (domain_id.empty())
				throw invalid_usage("No domain system identifier is specified for the inp2bin mode.");
			if (hgt.empty())
				m_pMode = std::make_unique<Inp2BinMode>(std::move(domain_id), fDiscardOutput, std::begin(lst_xml), std::end(lst_xml), std::move(output));
			else
				m_pMode = std::make_unique<Inp2BinMode>(std::move(domain_id), fDiscardOutput, std::begin(lst_xml), std::end(lst_xml), std::move(hgt), std::move(output));
		}else if (std::string_view(argv[1]) == "--outconv")
		{
			std::string domain_id;
			std::string input;
			std::string output;
			bool fDiscardOutput = false;

			for (i = 2; i < argc; ++i)
			{
				if (std::string_view(argv[i]) == "--domain")
				{
					if (!domain_id.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					domain_id = argv[i];
				}else if (std::string_view(argv[i]) == "--discard_output")
				{
					if (fDiscardOutput)
						throw invalid_usage();
					fDiscardOutput = true;
				}else if (std::string_view(argv[i]) == "--bin")
				{
					if (!input.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					input = argv[i];
				}else if (std::string_view(argv[i]) == "--txt")
				{
					if (!output.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					output = argv[i];
				}else if (argv[i][0] == '-' && argv[i][1] == '-')
				{
					std::ostringstream os;
					os << "Unknown parameter " << argv[i] << " of the inp2bin mode.";
					throw invalid_usage(os.str());
				}else
					throw invalid_usage();
			}
			if (input.empty())
				throw invalid_usage("No input XML files are specified for the inp2bin mode.");
			if (output.empty())
				throw invalid_usage("No output file is specified for the inp2bin mode.");
			if (domain_id.empty())
				throw invalid_usage("No domain system identifier is specified for the inp2bin mode.");
			m_pMode = std::make_unique<OutConvMode>(domain_id, fDiscardOutput, std::move(input), std::move(output));
		}else
		{
			std::string domain_id;
			std::string hgt;
			bool fDiscardOutput = false;
			std::string log;
			std::list<std::string> lst_xml_input;
			std::string binary_input;
			std::string binary_output;
			std::string text_output;
			for (i = 1; i < argc; ++i)
			{
				if (std::string_view(argv[i]) == "--hgt")
				{
					if (!hgt.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					hgt = argv[i];
				}else if (std::string_view(argv[i]) == "--domain")
				{
					if (!domain_id.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					domain_id = argv[i];
				}else if (std::string_view(argv[i]) == "--discard_output")
				{
					if (fDiscardOutput)
						throw invalid_usage();
					fDiscardOutput = true;
				}else if (std::string_view(argv[i]) == "--log")
				{
					if (!log.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					log = argv[i];
				}else if (std::string_view(argv[i]) == "--binary_input")
				{
					if (!binary_input.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					binary_input = argv[i];
				}else if (std::string_view(argv[i]) == "--xml_input")
				{
					if (++i == argc || !lst_xml_input.empty())
						throw invalid_usage();
					for (; i < argc; ++i)
					{
						if (argv[i][0] == '-' && argv[i][1] == '-')
						{
							--i;
							break;
						}
						lst_xml_input.emplace_back(argv[i]);
					}
					if (lst_xml_input.empty())
						throw invalid_usage();
				}else if (std::string_view(argv[i]) == "--binary_output")
				{
					if (!binary_output.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					binary_output = argv[i];
				}else if (std::string_view(argv[i]) == "--text_output")
				{
					if (!text_output.empty() || ++i == argc || (argv[i][0] == '-' && argv[i][1] == '-'))
						throw invalid_usage();
					text_output = argv[i];
				}else if (argv[i][0] == '-' && argv[i][1] == '-')
				{
					std::ostringstream os;
					os << "Unknown parameter " << argv[i] << " of the default mode.";
					throw invalid_usage(os.str());
				}else
					throw invalid_usage();
			}
			if (domain_id.empty())
				throw invalid_usage("Domain system identifier is not specified.");
			std::unique_ptr<ISimulationInput> pInput;
			if (!binary_input.empty())
			{
				if (!lst_xml_input.empty() || !hgt.empty())
					throw invalid_usage("Binary input cannot be specified together with XML ot HGT input.");
				pInput = std::make_unique<BinarySimulationInput>(std::move(binary_input));
			}else if (lst_xml_input.empty())
				throw invalid_usage("Neither XML nor binary input is specified");
			else
			{
				if (hgt.empty())
					pInput = std::make_unique<XmlSimulationInput>(std::begin(lst_xml_input), std::end(lst_xml_input));
				else
					pInput = std::make_unique<XmlSimulationInput>(std::begin(lst_xml_input), std::end(lst_xml_input), std::move(hgt));
			}
			std::unique_ptr<ISimulationResultsStorage> pOutput;
			if (!binary_output.empty())
			{
				if (text_output.empty())
					pOutput = make_results_storage(BinaryResultsStorage(binary_output, fDiscardOutput));
				else
					pOutput = make_results_storage(BinaryResultsStorage(binary_output, fDiscardOutput), TextualizedResultsStorage(text_output, fDiscardOutput));
			}else if (!text_output.empty())
				pOutput = make_results_storage(TextualizedResultsStorage(text_output, fDiscardOutput));
			else
				throw invalid_usage("No output is specified");
			if (log.empty())
				m_pMode = std::make_unique<DefaultMode>(std::move(domain_id), std::move(pInput), std::move(pOutput));
			else
				m_pMode = std::make_unique<DefaultMode>(std::move(domain_id), log, std::move(pInput), std::move(pOutput));
		}
	}
	Program& run()
	{
		m_pMode->Run();
		return *this;
	}
private:
	std::unique_ptr<IMode> m_pMode;
};

int main(int argc, char** argv)
{
	try
	{
		Program(argc, argv).run();
	}catch (std::exception& ex)
	{
		std::cerr << ex.what() << "\n";
		return -1;
	}
	return 0;
}
