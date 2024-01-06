#include <string>
#include <iostream>
#include <sstream>

class DebugLogger {
	public:
	DebugLogger() {
		log = "Debug Mode: ON\nInformation:";
	}
	
	DebugLogger* error(std::string line) {
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Error | " << line << "";
		log = ss.str();
		return this;
	}

	DebugLogger* info(std::string line) {
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Info | " <<line << "";
		log = ss.str();
		return this;
	}

	DebugLogger* warning(std::string line) {
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Warning | " <<line << "";
		log = ss.str();
		return this;
	}

	DebugLogger* append(std::string text) {
		log += text;
		return this;
	}
	
	
	DebugLogger* hex(int x) {
		sprintf(tmp, "0x%x", x);
		log += std::string(tmp);
		return this;
	}
	
	DebugLogger* pi(int o){
		log += std::to_string(o);
		return this;
	}
	
	DebugLogger* pf(float o){
		log += std::to_string(o);
		return this;
	}
	
	const char* toString(bool critical_error){
		std::string printer;
		if(critical_error){
			std::stringstream ss;
			ss << log << std::endl;
			ss << "Warning | Critical errors founded (See the above errors, and fix)"<<std::endl;
			ss << "EngineError | plStepSimulation is blocked!";
			printer = ss.str();
		}else{
			std::stringstream ss;
			ss << log << std::endl;
			ss << "Info | No has critical errors";
			printer = ss.str();
		}
		return printer.c_str();
	}
	
	private:
	std::string log;
	char tmp[10];
};
