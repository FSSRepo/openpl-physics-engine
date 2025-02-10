#include <string>
#include <iostream>
#include <sstream>

class DebugLogger {
	public:
	DebugLogger() {
		log = "Debug Mode: ON\nInformation:";
	}
	
	DebugLogger* error(std::string text) {
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Error | " << text << "";
		log = ss.str();
		return this;
	}

	DebugLogger* info(std::string text) {
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Info | " << text << "";
		log = ss.str();
		return this;
	}

	DebugLogger* warning(std::string text) {
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Warning | " << text << "";
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
	
	DebugLogger* pi(int o) {
		log += std::to_string(o);
		return this;
	}
	
	DebugLogger* pf(float o) {
		log += std::to_string(o);
		return this;
	}
	
	const char* toString(bool critical_error){
		std::string printer = log + "\n";
		if(critical_error) {
			printer += "Warning | Critical errors founded (See the above errors, and fix)\n";
			printer += "EngineError | plStepSimulation is blocked!";
		} else {
			printer += "Info | No has critical errors";
		}
		return printer.c_str();
	}
	
	private:
	std::string log;
	char tmp[10];
};
