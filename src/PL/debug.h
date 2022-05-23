#include <string>
#include <iostream>
#include <sstream>

class DebugLogger{
	public:
	DebugLogger(){
		log = "Debug Mode: ON\nInformation:";
	}
	
	DebugLogger* e(std::string line){
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Error | " << line << "";
		log = ss.str();
		return this;
	}
	
	DebugLogger* i(std::string line){
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Info | " <<line << "";
		log = ss.str();
		return this;
	}
	
	DebugLogger* w(std::string line){
		std::stringstream ss;
		ss << log << std::endl;
		ss << "Warning | " <<line << "";
		log = ss.str();
		return this;
	}
	
	DebugLogger* a(std::string text){
		std::stringstream ss;
		ss << log << text;
		log = ss.str();
		return this;
	}
	
	
	DebugLogger* hex(int x){
		std::stringstream ss;
		ss << log << x;
		log = ss.str();
		return this;
	}
	
	DebugLogger* pi(int o){
		std::stringstream ss;
		ss << log << o;
		log = ss.str();
		return this;
	}
	
	DebugLogger* pf(float o){
		std::stringstream ss;
		ss << log << o;
		log = ss.str();
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
};
