#ifndef __PROPERTIES_H_
#define __PROPERTIES_H_

#include <map>
#include <string>

namespace Util{

class Properties{
	private:
		std::map<std::string, std::string> propertyList;

		
	public:
		Properties() {}
		virtual ~Properties() {}
		
		void load(std::string filename);
        std::string trim(std::string& input);
		std::string getProperty(std::string property);
        void setProperty(std::string key, std::string value);
		void show();
};
}
#endif

