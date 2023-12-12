
#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <string>
#include <vector>
#include <sstream>
#include <stdint.h>
using namespace std;

class Settings {
public:
	Settings(){}
	Settings(const string& fn) {
		read(fn);
	}
	~Settings() {
		flush();
	}
	
	void read(const string& fn);
	void flush();
	

	template<typename T>
	void get(const string& key, T& val) const {
		int i = get_idx(key);
		val = vals[i];
	}
	
	template<typename T>
	void get_default(const string& key, T& val, const T& def_val) {
		int i = tryget_idx(key);
		if(i == -1){
			set(key, def_val);
			flush();
		}
		get(key, val);
	}

	template<typename T>
	void set(const string& key, const T& val) {
		ostringstream oss;
		oss << val;
		int i = tryget_idx(key);
		if(i == -1){
			push_back(key, oss.str());
		}else{
			vals[i] = oss.str();
		}
	}

	void erase(const string& key) {
		//TODO implement.
	}
	
protected:
	string file_name;
	vector<string> keys;
	vector<string> vals;
	vector<string> comments;
	vector<pair<char, int>> order;
	
	/**
	 * @return -1 if didn't found.
	 */
	int tryget_idx(const string& key) const noexcept;
	int get_idx(const string& key) const;
	void push_back(const string& key, const string& val);
	
};


template<>
void Settings::get(const string& key, int32_t& val) const;



#define GET_SETTING(s, var) \
	do{ \
		s.get(#var, var); \
	}while(0)
#define GET_DEFAULT_SETTING(s, var, def) \
	do{ \
		s.get_default(#var, var, def); \
	}while(0)
#define DEFINE_GET_DEFAULT_SETTING(s, type, var, def) \
	type var; s.get_default(#var, var, def);
#define SET_SETTING(s, var) \
	do{ \
		s.set(#var, var); \
	}while(0)
#define SET_AND_FLUSH_SETTING(s, var) \
	do{ \
		s.set(#var, var); \
		s.flush(); \
	}while(0)

#endif // SETTINGS_HPP
