
#include "Settings.hpp"

#include <fstream>
#include <stdexcept>
#include <algorithm>

#include <iostream>

void Settings::read(const string& fn) {
	file_name = fn;
	ifstream ifs(fn);
	string line;
	while(getline(ifs, line)){
		if(line.size() == 0){
			order.emplace_back('\n', -1);
		}else if(line[0] == '#'){
			order.emplace_back('#', comments.size());
			comments.emplace_back(line);
		}else{
			istringstream iss(line);
			string key, sep, val;
			iss >> key >> sep >> val;
			if(sep != "="){
				order.emplace_back('#', comments.size());
				comments.emplace_back(line);
				//TODO Faulty line warning.
			}else{
				push_back(key, val);
			}
		}
	}
}

void Settings::flush() {
	ofstream ofs(file_name);
	for(int i = 0; i != order.size(); i++){
		auto p = order[i];
		switch(p.first){
			case '\n':
				ofs << endl;
				break;
			case '#':
				ofs << comments[p.second] << endl;
				break;
			case '=':
				ofs << keys[p.second] << " = " 
					<< vals[p.second] << endl;
				break;
			default:
				//TODO assert.
				break;
		}
	}
	ofs.close();
}

int Settings::tryget_idx(const string& key) const noexcept {
	auto it = find(keys.begin(), keys.end(), key);
	if(it == keys.end()){
		return -1;
	}else{
		return it - keys.begin();
	}
}
	
int Settings::get_idx(const string& key) const {
	int i = tryget_idx(key);
	if(i < 0){
		//TODO Message.
		throw out_of_range("TODO");
	}
	return i;
}

void Settings::push_back(const string& key, const string& val) {
	order.emplace_back('=', keys.size());
	keys.emplace_back(key);
	vals.emplace_back(val);
}

template<>
void Settings::get(const string& key, int32_t& val) const {
	int i = get_idx(key);
	istringstream iss(vals[i]);
	//TODO Int32()
	iss >> val;
}
