
#pragma once

struct vec3 {
	float x;
	float y;
	float z;
	
	vec3() :
		x(0),
		y(0),
		z(0)
	{}
		
	vec3& operator=(const vec3& o){
		x = o.x;
		y = o.y;
		z = o.z;
		return *this;
	}
};

static inline bool operator==(const vec3& a, const vec3& b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

#include <iostream>

static std::ostream& operator<<(std::ostream& os, const vec3& v) {
	os << "vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
	return os;
}
