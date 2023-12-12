
#ifndef PRINT_UTILS_HPP
#define PRINT_UTILS_HPP

#include <iostream>

using namespace std;


#define PRINT(x) #x << " = " << x
#define POSITION __PRETTY_FUNCTION__ << " @ " << __LINE__ << ": "
#define DEBUG(x) do{ cout << PRINT(x) << endl; }while(0)
#define TRACE() do{ cout << POSITION << endl; }while(0)


#endif // PRINT_UTILS_HPP
