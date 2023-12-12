
#ifndef UART_HPP
#define UART_HPP

#include <string>
#include <vector>
#include <stdint.h>
using namespace std;

class UART {
public:
	UART(const string& dev_fn, int baud_rate);
	~UART();
	void write(const vector<uint8_t>& d);
	/**
	 * Read as much as can.
	 */
	vector<uint8_t> read();
	vector<uint8_t> read(size_t b_to_read);
private:
	int uart_fd;
};

#endif // UART_HPP
