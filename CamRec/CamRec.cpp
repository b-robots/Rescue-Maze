// CamRec.cpp : Defines the entry point for the application.
//K

#include "CamRec.h"

#include <iostream>

using namespace std;
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

int main()
{
	cout <<  exec("ifconfig") << endl;
	getchar();
	return 0;
}
