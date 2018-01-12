#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

int add(int a, int b) {
	return a + b;
}

int main() {
	std::map<std::string, std::function<int(int)>> adderMap;
	
	using namespace std::placeholders;
	adderMap["five"] = std::bind(add, 5, _1);
	adderMap["two_hundred"] = std::bind(add, 200, _1);
	adderMap["seventy_five"] = std::bind(add, 75, _1);
	
	std::cout << adderMap["five"](100) << std::endl;
}
