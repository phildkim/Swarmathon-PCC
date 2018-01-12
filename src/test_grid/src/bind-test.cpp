#include <functional>
#include <iostream>
#include <map>
#include <vector>

int add(int a, int b) {
	return a + b;
}



int main() {
	std::vector<std::function<int(int)>> adderArr;
	
	using namespace std::placeholders;
	adderArr.push_back(std::bind(add, 5, _1));
	adderArr.push_back(std::bind(add, 200, _1));
	adderArr.push_back(std::bind(add, 75, _1));
	
	for(std::function<int(int)> adder : adderArr) {
		std::cout << adder(100) << std::endl;
	}
}
