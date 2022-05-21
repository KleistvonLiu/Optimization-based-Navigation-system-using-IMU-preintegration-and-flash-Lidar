#include<iostream>

class MyArray
{
public:
	double* array_;
	MyArray(double array[5]) :array_(array) {};//给出默认参数
	MyArray();
	void PrintMyArray() { std::cout << array_[3]; }
};

int main() {
	double test[5] = { 1,2,3,4,5 };
	MyArray a(test);
	std::cout << a.array_[0];
	a.PrintMyArray();
	return 0;
}