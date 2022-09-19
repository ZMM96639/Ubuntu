
#include "Mysqrt.hpp"

int main(int argc, char *argv[])
{
    Mysqrt<double> test1{2.0};
    double result1 = test1.getVal();
    std::cout << result1 << std::endl;

    Mysqrt<double> test2{std::move(test1)};
    double result2 = test2.getVal();
    std::cout << result2 << std::endl;

    Mysqrt<double> test3;
    test3 = std::move(test1);
    double result3= test3.getVal();
    std::cout << result3 << std::endl;

    return 0;
}