
#pragma once

#include <iostream>
#include <cmath>

template <class T>
class Mysqrt
{
private:
    mutable T m_x{};

public:
    Mysqrt() = default;

    explicit Mysqrt(const T &x) : m_x{x}
    {
        std::cout << "contruction is recalled!" << std::endl;
    }

    Mysqrt(const Mysqrt &&rhs) : m_x{rhs.m_x}
    {
        std::cout << "move construction is recalled!" << std::endl;
    }

    Mysqrt &operator=(const Mysqrt &&rhs)
    {
        std::cout << "move assignment is recalled!" << std::endl;
        m_x = rhs.m_x;
        return *this;
    }

    ~Mysqrt()
    {
        std::cout << "destrution is recalled!" << std::endl;
    }

private:
    Mysqrt(const Mysqrt &) = delete;
    Mysqrt &operator=(const Mysqrt &) = delete;

    T implement(const T &x) const
    {
        if (x == 0)
        {
            return 0;
        }

        T x0 = x;
        while (1)
        {
            T xi = 0.5 * (x0 + x / x0);
            if (fabs(x0 - xi) < 1e-7)
            {
                break;
            }
            x0 = xi;
        }
        return x0;
    }

public:
    T getVal()
    {
        T tmp = implement(m_x);
        return tmp;
    }
};
