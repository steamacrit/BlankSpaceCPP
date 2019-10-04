#pragma once

template <typename T>
class DriveValues
{
public:
    DriveValues(T t = 0) : lf(t), la(t), rf(t), ra(t) 
    { 
        values.push_back(lf);
        values.push_back(la);
        values.push_back(rf);
        values.push_back(ra);
    }

    T lf;
    T la;
    T rf;
    T ra;

    std::vector<std::reference_wrapper<T>> values;
};