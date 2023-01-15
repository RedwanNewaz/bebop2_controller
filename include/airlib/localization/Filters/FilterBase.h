//
// Created by redwan on 1/14/23.
//

#ifndef BEBOP2_CONTROLLER_FILTERBASE_H
#define BEBOP2_CONTROLLER_FILTERBASE_H
#include <iostream>
#include <memory>
#include <vector>

class FilterBase;

typedef std::shared_ptr<FilterBase> FilterPtr;

class FilterBase: std::enable_shared_from_this<FilterBase>
{
public:
    FilterBase()
    {

    }
    virtual ~FilterBase()
    {

    }
    FilterPtr getPtr()
    {
        return shared_from_this();
    }
    virtual void init(const std::vector<double>& X0) = 0;
    virtual void update(const std::vector<double>& obs, std::vector<double>& result) = 0;
protected:
    std::vector<double> X_;

};

#endif //BEBOP2_CONTROLLER_FILTERBASE_H
