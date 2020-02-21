#ifndef POINT_H
#define POINT_H

#include <cstdint>
#include <selfie_fuzzy_logic/range.h>

class Point{
public:
    Point(int32_t x, int32_t y);
    int32_t getX();
    int32_t getY();

    void setX(int32_t min);
    void setY(int32_t max);

    void setRangeX(Range x_range);
    void setRangeY(Range y_range);
private:
    int32_t x, y;
    Range x_range, y_range;
};

#endif // POINT_H
