#ifndef MEMBERSHIP_H
#define MEMBERSHIP_H

#include <selfie_fuzzy_logic/range.h>
#include <selfie_fuzzy_logic/point.h>
#include <list>

class Membership{
public:
    Membership(Range x_range = Range(), Range y_range = Range(0, 1000));

 //   void addPoint(Point point);
    void addPoint(Point &point);
    int32_t getValue(int32_t x);
    void print();

    std::list<Point *> getPointList();
private:
    void addPointToList(Point &point);

    Range range;
    Range value_range;

    std::list<Point *> points_list;
};

#endif // MEMBERSHIP_H
