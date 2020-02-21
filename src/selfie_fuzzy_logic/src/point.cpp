#include <selfie_fuzzy_logic/point.h>

Point::Point(int32_t x, int32_t y){
    this->x = x;
    this->y = y;
}

int32_t Point::getX(){
    return x;
}

int32_t Point::getY(){
    return y;
}

void Point::setX(int32_t x){
    if(x_range.isInRange(x))
        this->x = x;
}

void Point::setY(int32_t y){
    if(y_range.isInRange(y))
        this->y = y;
}

void Point::setRangeX(Range x_range){
    this->x_range = x_range;

    x = x_range.getValueInRange(x);
}

void Point::setRangeY(Range y_range){
    this->y_range = y_range;

    y = y_range.getValueInRange(y);
}


