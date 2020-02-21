#include <selfie_fuzzy_logic/membership.h>
#include <algorithm>
#include <iostream>
#include <iterator>


Membership::Membership(Range x_range, Range y_range){
    this->range = x_range;
    value_range = y_range;
}

//void Membership::addPoint(Point point){
//    addPointToList(point);
//}

void Membership::addPoint(Point &point){
    addPointToList(point);
}

int32_t Membership::getValue(int32_t x){
    for (std::list<Point *>::iterator it=points_list.begin(); it != points_list.end(); ++it){
        if((*it)->getX() == x){
            return (*it)->getY();
        } else if((*it)->getX() > x){
            if(it == points_list.begin()){
                return (*points_list.front()).getY();
            } else {
                std::list<Point *>::iterator prev_it = it;
                std::advance(prev_it, -1);
                return Range((*prev_it)->getY(), (*it)->getY()).getValueForPart(Range((*prev_it)->getX(), (*it)->getX()).getPartOfRange(x));
            }
        }
    }
    return (*points_list.back()).getY();
}

void Membership::print(){
    for (std::list<Point *>::iterator it=points_list.begin(); it != points_list.end(); ++it){
        std::cout << ' ' << (int)(*it)->getX() << ' ' << (int)(*it)->getY() << std::endl;
    }
}

std::list<Point *> Membership::getPointList(){
    return points_list;
}

void Membership::addPointToList(Point &point){
    point.setRangeX(range);
    point.setRangeY(value_range);

    for (std::list<Point *>::iterator it=points_list.begin(); it != points_list.end(); ++it){
        if((*it)->getX() == point.getX()){
            return;
        } else if((*it)->getX() > point.getX()){
            points_list.insert(it, &point);
            return;
        }
    }
    points_list.push_back(&point);
}
