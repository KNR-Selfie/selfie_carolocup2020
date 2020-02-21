#include <selfie_fuzzy_logic/rule.h>

Rule::Rule(){

}

void Rule::addInput(int32_t &value, Membership &membership){
    inputs_list.push_back(Input(value, membership));
}

void Rule::addOutput(int32_t &value, Membership &membership){
    //outputs_list.push_back(Output(value, membership));
    output = new Output(value, membership);
}

void Rule::addOutput(Membership &membership){
    int32_t value = 0;
    output = new Output(value, membership);
}

int32_t Rule::calculate_input(){
    int32_t min = INT32_MAX;
    for (std::list<Input>::iterator it=inputs_list.begin(); it != inputs_list.end(); ++it){
        min = std::min(min, it->membership->getValue(*it->value));
    }
    return min;
}

int32_t Rule::getOutValue(){
    return output->membership->getValue(*output->value);
}

Membership *Rule::getOutMembership(){
return output->membership;
}

Rule::Input::Input(int32_t &value, Membership &membership){
    this->value = &value;
    this->membership = &membership;
}

Rule::Output::Output(int32_t &value, Membership &membership){
    this->value = &value;
    this->membership = &membership;
}
