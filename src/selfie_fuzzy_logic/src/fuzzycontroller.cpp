#include <selfie_fuzzy_logic/fuzzycontroller.h>

#include <iostream>

FuzzyController::FuzzyController(){

}

void FuzzyController::addRule(Rule rule){
    rules_list.push_back(rule);
}

float FuzzyController::getOut(){
    float out = 0.0;
    std::list<RuleOutput> unique_rules_outputs_list;
    for (std::list<Rule>::iterator it=rules_list.begin(); it != rules_list.end(); ++it){
        if(isObjectUnique(unique_rules_outputs_list, *it)){
            unique_rules_outputs_list.push_back(RuleOutput(it->calculate_input(), it->getOutValue(), *it->getOutMembership()));
        }
    }

    int32_t weights_sum = 0;
    for (std::list<RuleOutput>::iterator it=unique_rules_outputs_list.begin(); it != unique_rules_outputs_list.end(); ++it){
        weights_sum += it->input_membership_value;
        out += it->input_membership_value * it->out_value;
    }
    out /= weights_sum;
    return out;
}

uint8_t FuzzyController::isObjectUnique(std::list<RuleOutput> &unique_list, Rule object){
    for (std::list<RuleOutput>::iterator it=unique_list.begin(); it != unique_list.end(); ++it){
        if(it->out_membership == object.getOutMembership()){
            it->input_membership_value = std::max(it->input_membership_value, object.calculate_input()); //updateing max input;
            return false;
        }
    }
    return true;
}

FuzzyController::RuleOutput::RuleOutput(int32_t input_membership_value, int32_t out_value, Membership &out_membership){
    this->input_membership_value = input_membership_value;
    this->out_value = out_value;
    this->out_membership = &out_membership;
}
