#ifndef FUZZYCONTROLER_H
#define FUZZYCONTROLER_H

#include <selfie_fuzzy_logic/rule.h>

class FuzzyController{
public:
    FuzzyController();
    void addRule(Rule rule);
    float getOut();
private:
    struct RuleOutput {
        Membership* out_membership;
        int32_t out_value;
        int32_t input_membership_value;
        RuleOutput(int32_t input_membership_value, int32_t out_value, Membership &out_membership);
    };
    std::list<Rule> rules_list;
    uint8_t isObjectUnique(std::list<RuleOutput> &unique_list, Rule object);

};

#endif // FUZZYCONTROLER_H
