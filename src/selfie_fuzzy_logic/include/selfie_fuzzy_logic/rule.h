#ifndef RULE_H
#define RULE_H

#include <selfie_fuzzy_logic/membership.h>

class Rule
{
public:
    Rule();
    void addInput(int32_t &value, Membership &membership);
    void addOutput(int32_t &value, Membership &membership);
    void addOutput(Membership &membership);

    int32_t calculate_input();
    int32_t getOutValue();
    Membership *getOutMembership();
private:
    struct Input {
        Membership* membership;
        int32_t* value;
        Input(int32_t &value, Membership &membership);
    };
    struct Output {
        Membership* membership;
        int32_t* value;
        Output(int32_t &value, Membership &membership);
    };


    std::list<Input> inputs_list;
    //std::list<Output> outputs_list;
    Output *output;
};

#endif // RULE_H
