#include "cvxgen/solver.h"
#include <stdio.h>
#include <string>
#include <vector>

#define CONTROL_DIM 2
#define STATE_DIM 3
#define HORIZON 5

Vars vars;
Params params;
Workspace work;
Settings settings;

class solv_in{
private:
    std::vector<std::vector<bool>> set_vals;
    
public:
    solv_in(/* args */);
    ~solv_in();
    void set_parameters(int id, int index, float val);
    void visualize_parameters(void);
    void visualize_variables(void);
};
