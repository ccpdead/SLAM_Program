
#ifndef _QUADRATIC_CALCULATOR_H
#define _QUADRATIC_CALCULATOR_H
#include<vector>
#include<global_planner/potential_calculator.h>

namespace global_planner {

class QuadraticCalculator : public PotentialCalculator {
    public:
        QuadraticCalculator(int nx, int ny): PotentialCalculator(nx,ny) {}

        float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential);
};


} //end namespace global_planner
#endif
