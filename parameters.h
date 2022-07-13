#include <Eigen\Dense>

Eigen::Vector3d G{0.0, 0.0, 0.0};
Eigen::Vector3d W_IL{8.916855192971346e-05,-1.136750250097826e-04,-8.425062842436180e-06};


enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

double SOLVER_TIME = 1.0;
int NUM_ITERATIONS = 50;