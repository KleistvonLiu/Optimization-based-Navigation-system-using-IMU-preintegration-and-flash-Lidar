#include <Eigen\Dense>

Eigen::Vector3d G{0.0, 0.0, 9.8};
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

double SOLVER_TIME = 0.05;
int NUM_ITERATIONS = 10;