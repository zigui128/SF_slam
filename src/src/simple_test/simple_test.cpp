#include <gtest/gtest.h>
#include <climits>
#include <iostream>
#include <ceres/ceres.h>

using namespace ceres;
using namespace std;

int add(int a, int b){
    return a + b;
}

TEST(NumberCmpTest, ShouldPass){
   int cnt = 0;
   while(cnt < 10000000){
      cnt++;
    ASSERT_EQ(3, add(1,2));

   }
}

struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};


TEST(CeresTest, simpletest){


  double initial_x = 5.0;
  double x = initial_x;


  Problem problem;
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, NULL, &x);

  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";

  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

