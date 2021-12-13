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

  // 寻优参数x的初始值，为5
  double initial_x = 5.0;
  double x = initial_x;

  // 第二部分：构建寻优问题
  Problem problem;
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor); //使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即待寻优参数x的维度。
  problem.AddResidualBlock(cost_function, NULL, &x); //向问题中添加误差项，本问题比较简单，添加一个就行。

  //第三部分： 配置并运行求解器
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
  options.minimizer_progress_to_stdout = true;//输出到cout
  Solver::Summary summary;//优化信息
  Solve(options, &problem, &summary);//求解!!!

  std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
  //最终结果
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

