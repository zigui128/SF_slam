#include <gtest/gtest.h>
#include <climits>

// bad function:
// for example: how to deal with overflow?
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


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

