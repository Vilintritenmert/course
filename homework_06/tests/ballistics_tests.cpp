#include "ballistics.hpp"
#include <gtest/gtest.h>
#include <stdexcept>

using namespace std;

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
// We use magic numbers here because they are physical parameters of the ammunition and are not expected to change.

TEST(getAmmoParameters, IncorrectAmmoName)
{
  EXPECT_THROW(getAmmoParameters("Failed Ammo name"), invalid_argument);
};

TEST(getAmmoParameters, returnCoorrectAmmoParams)
{
  EXPECT_EQ(getAmmoParameters("VOG-17"), AmmoParameters({0.35, 0.07, 0.}));
}

TEST(computeDropSolution, IncorrectAmmoName)
{
  BallisticsInput input{{0., 0., 0.}, {0., 0., 0.}, 0., 0., "Failed Ammo "};
  EXPECT_THROW(computeDropSolution(input), invalid_argument);
};

TEST(computeDropSolution, incorrectZCoord)
{
  BallisticsInput input{{180., 180., -100.}, {200., 200., 0.}, 10., 10., "VOG-17"};

  EXPECT_THROW(computeDropSolution(input), invalid_argument);
};

TEST(computeDropSolution, nearTargetDistance)
{
  BallisticsInput input{{180., 180., 100.}, {200., 200., 0.}, 10., 10., "VOG-17"};
  DropSolution expected{173.759, 173.759, 166.688, 166.688};

  DropSolution result = computeDropSolution(input);

  EXPECT_NEAR(expected.fire_x, result.fire_x, 1e-3);
  EXPECT_NEAR(expected.fire_y, result.fire_y, 1e-3);
  EXPECT_NEAR(expected.tmp_x, result.tmp_x, 1e-3);
  EXPECT_NEAR(expected.tmp_y, result.tmp_y, 1e-3);
};

TEST(computeDropSolution, distanteTarget)
{
  BallisticsInput input{{0., 0., 100.}, {300., 300., 0.}, 20., 50., "GLIDING-VOG"};
  DropSolution expected{242.711, 242.711};

  DropSolution result = computeDropSolution(input);

  EXPECT_NEAR(expected.fire_x, result.fire_x, 1e-3);
  EXPECT_NEAR(expected.fire_y, result.fire_y, 1e-3);
  EXPECT_NEAR(expected.tmp_x, result.tmp_x, 1e-3);
  EXPECT_NEAR(expected.tmp_y, result.tmp_y, 1e-3);
};

TEST(computeDropSolution, distante)
{
  BallisticsInput input{{543., 232., 120.}, {1034., 432., 0.}, 13., 12., "GLIDING-RKG"};
  DropSolution expected{966.534, 404.519};

  DropSolution result = computeDropSolution(input);

  EXPECT_NEAR(expected.fire_x, result.fire_x, 1e-3);
  EXPECT_NEAR(expected.fire_y, result.fire_y, 1e-3);
  EXPECT_NEAR(expected.tmp_x, result.tmp_x, 1e-3);
  EXPECT_NEAR(expected.tmp_y, result.tmp_y, 1e-3);
};

TEST(computeDropSolution, rkg3WithTmpPoint)
{
  BallisticsInput input{{543., 232., 120.}, {553., 242., 0.}, 13., 12., "RKG-3"};
  DropSolution expected{513.085, 202.085, 504.6, 193.6};

  DropSolution result = computeDropSolution(input);

  EXPECT_NEAR(expected.fire_x, result.fire_x, 1e-3);
  EXPECT_NEAR(expected.fire_y, result.fire_y, 1e-3);
  EXPECT_NEAR(expected.tmp_x, result.tmp_x, 1e-3);
  EXPECT_NEAR(expected.tmp_y, result.tmp_y, 1e-3);
};

TEST(computeDropSolution, m67SameCordTargeet)
{
  BallisticsInput input{{543., 232., 120.}, {543., 232., 0.}, 13., 12., "M67"};
  DropSolution expected{490.496, 232., 478.496, 232.};

  DropSolution result = computeDropSolution(input);

  EXPECT_NEAR(expected.fire_x, result.fire_x, 1e-3);
  EXPECT_NEAR(expected.fire_y, result.fire_y, 1e-3);
  EXPECT_NEAR(expected.tmp_x, result.tmp_x, 1e-3);
  EXPECT_NEAR(expected.tmp_y, result.tmp_y, 1e-3);
};

auto main(int argc, char **argv) -> int
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)