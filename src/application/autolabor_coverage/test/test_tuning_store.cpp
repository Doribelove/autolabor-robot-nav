#include <autolabor_coverage/tuning_store.h>
#include <autolabor_coverage/HybridTuningConfig.h>
#include <gtest/gtest.h>
using namespace autolabor_coverage;
TEST(TuningStore, RestartLoadsSavedValuesInsteadOfLaunchDefaults) {
  char pattern[]="/tmp/hybrid-tuning-test-XXXXXX";ASSERT_NE(nullptr,::mkdtemp(pattern));
  const std::string file=std::string(pattern)+"/nested/hybrid.json";
  auto config=HybridTuningConfig::__getDefault__();config.hybrid_motion_step=.22;config.hybrid_max_expansions=62000;
  saveTuning(file,config);
  auto launch=HybridTuningConfig::__getDefault__();launch.hybrid_motion_step=.3;launch.hybrid_max_expansions=120000;
  auto restored=loadTuning(file,launch);
  EXPECT_DOUBLE_EQ(.22,restored.hybrid_motion_step);EXPECT_EQ(62000,restored.hybrid_max_expansions);
  std::ifstream stream(file);EXPECT_TRUE(stream.good());
}
TEST(TuningStore, CorruptFileIsNotSilentlyReset) {
  char pattern[]="/tmp/hybrid-tuning-bad-XXXXXX";ASSERT_NE(nullptr,::mkdtemp(pattern));
  const std::string file=std::string(pattern)+"/hybrid.json";
  {std::ofstream stream(file);stream<<"{broken";}
  EXPECT_THROW(loadTuning(file,HybridTuningConfig::__getDefault__()),std::exception);
}
TEST(TuningStore, FailedStagePreservesPreviousFile) {
  char pattern[]="/tmp/hybrid-tuning-stage-XXXXXX";ASSERT_NE(nullptr,::mkdtemp(pattern));
  const std::string file=std::string(pattern)+"/hybrid.json";
  auto config=HybridTuningConfig::__getDefault__();saveTuning(file,config);
  EXPECT_THROW(saveTuning(file+"/invalid.json",config),std::exception);
  EXPECT_TRUE(sameTuning(loadTuning(file,config),config));
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
