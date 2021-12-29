#include "xslam/dbow3/train_vocabulary_dataset.h"
#include "xslam/common/common.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

#include <string>

namespace xslam {
namespace dbow3 {

TEST(VocabularyTrain, train)
{
    std::string path = common::GetDBoW3DatasetDirectory();
    VocabularyTrain demo;
    demo.RunDemo(path);
}

} // namespace dbow3
} // namespace xslam