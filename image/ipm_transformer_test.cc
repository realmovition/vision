#include "image/ipm_transformer.h"

#include "gtest/gtest.h"

namespace wheel {
namespace vision {
namespace image {
namespace {

TEST(IpmTransformerTest, TransformSucceeds) {
  const IpmTransformer transformer;
  EXPECT_TRUE(transformer.Transform());
}

}  // namespace
}  // namespace image
}  // namespace vision
}  // namespace wheel
