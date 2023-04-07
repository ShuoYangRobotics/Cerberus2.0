#include "vilo/VILOEstimator.hpp"

VILOEstimator::VILOEstimator() {
  feature_manager_ = std::make_unique<FeatureManager>(Rs);
}