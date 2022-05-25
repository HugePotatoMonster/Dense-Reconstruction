#pragma once
namespace SemiGlobalMatching {
	namespace Constant {
		enum class CostAggregationOption : i32 {
			SMC_NONE = 0,
			SMC_FOUR_PATH = 1,
			SMC_EIGHT_PATH = 2,
			SMC_FOUR_PATH_CUDA = 3,
			SMC_EIGHT_PATH_CUDA = 4,
		};
		enum class CostCalculationOption : i32 {
			SMC_CENSUS_TRANSFORM = 0
		};
	}
}