#ifndef MAPPING_PROBABILITY_VALUES_H_
#define MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#define DCHECK_LE(val1, val2) (val1 < val2 ? val1 : val2)
#define DCHECK_GE(val1, val2) (val1 > val2 ? val1 : val2)

//把概率转换成odd的形式
inline float Odds(float probability)
{
  return probability / (1.f - probability);
}

inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;

template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
inline float ClampProbability(const float probability)
{
  return Clamp(probability, kMinProbability, kMaxProbability);
}

//unknown对应的value
constexpr u_int16_t kUnknownProbabilityValue = 0;
//32768
constexpr u_int16_t kUpdateMarker = 1u << 15;

inline int RoundToInt(const float x) { return std::lround(x); }

// Converts a probability to a uint16 in the [1, 32767] range.
// 把概率转换到[1,32767]
inline u_int16_t ProbabilityToValue(const float probability)
{
  const int value =
      RoundToInt((ClampProbability(probability) - kMinProbability) *
                         (32766.f / (kMaxProbability - kMinProbability))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

extern const std::vector<float>* const kValueToProbability;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
inline float ValueToProbability(const u_int16_t value) {
  return (*kValueToProbability)[value];
}

// 0 is unknown, [1, 32767] maps to [kMinProbability, kMaxProbability].
//　把[1,32767]映射到[kMinProbability, kMaxProbability]
//　传入一个整数　返回一个概率
//  也就是这个整数对应的概率
//　这个函数被下面的PrecomputeValueToProbability()函数调用
//　这里的value是[1,32767]
float SlowValueToProbability(const u_int16_t value)
{
  DCHECK_GE(value, 0);
  DCHECK_LE(value, 32767);
  if (value == kUnknownProbabilityValue)
  {
    // Unknown cells have kMinProbability.
    return kMinProbability;
  }
  const float kScale = (kMaxProbability - kMinProbability) / 32766.f;
  return value * kScale + (kMinProbability - kScale);
}

//计算所有的value[1,32768]到probability的转换
const std::vector<float>* PrecomputeValueToProbability()
{
  std::vector<float>* result = new std::vector<float>;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  //　这里计算两遍，因此无论加不加kUpdateMarker都可以进行转换
  //  kUpdateMarker＝32768
  for (int repeat = 0; repeat != 2; ++repeat)
  {
    for (int value = 0; value != 32768; ++value)
    {
      result->emplace_back(SlowValueToProbability(value));
    }
  }
  return result;
}

//实现计算好的value到probability的转换的数组
const std::vector<float>* const kValueToProbability = PrecomputeValueToProbability();

//计算查询表
//为了使用odds而计算的查询表
//这个函数传入的参数只有两种情况:
//1.hit_probability　对应的 odds
//2.miss_probability 对应的 odds
std::vector<u_int16_t> ComputeLookupTableToApplyOdds(const float odds)
{
  std::vector<u_int16_t> result;
  result.emplace_back(ProbabilityToValue(ProbabilityFromOdds(odds)));//+ kUpdateMarker

  for (int cell = 1; cell != 32768; ++cell)
  {
    //这里的步骤为：
    //1.把cell转换为概率
    //2.把概率转换为odd　然后乘以传入的参数odds
    //3.步骤２的得到的odds，转换为概率
    //4.把概率转换为cell　存入数组中
    result.emplace_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell])))); //+ kUpdateMarker
  }
  return result;
}

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_