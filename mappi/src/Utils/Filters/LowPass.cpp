/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Low Pass Filtering functions.
 *
 * -----------------------------------------------------------------------------
 */

#include "Utils/Filters/LowPass.hpp"

namespace mappi::filters {

template<typename T>
T lowPassFilter(T input)
{
  static T iCutOffFrequency = 5.0;  // [Hz]
  static T iDeltaTime = 0.05;       // [s]

  static auto start_time = std::chrono::system_clock::now();
  auto end_time = std::chrono::system_clock::now();

  static std::chrono::duration<double> elapsed_time;
  elapsed_time = end_time - start_time;

  start_time = end_time; // keep counting

  if( (elapsed_time.count() < 0.01) || 
      (elapsed_time.count() > 0.1) )
    iDeltaTime = 0.05;
  else
    iDeltaTime = elapsed_time.count();

  T ePow = 1 - exp(-iDeltaTime * 2 * M_PI * iCutOffFrequency);
  static T output = 0.0f;

  return output += (input-output)*ePow;
}

template<typename T>
void lowPassFilter(T input_0, T input_1, T input_2,
                  T& output_0, T& output_1, T& output_2)
{
  static T iCutOffFrequency = 5.0;  // [Hz]
  static T iDeltaTime = 0.05;       // [s]

  static auto start_time = std::chrono::system_clock::now();
  auto end_time = std::chrono::system_clock::now();

  static std::chrono::duration<double> elapsed_time;
  elapsed_time = end_time - start_time;

  start_time = end_time; // keep counting

  if( (elapsed_time.count() < 0.01) || 
      (elapsed_time.count() > 0.1) )
    iDeltaTime = 0.05;
  else
    iDeltaTime = elapsed_time.count();

  T ePow = 1 - exp(-iDeltaTime * 2 * M_PI * iCutOffFrequency);
  static T cum_out_0 = 0.0f;
  static T cum_out_1 = 0.0f;
  static T cum_out_2 = 0.0f;

  cum_out_0 += (input_0-cum_out_0)*ePow;
  cum_out_1 += (input_1-cum_out_1)*ePow;
  cum_out_2 += (input_2-cum_out_2)*ePow;

  output_0 = cum_out_0;
  output_1 = cum_out_1;
  output_2 = cum_out_2;
}

} // namespace mappi::filters