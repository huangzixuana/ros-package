/*
 * @Author: Hambin.Lu
 * @Description: ##
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef _UTIL_HPP_
#define _UTIL_HPP_

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <queue>

namespace serial_com {

template <typename T=float>
struct OneKalman
  {
  public:
    OneKalman(T vR = 0.1, T vQ = 5) : R(vR), Q(vQ) {}
    ~OneKalman() {}
    T Update(const T Data)
    {
      mid = last;
      p_mid = p_last + Q;
      kg = p_mid / (p_mid + R);
      now = mid + kg * (Data - mid);
      p_now = (1 - kg) * p_mid;
      p_last = p_now;
      last = now;

      return now;
    }

    void Reset(void)
    {
      last = 0;
      mid = 0;
      now = 0;
      p_last = 0;
      p_mid = 0;
      p_now = 0;
      kg = 0;
    }

  private:
    T R;
    T Q;
    T last = 0;
    T mid = 0;
    T now = 0;
    T p_last = 0;
    T p_mid = 0;
    T p_now = 0;
    T kg = 0;
  };

  template <typename T=float>
  struct PidControl
  {
  public:
    T PidCal(const T err)
    {
      T output;

      error = err;
      integ += error * dt;
      if (integ > iLimitMax)
        integ = iLimitMax;
      else if (integ < iLimitMix)
        integ = iLimitMix;

      deriv = error - prevError;
      outP = kp * error;
      outI = ki * integ;
      outD = kd * deriv;

      output = outP + outI + outD;
      if (output > outLimitMax)
        output = outLimitMax;
      else if (output < outLimitMix)
        output = outLimitMix;

      prevError = error;
      return output;
    }

    void PidInit(T p, T i, T d, T oMin, T oMax)
    {
      kp = p;
      ki = i;
      kd = d;
      outLimitMix = oMin;
      outLimitMax = oMax;
    }

    void PidReset(void)
    {
      error = 0;
      prevError = 0;
      integ = 0;
      deriv = 0;
      outP = 0;
      outI = 0;
      outD = 0;
    }

  private:
    T kp = 0;
    T ki = 0;
    T kd = 0;

    T error = 0;
    T prevError = 0;
    T integ = 0;
    T deriv = 0;
    T outP = 0;
    T outI = 0;
    T outD = 0;
    T iLimitMax = 0;
    T iLimitMix = 0;
    T outLimitMax = 0;
    T outLimitMix = 0;
    T dt = 0;
  };

}
#endif

