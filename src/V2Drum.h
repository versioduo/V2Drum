// © Kay Sievers <kay@versioduo.com>, 2020-2024
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <Arduino.h>

class V2Drum {
public:
  struct Config {
    // The number of steps to map the measurement to. 128 steps will emit values
    // from 0 to 127.
    uint16_t nSteps;

    // The exponential smoothing constant.
    float alpha;

    // Hysteresis lag; the amount of jitter we accept without changing the step value.
    // The unit is a fraction of the normalized 0..1 value of the min..max range.
    float lag;

    struct {
      // The normalized 0..1 value of the analog measurement range.
      float min;
      float max;

      // Correction curve exponent.
      float exponent;
    } pressure;

    struct {
      // The normalized 0..1 value of the analog measurement range.
      float min;
      float max;

      // Correction curve exponent.
      float exponent;

      // Sample time to detect the rising edge. Depending on the hardware, values
      // are in the range of 2 to 50 ms.
      uint32_t risingUsec;

      // Minimum time to hold note. The settle time to check for release again,
      // the hardware may bounce to zero, while the note is still held.
      uint32_t holdUsec;

      // Time to delay pressure/aftertouch events after detectiing a 'hit'.
      uint32_t pressureDelayUsec;

      // Time for the release to settle.
      uint32_t releaseUsec;
    } hit;

    struct {
      // Duration of the falling pressure to measure the release velocity.
      float minUsec;
      float maxUsec;
    } release;
  };

  constexpr V2Drum(const struct Config* config) : _config(config) {}
  void begin() {}

  void reset() {
    _now      = {};
    _history  = {};
    _pressure = {};
    _rising   = {};
    _hit      = {};
    _falling  = {};
  }

  // Measure and emit pressure events. A fast rising edge will emit a hit event,
  // the release to idle will clear it.
  void loop() {
    if (V2Base::getUsecSince(_now.usec) < 500)
      return;

    _now.usec = V2Base::getUsec();

    measure();
    sendPressure();

    switch (_now.state) {
      case State::Idle:
        if (_now.step == 0)
          break;

        _rising.usec = V2Base::getUsec();
        _now.state   = State::Rising;
        break;

      case State::Rising:
        if (_now.step == 0) {
          _now.state = State::Release;
          break;
        }

        // Remember the maximum value, it might bounce.
        if (_now.fraction > _rising.pressure)
          _rising.pressure = _now.fraction;

        // Sample timespan.
        if (V2Base::getUsecSince(_rising.usec) < _config->hit.risingUsec)
          break;

        // Require minimum rise distance. If we rise too slow, it is not a hit.
        if (_rising.pressure <= _config->hit.min) {
          _pressure.enabled = true;
          _now.state        = State::Release;
          break;
        }

        _now.state = State::Hit;
        break;

      case State::Hit: {
        // Normalized 0..1 fraction of the min..max range.
        if (_rising.pressure > _config->hit.max)
          _rising.pressure = _config->hit.max;

        float fraction = (_rising.pressure - _config->hit.min) / (_config->hit.max - _config->hit.min);

        // Apply exponential correction curve.
        fraction = powf(fraction, _config->hit.exponent);

        _hit.velocity = ceilf(fraction * (_config->nSteps - 1));
        _hit.usec     = V2Base::getUsec();
        _now.state    = State::HitHold;
        handleHit(_hit.velocity);
      } break;

      case State::HitHold:
        if (_hit.holdUsec == 0) {
          _hit.holdUsec = V2Base::getUsec();
          _falling.usec = V2Base::getUsec();
        }

        if (V2Base::getUsecSince(_hit.holdUsec) < _config->hit.holdUsec)
          break;

        // Clear the falling duration whenever the pressure rises again.
        if (_now.step >= _falling.step) {
          _falling.usec = V2Base::getUsec();
          _falling.step = _now.step;
        }

        if (_now.step == 0) {
          _pressure.enabled = true;
          _now.state        = State::HitRelease;
          break;
        }

        // If we stay in 'Hold', enable the pressure events only after the delay timespan.
        if (V2Base::getUsecSince(_hit.holdUsec) > _config->hit.pressureDelayUsec)
          _pressure.enabled = true;
        break;

      case State::HitRelease: {
        _hit.releaseUsec = V2Base::getUsec();

        uint32_t duration = _hit.releaseUsec - _falling.usec;
        if (duration > _config->release.maxUsec)
          duration = _config->release.maxUsec;
        else if (duration < _config->release.minUsec)
          duration = _config->release.minUsec;

        const float range    = _config->release.maxUsec - _config->release.minUsec;
        const float fraction = ((float)duration - _config->release.minUsec) / range;
        _falling.velocity    = 127 - (fraction * 126.f);

        _now.state = State::Release;
        handleRelease(_falling.velocity);
      } break;

      case State::Release:
        if (_now.fraction > 0.f)
          break;

        // Wait for the release to settle.
        if (V2Base::getUsecSince(_hit.releaseUsec) < _config->hit.releaseUsec)
          break;

        _now    = {};
        _rising = {};
        _hit    = {};

        // Make sure we send zeros if we sent out non-zero values.
        if (_pressure.sent)
          handlePressure(0, 0);

        if (_pressure.rawSent)
          handlePressureRaw(0, 0);
        _pressure = {};

        break;
    }
  }

  float getFraction() {
    return _pressure.fraction;
  }

  uint16_t getStep() {
    return _pressure.step;
  }

protected:
  // Normalized 0...1 analog measurement.
  virtual float handleMeasurement() = 0;

  // Sent whenever the step value changes.
  virtual void handlePressureRaw(float fraction, uint16_t step) {}

  // Sent whenever the step value changes. If a 'Hit' event is generated in tthis transition
  // transition, it is guaranteed to be emitted after the 'Hit.
  virtual void handlePressure(float fraction, uint16_t step) {}

  // Sent when a 'Hit' was detected.
  virtual void handleHit(uint8_t velocity) {}

  // Sent when the 'Hit' is released.
  virtual void handleRelease(uint8_t velocity) {}

private:
  enum class State {
    // No pressure detected.
    Idle,

    // Pressure rising, measured in a short timeframe. The minimum hit value
    // needs to be reached in this timeframe, a slow-rising value is a pressure
    // change only.
    Rising,

    // Hit event, with the maximum value of the measured pressure as velocity.
    Hit,

    // Active hit.
    HitHold,

    // Hit release event (velocity == 0).
    HitRelease,

    // Reset, wait for the pressure to be fully released and settled.
    Release
  };

  const struct Config* _config;

  struct {
    State    state;
    uint32_t usec;
    float    analog;
    float    fraction;
    uint16_t step;
  } _now{};

  struct {
    // The smoothed-out, normalized (0..1) analog measurement.
    float analog;

    // The edge of the lag range, set by the previous value change.
    float lag;
  } _history{};

  struct {
    float    fraction;
    uint8_t  step;
    uint32_t usec;
    bool     enabled;
    bool     sent;
    bool     rawSent;
  } _pressure{};

  struct {
    float    pressure;
    uint32_t usec;
  } _rising{};

  struct {
    uint8_t  velocity;
    uint32_t usec;
    uint32_t holdUsec;
    uint32_t releaseUsec;
  } _hit{};

  struct {
    uint32_t usec;
    uint16_t step;
    uint8_t  velocity;
  } _falling{};

  void measure() {
    _now.analog = handleMeasurement();

    // Low-pass filter, smooth the value.
    _history.analog *= 1 - _config->alpha;
    _history.analog += _now.analog * _config->alpha;

    if (_history.analog < _config->pressure.min) {
      _now.fraction = 0;
      _now.step     = 0;
      _history.lag  = 0 - _config->lag;

    } else if (_history.analog > _config->pressure.max) {
      _now.fraction = 1;
      _now.step     = _config->nSteps - 1;
      _history.lag  = 1 + _config->lag;

    } else {
      // Normalized 0..1 fraction of the min..max range.
      _now.fraction = (_history.analog - _config->pressure.min) / (_config->pressure.max - _config->pressure.min);

      // Exponential correction curve.
      _now.fraction = powf(_now.fraction, _config->pressure.exponent);

      // If the new measurement is inside the lag, don't update, use the current step value.
      if (fabs(_now.fraction - _history.lag) >= _config->lag)
        _now.step = roundf(_now.fraction * (_config->nSteps - 1));

      else
        _now.step = _pressure.step;
    }
  }

  void sendPressure() {
    if (_pressure.step == _now.step)
      return;

    if (V2Base::getUsecSince(_pressure.usec) < 20 * 1000)
      return;

    // Reposition the edge of the lag. We follow monotonic changes immediately,
    // but apply the lag if the direction changes.
    if (_now.fraction - _history.lag > 0.f)
      _history.lag = _now.fraction - _config->lag;

    else
      _history.lag = _now.fraction + _config->lag;

    _pressure.usec     = V2Base::getUsec();
    _pressure.fraction = _now.fraction;
    _pressure.step     = _now.step;

    // The final zero values will be sent at 'Release'.
    if (_now.step == 0)
      return;

    if (_pressure.enabled) {
      _pressure.sent = true;
      handlePressure(_now.fraction, _now.step);
    }

    _pressure.rawSent = true;
    handlePressureRaw(_now.fraction, _now.step);
  }
};
