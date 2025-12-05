#include <Arduino.h>
#include "FeedforwardTuner.h"

FeedforwardTuner::FeedforwardTuner(
    float cvMin, float cvMax,
    int numSamples,
    unsigned long settleTimeMs,
    const float &pvRef
)
: _cvMin(cvMin), _cvMax(cvMax),
  _N(numSamples), _settleTime(settleTimeMs),
  _pvRef(pvRef)
{
    _cv = new float[_N];
    _pv = new float[_N];

    _state = SET_CV;
    _index = 0;
    _currentCV = _cvMin;
    _stateStartTime = millis();
    _b0 = 0;
    _b1 = 0;
}

bool FeedforwardTuner::IsFinished() const {
    return _state == DONE;
}

void FeedforwardTuner::RunTime() {
    if (_state == DONE) return;

    unsigned long now = millis();

    switch (_state) {

        case SET_CV:
            _currentCV = _cvMin +
                (_cvMax - _cvMin) * ((float)_index / (_N - 1));
            _stateStartTime = now;
            _state = WAIT_SETTLE;
            break;

        case WAIT_SETTLE:
            if (now - _stateStartTime >= _settleTime)
                _state = RECORD_POINT;
            break;

        case RECORD_POINT:
            _cv[_index] = _currentCV;
            _pv[_index] = _pvRef;

            _index++;

            if (_index >= _N) {
                ComputeOLS();
                _state = DONE;
            } else {
                _state = SET_CV;
            }
            break;

        default:
            break;
    }
}

void FeedforwardTuner::ComputeOLS() {
    float sumX=0, sumY=0, sumXY=0, sumXX=0;

    for (int i=0; i<_N; i++) {
        float x = _pv[i];
        float y = _cv[i];
        sumX  += x;
        sumY  += y;
        sumXY += x*y;
        sumXX += x*x;
    }

    float meanX = sumX / _N;
    float meanY = sumY / _N;

    float num = sumXY - _N * meanX * meanY;
    float den = sumXX - _N * meanX * meanX;

    _b1 = num / den;
    _b0 = meanY - _b1 * meanX;
}
