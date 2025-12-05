#ifndef FEEDFORWARDTUNER_H
#define FEEDFORWARDTUNER_H

#include <Arduino.h>

class FeedforwardTuner {
public:
    FeedforwardTuner(
        float cvMin, float cvMax,      // sweep range
        int numSamples,                // number of (CV,PV) samples
        unsigned long settleTimeMs,    // wait time for PV settling
        const float &pvRef             // reference to PV variable
    );

    // Call from loop()
    void RunTime();

    bool IsFinished() const;

    float GetB0() const { return _b0; }

    float GetB1() const { return _b1; }

    float GetCV() const { return _currentCV; };

private:
    enum State {
        SET_CV,
        WAIT_SETTLE,
        RECORD_POINT,
        DONE
    };

    void ComputeOLS();

    float *_cv;
    float *_pv;
    int _N;

    float _cvMin, _cvMax;
    unsigned long _settleTime;
    const float &_pvRef;   // reference to PV variable

    State _state;
    int _index;

    float _currentCV;
    unsigned long _stateStartTime;

    float _b0, _b1;
};

#endif
