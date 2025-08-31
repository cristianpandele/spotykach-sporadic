#include "DelayProc.h"

void DelayProc::init(float sr, size_t maxDelaySamples)
{
    sampleRate = sr;
    delay.Init();
    dispersion.prepare(sampleRate);
    setDelay(delayMs);
}

void DelayProc::setDelay(float dMs)
{
    delayMs = dMs;
    delay.SetDelay(sampleRate * delayMs / 1000.0f);
}

void DelayProc::setParameters(float dMs, float fb, float fFreq)
{
    delayMs = dMs;
    feedback = fb;
    setDelay(delayMs);
    // Note: fFreq parameter is now unused since tone filter is removed
}

float DelayProc::process(float in)
{
    // Update input envelope
    float inputAbs = fabs(in);
    float attackCoeff = 1.0f - expf(-1.0f / (envAttackMs / 1000.0f * sampleRate));
    float releaseCoeff = 1.0f - expf(-1.0f / (envReleaseMs / 1000.0f * sampleRate));
    if (inputAbs > inputLevel)
    {
        inputLevel += (inputAbs - inputLevel) * attackCoeff;
    }
    else
    {
        inputLevel += (inputAbs - inputLevel) * releaseCoeff;
    }

    // Update age
    currentAge += growthRate / sampleRate;
    if (currentAge > 1.0f) currentAge = 1.0f;

    // Calculate dispersion based on age
    dispersionAmount = currentAge * 0.5f; // Example: dispersion grows with age

    // Set dispersion parameters
    Dispersion::Parameters dispParams;
    dispParams.dispersionAmount = dispersionAmount;
    dispParams.allpassFreq = 1000.0f; // Fixed frequency for now
    dispersion.setParameters(dispParams);

    // Process delay
    float read = delay.Read();
    // Apply dispersion
    float dispersed = dispersion.processSample(read);
    delay.Write(dispersed * feedback + in);

    // Update output envelope
    float outputAbs = fabs(dispersed);
    if (outputAbs > outputLevel)
    {
        outputLevel += (outputAbs - outputLevel) * attackCoeff;
    }
    else
    {
        outputLevel += (outputAbs - outputLevel) * releaseCoeff;
    }

    return dispersed;
}
