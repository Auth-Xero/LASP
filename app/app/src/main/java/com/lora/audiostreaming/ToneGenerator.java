package com.lora.audiostreaming;

public class ToneGenerator {
    private final int frequency;
    private final int sampleRate;
    private int sampleIndex;
    private String waveformType = "sine";

    public ToneGenerator(int frequency, int sampleRate) {
        this.frequency = frequency;
        this.sampleRate = sampleRate;
        this.sampleIndex = 0;
    }

    public void setWaveformType(String type) {
        this.waveformType = type;
    }

    public float[] generateSamples(int numSamples) {
        float[] samples = new float[numSamples];

        for (int i = 0; i < numSamples; i++) {
            double angle = 2.0 * Math.PI * frequency * sampleIndex / sampleRate;

            switch (waveformType) {
                case "sine":
                    samples[i] = (float) Math.sin(angle);
                    break;

                case "square":
                    samples[i] = Math.sin(angle) >= 0 ? 1.0f : -1.0f;
                    break;

                case "sawtooth":
                    double phase = (sampleIndex * frequency % sampleRate) / (double) sampleRate;
                    samples[i] = (float) (2.0 * phase - 1.0);
                    break;

                case "triangle":
                    double triPhase = (sampleIndex * frequency % sampleRate) / (double) sampleRate;
                    if (triPhase < 0.5) {
                        samples[i] = (float) (4.0 * triPhase - 1.0);
                    } else {
                        samples[i] = (float) (3.0 - 4.0 * triPhase);
                    }
                    break;

                default:
                    samples[i] = (float) Math.sin(angle);
            }

            sampleIndex++;
            if (sampleIndex >= sampleRate) {
                sampleIndex = 0;
            }
        }

        return samples;
    }

    public void reset() {
        sampleIndex = 0;
    }
}