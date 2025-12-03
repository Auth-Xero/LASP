package com.lora.audiostreaming;

import java.util.ArrayList;
import java.util.List;

public class Resampler {
    private final float ratio;
    private final List<Float> inputBuffer;
    private final List<Float> outputBuffer;
    private float inputIndex;
    private final float[] filter;
    private final int filterSize = 16;

    public Resampler(int fromRate, int toRate) {
        this.ratio = (float) fromRate / toRate;
        this.inputBuffer = new ArrayList<>();
        this.outputBuffer = new ArrayList<>();
        this.inputIndex = 0;
        this.filter = createKaiserFilter(filterSize);
    }

    private float[] createKaiserFilter(int size) {
        float[] filter = new float[size * 2 + 1];
        float beta = 5.0f;
        float i0Beta = besselI0(beta);

        for (int n = -size; n <= size; n++) {
            float windowPos = (float) n / size;
            float window = besselI0(beta * (float) Math.sqrt(1 - windowPos * windowPos)) / i0Beta;

            if (n == 0) {
                filter[n + size] = window;
            } else {
                float x = (float) (Math.PI * n);
                float sinc = (float) (Math.sin(x) / x);
                filter[n + size] = sinc * window;
            }
        }

        // Normalize filter
        float sum = 0;
        for (float f : filter) {
            sum += f;
        }
        for (int i = 0; i < filter.length; i++) {
            filter[i] /= sum;
        }

        return filter;
    }

    private float besselI0(float x) {
        float sum = 1.0f;
        float term = 1.0f;
        float threshold = 1e-12f;

        for (int k = 1; k < 50; k++) {
            term *= (x * x) / (4 * k * k);
            sum += term;
            if (term < threshold * sum) break;
        }

        return sum;
    }

    public float[] process(short[] inputSamples) {
        // Convert short samples to float and add to buffer
        for (short sample : inputSamples) {
            inputBuffer.add(sample / 32768.0f);
        }

        // Process resampling
        while (inputIndex + filterSize < inputBuffer.size()) {
            int baseIndex = (int) inputIndex;
            float fraction = inputIndex - baseIndex;

            float sample = 0;
            for (int i = -filterSize; i <= filterSize; i++) {
                int sampleIndex = baseIndex + i;
                if (sampleIndex >= 0 && sampleIndex < inputBuffer.size()) {
                    int filterIndex = i + filterSize;
                    sample += inputBuffer.get(sampleIndex) * filter[filterIndex];
                }
            }

            outputBuffer.add(sample);
            inputIndex += ratio;
        }

        // Clean up old input samples
        if (inputIndex > 2000) {
            int removeCount = (int) inputIndex;
            for (int i = 0; i < removeCount && !inputBuffer.isEmpty(); i++) {
                inputBuffer.remove(0);
            }
            inputIndex -= removeCount;
        }

        // Get output samples
        float[] output = new float[outputBuffer.size()];
        for (int i = 0; i < output.length; i++) {
            output[i] = outputBuffer.get(i);
        }
        outputBuffer.clear();

        return output;
    }

    public void reset() {
        inputBuffer.clear();
        outputBuffer.clear();
        inputIndex = 0;
    }
}