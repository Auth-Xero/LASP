package com.lora.audiostreaming;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

public class MorseCodeGenerator {
    private static final Map<Character, String> MORSE_CODE = new HashMap<>();

    static {
        MORSE_CODE.put('A', "·–");
        MORSE_CODE.put('B', "–···");
        MORSE_CODE.put('C', "–·–·");
        MORSE_CODE.put('D', "–··");
        MORSE_CODE.put('E', "·");
        MORSE_CODE.put('F', "··–·");
        MORSE_CODE.put('G', "––·");
        MORSE_CODE.put('H', "····");
        MORSE_CODE.put('I', "··");
        MORSE_CODE.put('J', "·–––");
        MORSE_CODE.put('K', "–·–");
        MORSE_CODE.put('L', "·–··");
        MORSE_CODE.put('M', "––");
        MORSE_CODE.put('N', "–·");
        MORSE_CODE.put('O', "–––");
        MORSE_CODE.put('P', "·––·");
        MORSE_CODE.put('Q', "––·–");
        MORSE_CODE.put('R', "·–·");
        MORSE_CODE.put('S', "···");
        MORSE_CODE.put('T', "–");
        MORSE_CODE.put('U', "··–");
        MORSE_CODE.put('V', "···–");
        MORSE_CODE.put('W', "·––");
        MORSE_CODE.put('X', "–··–");
        MORSE_CODE.put('Y', "–·––");
        MORSE_CODE.put('Z', "––··");
        MORSE_CODE.put('0', "–––––");
        MORSE_CODE.put('1', "·––––");
        MORSE_CODE.put('2', "··–––");
        MORSE_CODE.put('3', "···––");
        MORSE_CODE.put('4', "····–");
        MORSE_CODE.put('5', "·····");
        MORSE_CODE.put('6', "–····");
        MORSE_CODE.put('7', "––···");
        MORSE_CODE.put('8', "–––··");
        MORSE_CODE.put('9', "––––·");
        MORSE_CODE.put(' ', "/");
    }

    private static final int DOT_DURATION_MS = 100;
    private static final int TONE_FREQUENCY = 800;
    private static final int SAMPLE_RATE = 8000;

    public String textToMorse(String text) {
        StringBuilder morse = new StringBuilder();

        for (char c : text.toUpperCase().toCharArray()) {
            String code = MORSE_CODE.get(c);
            if (code != null) {
                if (morse.length() > 0) {
                    morse.append(" ");
                }
                morse.append(code);
            }
        }

        return morse.toString();
    }

    public void playMorse(String text, ConcurrentLinkedQueue<Float> sampleBuffer, float volume) {
        String morse = textToMorse(text);

        for (char symbol : morse.toCharArray()) {
            if (symbol == '·') {
                generateTone(DOT_DURATION_MS, sampleBuffer, volume);
                sleep(DOT_DURATION_MS);
            } else if (symbol == '–') {
                generateTone(DOT_DURATION_MS * 3, sampleBuffer, volume);
                sleep(DOT_DURATION_MS);
            } else if (symbol == ' ') {
                sleep(DOT_DURATION_MS * 3);
            } else if (symbol == '/') {
                sleep(DOT_DURATION_MS * 7);
            }
        }
    }

    private void generateTone(int durationMs, ConcurrentLinkedQueue<Float> sampleBuffer, float volume) {
        int numSamples = (SAMPLE_RATE * durationMs) / 1000;

        for (int i = 0; i < numSamples; i++) {
            double angle = 2.0 * Math.PI * TONE_FREQUENCY * i / SAMPLE_RATE;
            float sample = (float) Math.sin(angle) * volume;
            sampleBuffer.offer(sample);
        }
    }

    private void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}