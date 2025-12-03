package com.lora.audiostreaming;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;
import java.util.ArrayList;
import java.util.List;

public class WaveformView extends View {
    private Paint paint;
    private Path path;
    private List<Float> waveformData;
    private int maxDataPoints = 200;

    public WaveformView(Context context) {
        super(context);
        init();
    }

    public WaveformView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public WaveformView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        paint = new Paint();
        paint.setColor(0xFF3B82F6); // Blue color
        paint.setStrokeWidth(3f);
        paint.setStyle(Paint.Style.STROKE);
        paint.setAntiAlias(true);

        path = new Path();
        waveformData = new ArrayList<>();
    }

    public void updateWaveform(float[] audioData) {
        // Downsample audio data for visualization
        int step = Math.max(1, audioData.length / maxDataPoints);
        waveformData.clear();

        for (int i = 0; i < audioData.length; i += step) {
            waveformData.add(audioData[i]);
        }

        // Keep only recent data
        while (waveformData.size() > maxDataPoints) {
            waveformData.remove(0);
        }

        invalidate(); // Trigger redraw
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        if (waveformData.isEmpty()) {
            // Draw center line when no data
            canvas.drawLine(0, getHeight() / 2f, getWidth(), getHeight() / 2f, paint);
            return;
        }

        path.reset();

        float width = getWidth();
        float height = getHeight();
        float centerY = height / 2f;
        float xStep = width / (float) maxDataPoints;

        // Draw waveform
        for (int i = 0; i < waveformData.size(); i++) {
            float x = i * xStep;
            float y = centerY + (waveformData.get(i) * centerY * 0.8f);

            if (i == 0) {
                path.moveTo(x, y);
            } else {
                path.lineTo(x, y);
            }
        }

        canvas.drawPath(path, paint);
    }

    public void clear() {
        waveformData.clear();
        invalidate();
    }
}