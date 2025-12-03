package com.lora.audiostreaming;

import android.Manifest;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import android.media.AudioTrack;
import android.media.MediaRecorder;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import androidx.cardview.widget.CardView;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "LASP";
    private static final String ACTION_USB_PERMISSION = "com.lora.audiostreaming.USB_PERMISSION";

    // Audio constants matching web version
    private static final int TARGET_SAMPLE_RATE = 8000;
    private static final int PACKET_SIZE = 480;
    private static final int BAUD_RATE = 115200;

    // Frame constants for packet protocol
    private static final byte FRAME_START_1 = 0x61;
    private static final byte FRAME_START_2 = 0x6F;
    private static final byte FRAME_START_3 = 0x73;
    private static final byte FRAME_TYPE_AUDIO = (byte) 0xAD;

    // UI Elements
    private TextView connectionStatus;
    private TextView sourceStatus;
    private TextView packetCountView;
    private TextView streamRateView;
    private TextView statPackets;
    private TextView statRate;
    private TextView statSamples;
    private TextView statUptime;
    private TextView logView;
    private TextView volumeValue;
    private Button connectBtn;
    private Button disconnectBtn;
    private SeekBar volumeSlider;

    // Audio source cards
    private CardView micCard;
    private CardView fileCard;
    private CardView toneCard;
    private CardView morseCard;

    // Audio control panels
    private LinearLayout micControls;
    private LinearLayout fileControls;
    private LinearLayout toneControls;
    private LinearLayout morseControls;

    // USB Serial
    private UsbManager usbManager;
    private UsbSerialPort serialPort;
    private SerialInputOutputManager serialIoManager;
    private ExecutorService executor = Executors.newSingleThreadExecutor();

    // Audio components
    private AudioRecord audioRecord;
    private AudioTrack audioTrack;
    private boolean isRecording = false;
    private boolean isStreaming = false;
    private String currentSource = null;

    // Statistics
    private int packetCount = 0;
    private long streamStartTime = 0;
    private Timer statsTimer;

    // Audio processing
    private ConcurrentLinkedQueue<Float> sampleBuffer = new ConcurrentLinkedQueue<>();
    private Resampler resampler;
    private Handler handler = new Handler(Looper.getMainLooper());
    private Timer packetTimer;
    private float audioVolume = 0.5f;

    // Permission request codes
    private static final int PERMISSION_REQUEST_RECORD_AUDIO = 1;
    private static final int PERMISSION_REQUEST_STORAGE = 2;

    // StringBuilder for log messages with max lines
    private StringBuilder logMessages = new StringBuilder();
    private int logLineCount = 0;
    private static final int MAX_LOG_LINES = 50;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initializeViews();
        setupEventListeners();
        checkPermissions();
        initializeUSB();
        initializeAudioComponents();
        startStatsTimer();

        logMessage("System initialized with enhanced audio processing", "success");
    }

    private void initializeViews() {
        // Status bar views
        connectionStatus = findViewById(R.id.connectionStatus);
        sourceStatus = findViewById(R.id.sourceStatus);
        packetCountView = findViewById(R.id.packetCount);
        streamRateView = findViewById(R.id.streamRate);

        // Connection buttons
        connectBtn = findViewById(R.id.connectBtn);
        disconnectBtn = findViewById(R.id.disconnectBtn);

        // Audio source cards
        micCard = findViewById(R.id.micCard);
        fileCard = findViewById(R.id.fileCard);
        toneCard = findViewById(R.id.toneCard);
        morseCard = findViewById(R.id.morseCard);

        // Control panels
        micControls = findViewById(R.id.micControls);
        fileControls = findViewById(R.id.fileControls);
        toneControls = findViewById(R.id.toneControls);
        morseControls = findViewById(R.id.morseControls);

        // Statistics views
        statPackets = findViewById(R.id.statPackets);
        statRate = findViewById(R.id.statRate);
        statSamples = findViewById(R.id.statSamples);
        statUptime = findViewById(R.id.statUptime);

        // Volume control
        volumeSlider = findViewById(R.id.volumeSlider);
        volumeValue = findViewById(R.id.volumeValue);

        // Log view
        logView = findViewById(R.id.logContainer);
    }

    private void setupEventListeners() {
        // Connection buttons
        connectBtn.setOnClickListener(v -> connectUSB());
        disconnectBtn.setOnClickListener(v -> disconnectUSB());

        // Audio source cards
        micCard.setOnClickListener(v -> selectSource("mic"));
        fileCard.setOnClickListener(v -> selectSource("file"));
        toneCard.setOnClickListener(v -> selectSource("tone"));
        morseCard.setOnClickListener(v -> selectSource("morse"));

        // Volume slider
        volumeSlider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                audioVolume = progress / 100.0f;
                volumeValue.setText(progress + "%");
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });

        // Microphone controls
        Button startMicBtn = findViewById(R.id.startMic);
        Button stopMicBtn = findViewById(R.id.stopMic);
        startMicBtn.setOnClickListener(v -> startMicrophone());
        stopMicBtn.setOnClickListener(v -> stopMicrophone());

        // File controls
        Button selectFileBtn = findViewById(R.id.selectFile);
        Button playFileBtn = findViewById(R.id.playFile);
        Button pauseFileBtn = findViewById(R.id.pauseFile);
        Button stopFileBtn = findViewById(R.id.stopFile);
        selectFileBtn.setOnClickListener(v -> selectAudioFile());
        playFileBtn.setOnClickListener(v -> playAudioFile());
        pauseFileBtn.setOnClickListener(v -> pauseAudioFile());
        stopFileBtn.setOnClickListener(v -> stopAudioFile());

        // Tone generator controls
        Button startToneBtn = findViewById(R.id.startTone);
        Button stopToneBtn = findViewById(R.id.stopTone);
        SeekBar frequencySlider = findViewById(R.id.frequencySlider);
        TextView freqValue = findViewById(R.id.freqValue);

        startToneBtn.setOnClickListener(v -> startToneGenerator());
        stopToneBtn.setOnClickListener(v -> stopToneGenerator());

        frequencySlider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                int frequency = 100 + (progress * 29); // 100Hz to 3000Hz
                freqValue.setText(frequency + " Hz");
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });

        // Morse code controls
        Button playMorseBtn = findViewById(R.id.playMorse);
        Button stopMorseBtn = findViewById(R.id.stopMorse);
        playMorseBtn.setOnClickListener(v -> playMorse());
        stopMorseBtn.setOnClickListener(v -> stopMorse());
    }

    private void checkPermissions() {
        List<String> permissionsToRequest = new ArrayList<>();

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
                != PackageManager.PERMISSION_GRANTED) {
            permissionsToRequest.add(Manifest.permission.RECORD_AUDIO);
        }

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE)
                != PackageManager.PERMISSION_GRANTED) {
            permissionsToRequest.add(Manifest.permission.READ_EXTERNAL_STORAGE);
        }

        if (!permissionsToRequest.isEmpty()) {
            ActivityCompat.requestPermissions(this,
                    permissionsToRequest.toArray(new String[0]),
                    PERMISSION_REQUEST_RECORD_AUDIO);
        }
    }

    private void initializeUSB() {
        usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);

        // Register USB receiver for connection events
        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        filter.addAction(ACTION_USB_PERMISSION);

        // Use ContextCompat for backward compatibility and specify RECEIVER_NOT_EXPORTED
        // since we're only receiving broadcasts from our own app and the system
        ContextCompat.registerReceiver(
                this,
                usbReceiver,
                filter,
                ContextCompat.RECEIVER_NOT_EXPORTED
        );
    }

    private void initializeAudioComponents() {
        // Initialize the resampler for converting sample rates
        int nativeRate = AudioTrack.getNativeOutputSampleRate(AudioManager.STREAM_MUSIC);
        resampler = new Resampler(nativeRate, TARGET_SAMPLE_RATE);

        // Setup audio record for microphone input
        int minBufferSize = AudioRecord.getMinBufferSize(
                nativeRate,
                AudioFormat.CHANNEL_IN_MONO,
                AudioFormat.ENCODING_PCM_16BIT
        );

        if (minBufferSize != AudioRecord.ERROR_BAD_VALUE) {
            try {
                audioRecord = new AudioRecord(
                        MediaRecorder.AudioSource.MIC,
                        nativeRate,
                        AudioFormat.CHANNEL_IN_MONO,
                        AudioFormat.ENCODING_PCM_16BIT,
                        minBufferSize * 2
                );
            } catch (SecurityException e) {
                logMessage("Microphone permission not granted", "error");
            }
        }
    }

    private void connectUSB() {
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);

        if (availableDrivers.isEmpty()) {
            logMessage("No USB serial devices found", "error");
            Toast.makeText(this, "No USB serial devices found", Toast.LENGTH_SHORT).show();
            return;
        }

        UsbSerialDriver driver = availableDrivers.get(0);
        UsbDevice device = driver.getDevice();

        if (!usbManager.hasPermission(device)) {
            // Create an immutable PendingIntent for USB permission request
            // We use FLAG_IMMUTABLE because this intent doesn't need to be modified
            // after creation - it's just a callback for the permission response
            int flags = PendingIntent.FLAG_UPDATE_CURRENT;
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.M) {
                // On Android 6.0 (API 23) and higher, we can use FLAG_IMMUTABLE
                // On Android 12 (API 31) and higher, this is required
                flags |= PendingIntent.FLAG_IMMUTABLE;
            }

            PendingIntent permissionIntent = PendingIntent.getBroadcast(
                    this, 0, new Intent(ACTION_USB_PERMISSION), flags
            );
            usbManager.requestPermission(device, permissionIntent);
            return;
        }

        UsbDeviceConnection connection = usbManager.openDevice(device);
        if (connection == null) {
            logMessage("Failed to open USB device", "error");
            return;
        }

        serialPort = driver.getPorts().get(0);
        try {
            serialPort.open(connection);
            serialPort.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            serialIoManager = new SerialInputOutputManager(serialPort, serialListener);
            executor.submit(serialIoManager);

            handler.post(() -> {
                connectBtn.setEnabled(false);
                disconnectBtn.setEnabled(true);
                connectionStatus.setText("Connected");
                connectionStatus.setTextColor(getResources().getColor(R.color.status_connected));
                logMessage("Serial connection established at " + BAUD_RATE + " baud", "success");
            });

        } catch (IOException e) {
            logMessage("Failed to connect: " + e.getMessage(), "error");
            try {
                serialPort.close();
            } catch (IOException ignored) {}
        }
    }

    private void disconnectUSB() {
        stopAllAudio();

        if (serialIoManager != null) {
            serialIoManager.stop();
            serialIoManager = null;
        }

        if (serialPort != null) {
            try {
                serialPort.close();
            } catch (IOException e) {
                Log.e(TAG, "Error closing port", e);
            }
            serialPort = null;
        }

        handler.post(() -> {
            connectBtn.setEnabled(true);
            disconnectBtn.setEnabled(false);
            connectionStatus.setText("Disconnected");
            connectionStatus.setTextColor(getResources().getColor(R.color.status_inactive));
            logMessage("Serial connection terminated", "info");
        });
    }

    private void selectSource(String source) {
        // Reset all cards and controls
        micCard.setCardBackgroundColor(getResources().getColor(R.color.card_background));
        fileCard.setCardBackgroundColor(getResources().getColor(R.color.card_background));
        toneCard.setCardBackgroundColor(getResources().getColor(R.color.card_background));
        morseCard.setCardBackgroundColor(getResources().getColor(R.color.card_background));

        micControls.setVisibility(View.GONE);
        fileControls.setVisibility(View.GONE);
        toneControls.setVisibility(View.GONE);
        morseControls.setVisibility(View.GONE);

        stopAllAudio();
        currentSource = source;

        // Highlight selected card and show controls
        switch(source) {
            case "mic":
                micCard.setCardBackgroundColor(getResources().getColor(R.color.card_active));
                micControls.setVisibility(View.VISIBLE);
                sourceStatus.setText("Microphone");
                break;
            case "file":
                fileCard.setCardBackgroundColor(getResources().getColor(R.color.card_active));
                fileControls.setVisibility(View.VISIBLE);
                sourceStatus.setText("Audio File");
                break;
            case "tone":
                toneCard.setCardBackgroundColor(getResources().getColor(R.color.card_active));
                toneControls.setVisibility(View.VISIBLE);
                sourceStatus.setText("Tone Generator");
                break;
            case "morse":
                morseCard.setCardBackgroundColor(getResources().getColor(R.color.card_active));
                morseControls.setVisibility(View.VISIBLE);
                sourceStatus.setText("Morse Code");
                break;
        }

        sourceStatus.setTextColor(getResources().getColor(R.color.status_streaming));
    }

    private void startMicrophone() {
        if (audioRecord == null || audioRecord.getState() != AudioRecord.STATE_INITIALIZED) {
            logMessage("Audio record not initialized", "error");
            return;
        }

        isRecording = true;
        audioRecord.startRecording();
        startStreaming();

        // Start recording thread
        new Thread(() -> {
            short[] buffer = new short[PACKET_SIZE];

            while (isRecording) {
                int read = audioRecord.read(buffer, 0, buffer.length);
                if (read > 0) {
                    // Convert to float and add to sample buffer
                    for (int i = 0; i < read; i++) {
                        float sample = buffer[i] / 32768.0f * audioVolume;
                        sampleBuffer.offer(sample);
                    }
                }
            }
        }).start();

        logMessage("Microphone input activated", "success");
    }

    private void stopMicrophone() {
        isRecording = false;
        if (audioRecord != null) {
            audioRecord.stop();
        }
        stopAllAudio();
    }

    private void selectAudioFile() {
        Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
        intent.setType("audio/*");
        startActivityForResult(intent, 100);
    }

    private void playAudioFile() {
        // Implementation for audio file playback
        logMessage("Audio file playback started", "success");
        startStreaming();
    }

    private void pauseAudioFile() {
        logMessage("Audio file playback paused", "info");
        stopStreaming();
    }

    private void stopAudioFile() {
        stopAllAudio();
    }

    private void startToneGenerator() {
        SeekBar freqSlider = findViewById(R.id.frequencySlider);
        int frequency = 100 + (freqSlider.getProgress() * 29);

        ToneGenerator toneGen = new ToneGenerator(frequency, TARGET_SAMPLE_RATE);
        startStreaming();

        // Generate tone samples in background thread
        new Thread(() -> {
            while (isStreaming) {
                float[] samples = toneGen.generateSamples(PACKET_SIZE);
                for (float sample : samples) {
                    sampleBuffer.offer(sample * audioVolume);
                }

                try {
                    Thread.sleep(10); // Small delay to prevent buffer overflow
                } catch (InterruptedException e) {
                    break;
                }
            }
        }).start();

        logMessage("Tone generator started: " + frequency + "Hz", "success");
    }

    private void stopToneGenerator() {
        stopAllAudio();
    }

    private void playMorse() {
        TextView morseInput = findViewById(R.id.morseInput);
        String text = morseInput.getText().toString().toUpperCase();

        if (text.isEmpty()) {
            logMessage("No message provided for Morse encoding", "warning");
            return;
        }

        MorseCodeGenerator morseGen = new MorseCodeGenerator();
        String morseCode = morseGen.textToMorse(text);

        TextView morseOutput = findViewById(R.id.morseOutput);
        morseOutput.setText(morseCode);

        startStreaming();

        // Play morse code in background
        new Thread(() -> {
            morseGen.playMorse(text, sampleBuffer, audioVolume);
            handler.post(() -> {
                stopAllAudio();
                logMessage("Morse code transmission completed", "info");
            });
        }).start();

        logMessage("Morse code transmission initiated", "success");
    }

    private void stopMorse() {
        stopAllAudio();
    }

    private void stopAllAudio() {
        isRecording = false;
        isStreaming = false;

        if (audioRecord != null && audioRecord.getState() == AudioRecord.STATE_INITIALIZED) {
            audioRecord.stop();
        }

        stopStreaming();

        sourceStatus.setTextColor(getResources().getColor(R.color.status_inactive));
    }

    private void startStreaming() {
        if (isStreaming) return;

        isStreaming = true;
        packetCount = 0;
        streamStartTime = System.currentTimeMillis();
        sampleBuffer.clear();

        startPacketSender();
        logMessage("Audio streaming initiated with precise timing", "success");
    }

    private void stopStreaming() {
        isStreaming = false;
        if (packetTimer != null) {
            packetTimer.cancel();
            packetTimer = null;
        }
        streamStartTime = 0;
        logMessage("Audio streaming terminated", "info");
    }

    private void startPacketSender() {
        int packetIntervalMs = (int) Math.floor((PACKET_SIZE / (float) TARGET_SAMPLE_RATE) * 1000) - 4;

        packetTimer = new Timer();
        packetTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (!isStreaming) {
                    cancel();
                    return;
                }

                if (sampleBuffer.size() >= PACKET_SIZE) {
                    byte[] audioData = new byte[PACKET_SIZE];

                    for (int i = 0; i < PACKET_SIZE; i++) {
                        Float sample = sampleBuffer.poll();
                        if (sample != null) {
                            audioData[i] = convertToPCM(sample);
                        } else {
                            audioData[i] = (byte) 128; // Silence
                        }
                    }

                    byte[] packet = buildAudioPacket(audioData);
                    sendPacket(packet);
                }
            }
        }, 0, packetIntervalMs);

        logMessage("Packet timer started: " + packetIntervalMs + "ms interval", "info");
    }

    private byte convertToPCM(float sample) {
        // Add dithering for better audio quality
        float dither = (float) ((Math.random() - 0.5) / 256.0);
        float dithered = sample + dither;

        // Clamp and apply soft limiting
        float clamped = Math.max(-1, Math.min(1, dithered));
        if (Math.abs(clamped) > 0.95f) {
            clamped = (float) (Math.tanh(clamped * 2) * 0.95);
        }

        // Convert to unsigned 8-bit PCM
        int scaled = (int) ((clamped * 127.5) + 127.5);
        return (byte) Math.max(0, Math.min(255, scaled));
    }

    private byte[] buildAudioPacket(byte[] audioData) {
        int totalSize = 4 + 1 + PACKET_SIZE + 1;
        byte[] packet = new byte[totalSize];
        byte checksum = 0;
        int idx = 0;

        // Frame header
        packet[idx] = FRAME_START_1;
        checksum ^= packet[idx++];
        packet[idx] = FRAME_START_2;
        checksum ^= packet[idx++];
        packet[idx] = FRAME_START_3;
        checksum ^= packet[idx++];
        packet[idx] = FRAME_TYPE_AUDIO;
        checksum ^= packet[idx++];
        packet[idx] = (byte) 240; // Additional byte as in original
        checksum ^= packet[idx++];

        // Audio data
        for (int i = 0; i < PACKET_SIZE; i++) {
            packet[idx] = audioData[i];
            checksum ^= packet[idx++];
        }

        // Checksum
        packet[idx] = checksum;

        return packet;
    }

    private void sendPacket(byte[] packet) {
        if (serialPort == null || !serialPort.isOpen()) return;

        try {
            serialPort.write(packet, 1000);
            packetCount++;
            handler.post(this::updateStats);
        } catch (IOException e) {
            Log.e(TAG, "Error sending packet", e);
            handler.post(() -> {
                logMessage("Transmission error: " + e.getMessage(), "error");
                stopStreaming();
            });
        }
    }

    private void updateStats() {
        statPackets.setText(String.valueOf(packetCount));
        packetCountView.setText(String.valueOf(packetCount));
        statSamples.setText(String.format(Locale.US, "%,d", packetCount * PACKET_SIZE));

        if (streamStartTime > 0) {
            long elapsed = (System.currentTimeMillis() - streamStartTime) / 1000;
            long minutes = elapsed / 60;
            long seconds = elapsed % 60;
            statUptime.setText(String.format(Locale.US, "%d:%02d", minutes, seconds));

            float rate = elapsed > 0 ? (float) packetCount / elapsed : 0;
            statRate.setText(String.format(Locale.US, "%.1f", rate));
            streamRateView.setText(String.format(Locale.US, "%.1f pkt/s", rate));
        }
    }

    private void startStatsTimer() {
        statsTimer = new Timer();
        statsTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (isStreaming) {
                    handler.post(() -> updateStats());
                }
            }
        }, 0, 1000);
    }

    private void logMessage(String message, String type) {
        String timestamp = new java.text.SimpleDateFormat("HH:mm:ss").format(new java.util.Date());
        String logEntry = timestamp + " [" + type.toUpperCase() + "] " + message;

        handler.post(() -> {
            // Manage log buffer size
            logMessages.append(logEntry).append("\n");
            logLineCount++;

            // Remove old lines if exceeded max
            if (logLineCount > MAX_LOG_LINES) {
                String currentLog = logMessages.toString();
                int firstNewline = currentLog.indexOf("\n");
                if (firstNewline > 0) {
                    logMessages.delete(0, firstNewline + 1);
                    logLineCount--;
                }
            }

            logView.setText(logMessages.toString());
        });
    }

    // USB Broadcast Receiver
    private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        if (device != null) {
                            connectUSB();
                        }
                    } else {
                        logMessage("USB permission denied", "error");
                    }
                }
            } else if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
                UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                if (device != null && serialPort != null) {
                    disconnectUSB();
                }
            }
        }
    };

    // Serial listener for incoming data
    private final SerialInputOutputManager.Listener serialListener = new SerialInputOutputManager.Listener() {
        @Override
        public void onNewData(byte[] data) {
            // Handle incoming serial data if needed
        }

        @Override
        public void onRunError(Exception e) {
            handler.post(() -> {
                logMessage("Serial connection error: " + e.getMessage(), "error");
                disconnectUSB();
            });
        }
    };

    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopAllAudio();
        disconnectUSB();
        if (statsTimer != null) {
            statsTimer.cancel();
        }
        unregisterReceiver(usbReceiver);
        executor.shutdown();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (requestCode == 100 && resultCode == RESULT_OK && data != null) {
            Uri audioUri = data.getData();
            if (audioUri != null) {
                TextView fileLabel = findViewById(R.id.fileLabel);
                fileLabel.setText(audioUri.getLastPathSegment());
                findViewById(R.id.playFile).setEnabled(true);
                logMessage("Audio file loaded: " + audioUri.getLastPathSegment(), "success");
            }
        }
    }
}