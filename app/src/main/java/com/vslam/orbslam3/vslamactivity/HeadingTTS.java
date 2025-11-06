package com.vslam.orbslam3.vslamactivity;

import android.content.Context;
import android.speech.tts.TextToSpeech;
import android.util.Log;

import java.util.Locale;

public class HeadingTTS {

    private static final String TAG = "HeadingTTS";
    private TextToSpeech tts;
    private long lastSpeakTime = 0L;
    private String lastText = "";

    public HeadingTTS(Context context) {
        tts = new TextToSpeech(context, status -> {
            if (status == TextToSpeech.SUCCESS) {
                tts.setLanguage(Locale.CHINA);          // 中文
                tts.setSpeechRate(1.0f);                // 语速
                tts.setPitch(1.0f);                     // 音调
            } else {
                Log.e(TAG, "TTS 初始化失败");
            }
        });
    }

    /**
     * 播报角度（单位：度），保留一位小数
     */
    public void speakHeading(String text) {
        long now = System.currentTimeMillis();
        // 1. 内容相同且 1000 ms 内直接返回
        if (text.equals(lastText) && now - lastSpeakTime < 1000) {
            return;
        }
        // 2. 真正播报
        if (tts != null) {
            tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, null);
            lastSpeakTime = now;
            lastText = text;
        }
    }

    public void shutdown() {
        if (tts != null) {
            tts.stop();
            tts.shutdown();
            tts = null;
        }
    }
}