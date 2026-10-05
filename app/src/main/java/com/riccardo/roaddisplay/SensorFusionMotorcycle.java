package com.riccardo.roaddisplay;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Sensor Fusion specifico per dinamica motociclistica:
 * - Filtra le vibrazioni del manubrio con filtro passa-basso a 12 Hz
 * - Compensa la forza centrifuga in curva usando roll-rate e velocità GPS
 * - Calcola il vero angolo di piega (Lean Angle)
 */
public class SensorFusionMotorcycle implements SensorEventListener {

    private float filteredRollDeg = 0f;
    private float maxLeanLeftDeg = 0f;
    private float maxLeanRightDeg = 0f;
    private float lateralG = 0f;

    private long lastTimestampNs = 0;
    private float currentSpeedMs = 0f;

    // Filtro Passa-Basso Butterworth 1° ordine per vibrazioni del manubrio
    private static final float ALPHA_VIBRATION = 0.82f;
    private float[] rawGyro = new float[3];

    public void updateGpsSpeed(float speedKmh) {
        this.currentSpeedMs = speedKmh / 3.6f;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            // Filtro antivibrazione sulle armoniche dei giri motore
            rawGyro[0] = ALPHA_VIBRATION * rawGyro[0] + (1 - ALPHA_VIBRATION) * event.values[0];
            rawGyro[1] = ALPHA_VIBRATION * rawGyro[1] + (1 - ALPHA_VIBRATION) * event.values[1];
            rawGyro[2] = ALPHA_VIBRATION * rawGyro[2] + (1 - ALPHA_VIBRATION) * event.values[2];

            if (lastTimestampNs != 0) {
                float dt = (event.timestamp - lastTimestampNs) * 1.0f / 1_000_000_000.0f;
                if (dt > 0 && dt < 0.2f) {
                    // Integrazione giroscopica sull'asse di rollio della moto
                    float rollRateDegS = (float) Math.toDegrees(rawGyro[0]);
                    float integratedRoll = filteredRollDeg + rollRateDegS * dt;

                    // Stima dinamica della piega dalla velocità e yaw rate (forza centrifuga):
                    // tan(phi) = (v * omega_z) / g
                    float yawRateRadS = rawGyro[2];
                    float dynamicLeanDeg = 0f;
                    if (currentSpeedMs > 5.0f) { // Sopra i 18 km/h
                        float tanPhi = (currentSpeedMs * yawRateRadS) / 9.81f;
                        dynamicLeanDeg = (float) Math.toDegrees(Math.atan(tanPhi));
                    }

                    // Filtro complementare: 96% giroscopio, 4% compensazione dinamica
                    if (currentSpeedMs > 5.0f) {
                        filteredRollDeg = 0.96f * integratedRoll + 0.04f * dynamicLeanDeg;
                    } else {
                        filteredRollDeg = integratedRoll * 0.98f; // Decadimento da fermo
                    }

                    // Calcolo accelerazione laterale (G-force)
                    lateralG = (float) Math.sin(Math.toRadians(filteredRollDeg));

                    // Aggiornamento massimi di sessione
                    if (filteredRollDeg < -maxLeanLeftDeg) {
                        maxLeanLeftDeg = Math.abs(filteredRollDeg);
                    }
                    if (filteredRollDeg > maxLeanRightDeg) {
                        maxLeanRightDeg = filteredRollDeg;
                    }
                }
            }
            lastTimestampNs = event.timestamp;
        }
    }

    public float getLeanAngleDeg() {
        return filteredRollDeg;
    }

    public float getMaxLeanLeftDeg() {
        return maxLeanLeftDeg;
    }

    public float getMaxLeanRightDeg() {
        return maxLeanRightDeg;
    }

    public float getLateralG() {
        return lateralG;
    }

    public void resetPeakLean() {
        maxLeanLeftDeg = 0f;
        maxLeanRightDeg = 0f;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}
}
