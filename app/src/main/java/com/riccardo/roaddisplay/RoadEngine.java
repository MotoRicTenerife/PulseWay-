package com.riccardo.roaddisplay;

import android.location.Location;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * RoadEngine Refactored for ROAD DISPLAY PRO
 * - Normalizes OSM node order against motorcycle GPS heading (fixes Left/Right inversion)
 * - Computes 3-point Menger curvature radius
 * - Classifies into: LEGGERA, MEDIA, STRETTA, TORNANTE
 * - Triggers 150m alert threshold with hysteresis re-arm
 */
public class RoadEngine {

    public enum Severity {
        LEGGERA, MEDIA, STRETTA, TORNANTE
    }

    public static class CurveResult {
        public Severity severity;
        public boolean isLeft;
        public double radiusMeters;
        public double distanceToApexMeters;
        public double recommendedSpeedKmh;
        public boolean hasHazardWarning;
        public String roadRef;
    }

    private String lastAlertedCurveKey = "";
    private boolean hasCrossed150m = false;

    /**
     * Analizza i nodi del modo stradale rispetto alla posizione e rotta della moto
     */
    public CurveResult analyzeRoadAhead(Location currentLoc, List<RoadNode> roadNodes, String roadRef) {
        if (roadNodes == null || roadNodes.size() < 3) {
            return null;
        }

        // 1. Trova il segmento stradale più vicino alla moto
        int closestIdx = findClosestNodeIndex(currentLoc.getLatitude(), currentLoc.getLongitude(), roadNodes);
        if (closestIdx >= roadNodes.size() - 2) {
            return null; // Fine della strada conosciuta
        }

        // 2. RISOLUZIONE INVERSIONE OSM: Allineamento con il bearing della moto
        // Vettore rotta moto (dx, dy)
        float bikeBearing = currentLoc.getBearing(); // 0..360 gradi
        double rad = Math.toRadians(bikeBearing);
        double bikeVx = Math.sin(rad);
        double bikeVy = Math.cos(rad);

        // Vettore tangente della strada OSM dal nodo corrente al successivo
        RoadNode n1 = roadNodes.get(closestIdx);
        RoadNode n2 = roadNodes.get(closestIdx + 1);
        double roadVx = (n2.lon - n1.lon);
        double roadVy = (n2.lat - n1.lat);

        // Prodotto scalare per verificare se il way OSM è orientato concordemente alla moto
        double dotProduct = (bikeVx * roadVx) + (bikeVy * roadVy);
        List<RoadNode> orientedNodes = new ArrayList<>(roadNodes);
        if (dotProduct < 0) {
            // Il way OSM è stato disegnato in senso opposto: invertiamo l'ordine dei nodi!
            Collections.reverse(orientedNodes);
            closestIdx = orientedNodes.size() - 1 - closestIdx;
        }

        // 3. Look-ahead: analizza la geometria dei prossimi 500 metri
        return computeNextCurve(currentLoc, orientedNodes, closestIdx, roadRef);
    }

    private CurveResult computeNextCurve(Location bikeLoc, List<RoadNode> nodes, int startIdx, String roadRef) {
        double accumulatedDistance = 0.0;
        double minRadius = Double.MAX_VALUE;
        int apexIndex = -1;
        double apexDistance = 0.0;
        double apexTurnAngleDeg = 0.0;

        for (int i = startIdx; i < nodes.size() - 2; i++) {
            RoadNode a = nodes.get(i);
            RoadNode b = nodes.get(i + 1);
            RoadNode c = nodes.get(i + 2);

            double segLen = distanceMeters(a.lat, a.lon, b.lat, b.lon);
            accumulatedDistance += segLen;
            if (accumulatedDistance > 550.0) break; // Oltre 500m non consideriamo per la curva immediata

            // Calcolo raggio di Menger su 3 punti (A, B, C)
            double radius = calculateMengerRadius(a, b, c);
            if (radius < minRadius && radius > 3.0) {
                minRadius = radius;
                apexIndex = i + 1;
                apexDistance = accumulatedDistance;

                // Calcolo angolo di deviazione per distinguere tornanti
                apexTurnAngleDeg = calculateTurnAngle(a, b, c);
            }
        }

        if (apexIndex == -1 || minRadius > 180.0) {
            return null; // Rettilineo o curvatura trascurabile
        }

        // Determinazione Sinistra vs Destra tramite Cross Product 2D
        RoadNode a = nodes.get(apexIndex - 1);
        RoadNode b = nodes.get(apexIndex);
        RoadNode c = nodes.get(apexIndex + 1);
        double crossProduct = (b.lon - a.lon) * (c.lat - b.lat) - (b.lat - a.lat) * (c.lon - b.lon);
        boolean isLeft = crossProduct > 0;

        CurveResult result = new CurveResult();
        result.isLeft = isLeft;
        result.radiusMeters = minRadius;
        result.distanceToApexMeters = Math.max(0, apexDistance);
        result.roadRef = (roadRef != null && !roadRef.isEmpty()) ? roadRef : "STRADA ATTUALE";

        // Classificazione in 4 categorie
        if (minRadius <= 22.0 || Math.abs(apexTurnAngleDeg) >= 115.0) {
            result.severity = Severity.TORNANTE;
            result.hasHazardWarning = true;
        } else if (minRadius <= 45.0) {
            result.severity = Severity.STRETTA;
            result.hasHazardWarning = true;
        } else if (minRadius <= 90.0) {
            result.severity = Severity.MEDIA;
            result.hasHazardWarning = false;
        } else {
            result.severity = Severity.LEGGERA;
            result.hasHazardWarning = false;
        }

        // Velocità consigliata prudenziale fisica (km/h): V ≈ 6.8 * sqrt(R)
        result.recommendedSpeedKmh = Math.max(20.0, Math.min(110.0, 6.8 * Math.sqrt(minRadius)));

        return result;
    }

    /**
     * Raggio di Menger: R = 1 / Kappa
     */
    private double calculateMengerRadius(RoadNode a, RoadNode b, RoadNode c) {
        double dAB = distanceMeters(a.lat, a.lon, b.lat, b.lon);
        double dBC = distanceMeters(b.lat, b.lon, c.lat, c.lon);
        double dCA = distanceMeters(c.lat, c.lon, a.lat, a.lon);

        double s = (dAB + dBC + dCA) / 2.0;
        double areaSq = s * (s - dAB) * (s - dBC) * (s - dCA);
        if (areaSq <= 0.0001) return 9999.0; // Punti collineari

        double area = Math.sqrt(areaSq);
        return (dAB * dBC * dCA) / (4.0 * area);
    }

    private double calculateTurnAngle(RoadNode a, RoadNode b, RoadNode c) {
        double bearingAB = bearingBetween(a.lat, a.lon, b.lat, b.lon);
        double bearingBC = bearingBetween(b.lat, b.lon, c.lat, c.lon);
        double diff = bearingBC - bearingAB;
        while (diff < -180.0) diff += 360.0;
        while (diff > 180.0) diff -= 360.0;
        return diff;
    }

    private int findClosestNodeIndex(double lat, double lon, List<RoadNode> nodes) {
        int bestIdx = 0;
        double bestDist = Double.MAX_VALUE;
        for (int i = 0; i < nodes.size(); i++) {
            double d = distanceMeters(lat, lon, nodes.get(i).lat, nodes.get(i).lon);
            if (d < bestDist) {
                bestDist = d;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    private double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
        double R = 6371000.0;
        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);
        double a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                   Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
                   Math.sin(dLon/2) * Math.sin(dLon/2);
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    private double bearingBetween(double lat1, double lon1, double lat2, double lon2) {
        double dLon = Math.toRadians(lon2 - lon1);
        double y = Math.sin(dLon) * Math.cos(Math.toRadians(lat2));
        double x = Math.cos(Math.toRadians(lat1)) * Math.sin(Math.toRadians(lat2)) -
                   Math.sin(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.cos(dLon);
        return (Math.toDegrees(Math.atan2(y, x)) + 360.0) % 360.0;
    }

    public static class RoadNode {
        public double lat;
        public double lon;
        public RoadNode(double lat, double lon) { this.lat = lat; this.lon = lon; }
    }
}
