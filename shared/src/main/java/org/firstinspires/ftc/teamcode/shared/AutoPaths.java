package org.firstinspires.ftc.teamcode.shared;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.Arrays;
import java.util.List;

/**
 * Otonom pathleri. Robottaki otonom OpMode'u ve simülasyon aynı pathleri kullanır,
 * yani path'i burada değiştirip önce bilgisayarda deneyebilirsin.
 *
 * Koordinatlar Pedro koordinatlarıdır: inç, 0..144, açı radyan.
 * Pedro Visualizer'da çizdiğin noktaları buraya kopyalayabilirsin.
 */
public final class AutoPaths {
    private AutoPaths() {}

    public static final Pose START = new Pose(56, 8, Math.toRadians(90));

    private static final Pose SCORE = new Pose(56, 84, Math.toRadians(135));
    private static final Pose PICKUP_1 = new Pose(24, 84, Math.toRadians(180));
    private static final Pose PICKUP_2 = new Pose(24, 60, Math.toRadians(180));
    private static final Pose PARK = new Pose(40, 30, Math.toRadians(90));

    /** Örnek otonom: sırayla takip edilecek path'ler. Kendi rotanla değiştir. */
    public static List<Path> exampleAuto() {
        return Arrays.asList(
                Paths.line(START, SCORE).linear(START, SCORE),
                Paths.line(SCORE, PICKUP_1).linear(SCORE, PICKUP_1),
                Paths.curve(PICKUP_1, new Pose(48, 70), SCORE).linear(PICKUP_1, SCORE),
                Paths.curve(SCORE, new Pose(40, 64), PICKUP_2).linear(SCORE, PICKUP_2),
                Paths.line(PICKUP_2, PARK).linear(PICKUP_2, PARK)
        );
    }
}
