package org.firstinspires.ftc.teamcode.shared;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/**
 * Pedro Foresight katsayıları. TEK KAYNAK burası:
 * - Robotta: pedro/Constants.java bu config'i kullanır.
 * - Simülasyonda: SimMain bu config'i kullanır.
 * Robotta yeniden tune ettiğinde değerleri sadece buraya yaz.
 */
public final class PedroTuning {
    private PedroTuning() {}

    // Tuner'ın ölçtüğü fiziksel değerler (inç/s, inç/s²). Simülasyonun fizik modeli de
    // bunları kullanır; böylece sim'deki robot gerçek robot kadar hızlı/yavaş davranır.
    public static final double MAX_FORWARD_VELOCITY = 75.73396055250937;
    public static final double MAX_STRAFE_VELOCITY = 63.57414091588733;
    public static final double NATURAL_FORWARD_DECEL = 23.290467203611882;
    public static final double NATURAL_STRAFE_DECEL = 31.37512380172239;

    public static final ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.28174747981768367);
                Controller secondaryTranslationalForward = Controller.proportional(0.10409816536132027);
                Controller primaryTranslationalLateral = Controller.proportional(0.6240497469780856);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23056970659205706);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013015655143799272));
                c.brake.set(Controller.proportionalFeedforward(0.011063306872229381));

                c.headingFeedback.set(Controller.proportional(4.520170329099175));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05914270029891561, 0.010248538324945692));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09740627470965782, 0.19269920720519093));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002393725535590111, 4.1328197955757794E-4));

                c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
                c.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY);
                c.naturalForwardDeceleration.set(NATURAL_FORWARD_DECEL);
                c.naturalStrafeDeceleration.set(NATURAL_STRAFE_DECEL);
            }
    );
}
