package org.firstinspires.ftc.teamcode.shared;

/**
 * FTC Gamepad'in donanımdan bağımsız kopyası.
 * Robotta FTC'nin gamepad1'inden, simülasyonda bilgisayara takılı kumandadan doldurulur.
 *
 * Eksen işaretleri FTC ile AYNIDIR: çubuk YUKARI itildiğinde stickY NEGATİF olur.
 */
public class DriverInput {
    public double leftStickX, leftStickY, rightStickX, rightStickY;
    public double leftTrigger, rightTrigger;
    public boolean a, b, x, y;
    public boolean leftBumper, rightBumper;
    public boolean dpadUp, dpadDown, dpadLeft, dpadRight;
    public boolean start, back;

    // Bir önceki döngüdeki tuş durumları (justPressed için)
    private boolean pa, pb, px, py, plb, prb, pup, pdown, pleft, pright, pstart, pback;

    /** Her döngünün SONUNDA çağır: "yeni basıldı" bilgisini günceller. */
    public void endCycle() {
        pa = a; pb = b; px = x; py = y; plb = leftBumper; prb = rightBumper;
        pup = dpadUp; pdown = dpadDown; pleft = dpadLeft; pright = dpadRight;
        pstart = start; pback = back;
    }

    public boolean aPressed() { return a && !pa; }
    public boolean bPressed() { return b && !pb; }
    public boolean xPressed() { return x && !px; }
    public boolean yPressed() { return y && !py; }
    public boolean leftBumperPressed() { return leftBumper && !plb; }
    public boolean rightBumperPressed() { return rightBumper && !prb; }
    public boolean dpadUpPressed() { return dpadUp && !pup; }
    public boolean dpadDownPressed() { return dpadDown && !pdown; }
    public boolean dpadLeftPressed() { return dpadLeft && !pleft; }
    public boolean dpadRightPressed() { return dpadRight && !pright; }
    public boolean startPressed() { return start && !pstart; }
    public boolean backPressed() { return back && !pback; }

    /** Çubuklardan herhangi biri ölü bölgenin dışında mı? (otonomu iptal etmek için) */
    public boolean sticksActive(double deadband) {
        return Math.abs(leftStickX) > deadband || Math.abs(leftStickY) > deadband
                || Math.abs(rightStickX) > deadband || Math.abs(rightStickY) > deadband;
    }
}
