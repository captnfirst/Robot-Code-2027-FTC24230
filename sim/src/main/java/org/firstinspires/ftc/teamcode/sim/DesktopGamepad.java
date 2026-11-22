package org.firstinspires.ftc.teamcode.sim;

import com.studiohartman.jamepad.ControllerManager;
import com.studiohartman.jamepad.ControllerState;

import org.firstinspires.ftc.teamcode.shared.DriverInput;

/**
 * Bilgisayara takılı Xbox / PlayStation / Logitech kumandasını okur ve
 * FTC gamepad'i ile aynı işaretlerle DriverInput'a yazar.
 */
public class DesktopGamepad implements AutoCloseable {
    private final ControllerManager manager;
    private boolean wasConnected = false;

    private DesktopGamepad(ControllerManager manager) {
        this.manager = manager;
    }

    /** Kumanda kütüphanesi yüklenemezse null döner (ör. desteklenmeyen işletim sistemi). */
    public static DesktopGamepad tryCreate() {
        java.io.PrintStream err = System.err;
        try {
            ControllerManager m = new ControllerManager();
            // Jamepad ek eşleme dosyası bulamayınca uzun bir hata basar ama SDL'in kendi
            // eşlemeleriyle sorunsuz çalışır; o gereksiz mesajı gizle.
            System.setErr(new java.io.PrintStream(new java.io.ByteArrayOutputStream()));
            try {
                m.initSDLGamepad();
            } finally {
                System.setErr(err);
            }
            return new DesktopGamepad(m);
        } catch (Throwable t) {
            System.setErr(err);
            System.out.println("[Sim] Gamepad kutuphanesi baslatilamadi: " + t);
            return null;
        }
    }

    /** Kumandayı okuyup in'e yazar. Kumanda bağlı değilse false döner ve in'i sıfırlar. */
    public boolean read(DriverInput in) {
        manager.update();
        ControllerState s = manager.getState(0);
        if (!s.isConnected) {
            if (wasConnected) System.out.println("[Sim] Kumanda baglantisi koptu.");
            wasConnected = false;
            zero(in);
            return false;
        }
        if (!wasConnected) System.out.println("[Sim] Kumanda baglandi: " + s.controllerType);
        wasConnected = true;

        double ySign = SimConfig.INVERT_STICK_Y ? -1.0 : 1.0;
        in.leftStickX = s.leftStickX;
        in.leftStickY = ySign * s.leftStickY;
        in.rightStickX = s.rightStickX;
        in.rightStickY = ySign * s.rightStickY;
        in.leftTrigger = s.leftTrigger;
        in.rightTrigger = s.rightTrigger;
        in.a = s.a;
        in.b = s.b;
        in.x = s.x;
        in.y = s.y;
        in.leftBumper = s.lb;
        in.rightBumper = s.rb;
        in.dpadUp = s.dpadUp;
        in.dpadDown = s.dpadDown;
        in.dpadLeft = s.dpadLeft;
        in.dpadRight = s.dpadRight;
        in.start = s.start;
        in.back = s.back;
        return true;
    }

    private static void zero(DriverInput in) {
        in.leftStickX = in.leftStickY = in.rightStickX = in.rightStickY = 0;
        in.leftTrigger = in.rightTrigger = 0;
        in.a = in.b = in.x = in.y = false;
        in.leftBumper = in.rightBumper = false;
        in.dpadUp = in.dpadDown = in.dpadLeft = in.dpadRight = false;
        in.start = in.back = false;
    }

    @Override
    public void close() {
        manager.quitSDLGamepad();
    }
}
