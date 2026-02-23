package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.EnumMap;

public class Ruleta {

    public static final double SLOT_C1 = 0.04;
        public static final double SLOT_C2 = 0.265;
    public static final double SLOT_C3 = 0.485;

    public static final double SLOT_S1 = 0.376;
    public static final double SLOT_S2 = 0.6;
    public static final double SLOT_S3 = 0.82;
    public static final double SAFE = 0.52;
    private static double lastpos = 0;

    public enum Color { PURPLE, GREEN }

    public enum Slot {
        C1(SLOT_C1, false),
        C2(SLOT_C2, false),
        C3(SLOT_C3, false),
        S1(SLOT_S1, true),
        S2(SLOT_S2, true),
        S3(SLOT_S3, true);

        public final double pos;
        public final boolean scoring;

        Slot(double pos, boolean scoring) {
            this.pos = pos;
            this.scoring = scoring;
        }
    }

    public enum Plan3 {
        PPP(Color.PURPLE, Color.PURPLE, Color.PURPLE),
        PPG(Color.PURPLE, Color.PURPLE, Color.GREEN),
        PGP(Color.PURPLE, Color.GREEN,  Color.PURPLE),
        GPP(Color.GREEN,  Color.PURPLE, Color.PURPLE),
        PGG(Color.PURPLE, Color.GREEN,  Color.GREEN),
        GPG(Color.GREEN,  Color.PURPLE, Color.GREEN),
        GGP(Color.GREEN,  Color.GREEN,  Color.PURPLE),
        GGG(Color.GREEN,  Color.GREEN,  Color.GREEN);

        private final Color[] seq;
        Plan3(Color... seq) { this.seq = seq; }
        public Color[] seq() { return seq; }
    }

    private final Servo s1, s2;
    private final EnumMap<Slot, Color> slotColor = new EnumMap<>(Slot.class);
    private final Deque<Color> scoreQueue = new ArrayDeque<>();

    public Ruleta(HardwareMap hw) {
        s1 = hw.get(Servo.class, "ruleta1");
        s2 = hw.get(Servo.class, "ruleta2");
        clear();
    }

    public void goTo(Slot slot) {
        s1.setPosition(slot.pos);
        s2.setPosition(slot.pos);
    }
    public void setPoz(double pozitie){
        lastpos = pozitie;
        s1.setPosition(pozitie);
        s2.setPosition(pozitie);
    }
    public static double getPosition(){
        return lastpos;
    }

    public void clear() {
        for (Slot s : Slot.values()) slotColor.put(s, null);
        scoreQueue.clear();
    }

    public boolean isEmpty(Slot slot) {
        return slotColor.get(slot) == null;
    }

    public Slot firstFreeCollectSlot() {
        if (isEmpty(Slot.C1)) return Slot.C1;
        if (isEmpty(Slot.C2)) return Slot.C2;
        if (isEmpty(Slot.C3)) return Slot.C3;
        return null;
    }
    public Slot lastFreeCollectSlot() {
        if (isEmpty(Slot.C3)) return Slot.C3;
        if (isEmpty(Slot.C2)) return Slot.C2;
        if (isEmpty(Slot.C1)) return Slot.C1;
        return null;
    }

    public Slot onBallIntake(boolean isGreen) {
        Color c = isGreen ? Color.GREEN : Color.PURPLE;
        Slot free = firstFreeCollectSlot();
        if (free == null) return null;
        slotColor.put(free, c);
        return free;
    }

    public void setPlan(Plan3 plan) {
        scoreQueue.clear();
        scoreQueue.addAll(Arrays.asList(plan.seq()));
    }

    public boolean moveToScore(Slot from, Slot to) {
        if (from.scoring || !to.scoring) return false;
        Color c = slotColor.get(from);
        if (c == null || slotColor.get(to) != null) return false;
        slotColor.put(from, null);
        slotColor.put(to, c);
        return true;
    }

    public Slot nextScoreSlotFromPlan() {
        Color wanted = scoreQueue.peekFirst();
        if (wanted == null) return null;

        for (Slot s : new Slot[]{Slot.S1, Slot.S2, Slot.S3}) {
            if (!isEmpty(s) && slotColor.get(s) == wanted) {
                scoreQueue.removeFirst();
                return s;
            }
        }
        return null;
    }

    public Color popScoredBall(Slot s) {
        Color c = slotColor.get(s);
        slotColor.put(s, null);
        return c;
    }

    public String debug() {
        return "C[" + slotColor.get(Slot.C1) + "," +
                slotColor.get(Slot.C2) + "," +
                slotColor.get(Slot.C3) + "] S[" +
                slotColor.get(Slot.S1) + "," +
                slotColor.get(Slot.S2) + "," +
                slotColor.get(Slot.S3) + "]";
    }
    public static final double[] SLOTURI_COLECTARE = new double[]{
            Ruleta.SLOT_C1,
            Ruleta.SLOT_C2,
            Ruleta.SLOT_C3
    };
    public void colectareSafe(){
        setPoz(SAFE);
    }
}
