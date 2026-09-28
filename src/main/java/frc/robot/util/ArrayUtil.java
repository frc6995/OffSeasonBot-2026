package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;

/**
 * Small array helpers used by the IO layers to batch status signals.
 *
 * <p>{@link BaseStatusSignal#refreshAll} takes varargs, and IO layers keep per-motor signals in
 * arrays so they can be indexed alongside the inputs arrays they populate.
 */
public final class ArrayUtil {
    private ArrayUtil() {}

    /**
     * Flattens several signal arrays into one. The result is a fresh array every call, so hold the
     * result in a field rather than calling this from a hot loop where it would allocate at 50 Hz.
     */
    public static BaseStatusSignal[] concat(BaseStatusSignal[]... arrays) {
        int length = 0;
        for (BaseStatusSignal[] array : arrays) {
            length += array.length;
        }

        BaseStatusSignal[] combined = new BaseStatusSignal[length];
        int offset = 0;
        for (BaseStatusSignal[] array : arrays) {
            System.arraycopy(array, 0, combined, offset, array.length);
            offset += array.length;
        }
        return combined;
    }
}
