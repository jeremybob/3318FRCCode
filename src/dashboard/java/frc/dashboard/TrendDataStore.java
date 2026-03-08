package frc.dashboard;

/**
 * Fixed-capacity ring-buffer that stores timestamped double samples for trend
 * plotting.  One instance is created per metric (battery voltage, loop period,
 * shooter spin-up latency).  All public methods are called on the Swing EDT so
 * no synchronization is required.
 */
public final class TrendDataStore {

    private final double[] timestamps;
    private final double[] values;
    private final int capacity;
    private int head;      // next write index
    private int count;     // number of valid entries

    public TrendDataStore(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("capacity must be > 0");
        }
        this.capacity = capacity;
        this.timestamps = new double[capacity];
        this.values = new double[capacity];
    }

    /** Append a sample. {@code timestampSec} should be monotonically increasing. */
    public void add(double timestampSec, double value) {
        timestamps[head] = timestampSec;
        values[head] = value;
        head = (head + 1) % capacity;
        if (count < capacity) {
            count++;
        }
    }

    /** Number of stored samples. */
    public int size() {
        return count;
    }

    /** Read the i-th oldest sample timestamp (0 = oldest). */
    public double timestamp(int i) {
        return timestamps[index(i)];
    }

    /** Read the i-th oldest sample value (0 = oldest). */
    public double value(int i) {
        return values[index(i)];
    }

    /** Clear all stored data. */
    public void clear() {
        head = 0;
        count = 0;
    }

    /** The maximum number of samples this store can hold. */
    public int capacity() {
        return capacity;
    }

    private int index(int i) {
        if (i < 0 || i >= count) {
            throw new IndexOutOfBoundsException("index " + i + " out of range [0, " + count + ")");
        }
        return (head - count + i + capacity) % capacity;
    }
}
