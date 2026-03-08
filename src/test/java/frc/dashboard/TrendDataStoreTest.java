package frc.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class TrendDataStoreTest {

    @Test
    void addAndReadBackSamples() {
        TrendDataStore store = new TrendDataStore(5);
        store.add(1.0, 12.5);
        store.add(2.0, 12.3);
        store.add(3.0, 12.1);

        assertEquals(3, store.size());
        assertEquals(1.0, store.timestamp(0));
        assertEquals(12.5, store.value(0));
        assertEquals(3.0, store.timestamp(2));
        assertEquals(12.1, store.value(2));
    }

    @Test
    void ringBufferOverwritesOldestSamples() {
        TrendDataStore store = new TrendDataStore(3);
        store.add(1.0, 10.0);
        store.add(2.0, 20.0);
        store.add(3.0, 30.0);
        store.add(4.0, 40.0); // overwrites (1.0, 10.0)

        assertEquals(3, store.size());
        // Oldest should now be (2.0, 20.0)
        assertEquals(2.0, store.timestamp(0));
        assertEquals(20.0, store.value(0));
        // Newest
        assertEquals(4.0, store.timestamp(2));
        assertEquals(40.0, store.value(2));
    }

    @Test
    void clearResetsStore() {
        TrendDataStore store = new TrendDataStore(10);
        store.add(1.0, 5.0);
        store.add(2.0, 6.0);
        store.clear();

        assertEquals(0, store.size());
    }

    @Test
    void indexOutOfBoundsThrows() {
        TrendDataStore store = new TrendDataStore(5);
        store.add(1.0, 10.0);
        assertThrows(IndexOutOfBoundsException.class, () -> store.value(1));
        assertThrows(IndexOutOfBoundsException.class, () -> store.value(-1));
    }

    @Test
    void capacityReturnsConfiguredValue() {
        TrendDataStore store = new TrendDataStore(42);
        assertEquals(42, store.capacity());
    }

    @Test
    void constructorRejectsZeroCapacity() {
        assertThrows(IllegalArgumentException.class, () -> new TrendDataStore(0));
    }
}
