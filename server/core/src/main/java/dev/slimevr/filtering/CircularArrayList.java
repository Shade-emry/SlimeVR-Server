package dev.slimevr.filtering;

import java.util.*;

/**
 * A circular buffer implementation using ArrayList for efficient random access.
 * This class is thread-unsafe; external synchronization is required for concurrent access.
 */
public class CircularArrayList<E> extends AbstractList<E> implements RandomAccess {

    private final int capacity; // Maximum number of elements the buffer can hold
    private final List<E> buffer; // Underlying storage
    private int head = 0; // Index of the first element
    private int tail = 0; // Index after the last element

    public CircularArrayList(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("Capacity must be positive");
        }
        this.capacity = capacity + 1; // Extra slot for circular behavior
        this.buffer = new ArrayList<>(Collections.nCopies(this.capacity, null));
    }

    public int capacity() {
        return capacity - 1;
    }

    private int wrapIndex(int index) {
        // Ensure index is within bounds using modulo arithmetic
        return (index % capacity + capacity) % capacity;
    }

    @Override
    public int size() {
        return (tail - head + capacity) % capacity;
    }

    @Override
    public E get(int index) {
        if (index < 0 || index >= size()) {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size());
        }
        return buffer.get(wrapIndex(head + index));
    }

    public E getLatest() {
        if (size() == 0) {
            throw new NoSuchElementException("Buffer is empty");
        }
        return buffer.get(wrapIndex(tail - 1));
    }

    @Override
    public E set(int index, E element) {
        if (index < 0 || index >= size()) {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size());
        }
        return buffer.set(wrapIndex(head + index), element);
    }

    @Override
    public void add(int index, E element) {
        if (index < 0 || index > size()) {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size());
        }
        if (size() == capacity - 1) {
            throw new IllegalStateException("Buffer is full");
        }

        // Shift elements to make space for the new element
        if (index < size()) {
            for (int i = size(); i > index; i--) {
                buffer.set(wrapIndex(head + i), get(i - 1));
            }
        }

        // Insert the new element
        buffer.set(wrapIndex(head + index), element);
        tail = wrapIndex(tail + 1);
    }

    @Override
    public E remove(int index) {
        if (index < 0 || index >= size()) {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size());
        }

        E removedElement = get(index);

        // Shift elements to fill the gap
        if (index > 0) {
            for (int i = index; i < size() - 1; i++) {
                set(i, get(i + 1));
            }
        }

        tail = wrapIndex(tail - 1);
        return removedElement;
    }

    public void clear() {
        head = 0;
        tail = 0;
        Collections.fill(buffer, null);
    }
}
