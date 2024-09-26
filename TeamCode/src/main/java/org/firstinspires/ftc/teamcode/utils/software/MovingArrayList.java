package org.firstinspires.ftc.teamcode.utils.software;

import org.apache.commons.math3.stat.StatUtils;

import java.util.ArrayList;
import java.util.Collections;

public class MovingArrayList {

    private ArrayList<Double> arrayList;
    private int size;

    public MovingArrayList(int size) {
        if (size <= 0) {
            throw new IllegalArgumentException("Size must be greater than zero");
        }
        this.size = size;
        this.arrayList = new ArrayList<>(size);
    }

    public synchronized void add(double value) {
        if (arrayList.size() == size) {
            arrayList.remove(0); // Remove the oldest element if the size exceeds the limit
        }
        arrayList.add(value);
    }

    public ArrayList<Double> getArrayList() {
        return new ArrayList<>(arrayList); // Return a copy to prevent external modifications
    }

    public synchronized double getMean() {
        if(size > 5 && arrayList.size() > (size -5)) {
            removeMinMax();
        }

        double[] array = arrayList.stream()
                .mapToDouble(Double::doubleValue)
                .toArray();

        return StatUtils.mean(array);
    }

    public synchronized double getAvg() {
        if(arrayList.size() > 3) {
            removeMinMax();
        }

        return arrayList.stream()
                .mapToDouble(Double::doubleValue).average().orElse(0.0);
    }

    public void removeMinMax() {
        if (arrayList == null || arrayList.size() < 3) {
            throw new IllegalArgumentException("List must have at least 3 elements");
        }

        // Find the indices of the min and max values
        int minIndex = arrayList.indexOf(Collections.min(arrayList));
        int maxIndex = arrayList.indexOf(Collections.max(arrayList));

        // Remove the min and max values
        arrayList.remove(minIndex);
        arrayList.remove(maxIndex > minIndex ? maxIndex - 1 : maxIndex); // Adjust index if max was after min
    }

    public int size() {
        return arrayList.size();
    }

    public boolean isEmpty() {
        return arrayList.isEmpty();
    }

    public void clear() {
        arrayList.clear();
    }
}
