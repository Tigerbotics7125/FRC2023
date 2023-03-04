/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.lib;

import java.util.Stack;

public class RollingAverage {
    Stack<Double> mValues;
    int mMaxSize;

    public RollingAverage(int size) {
        mMaxSize = size;
    }

    public double getAverage() {
        while (mValues.size() > mMaxSize) {
            mValues.pop();
        }
        return mValues.stream().mapToDouble(Number::doubleValue).average().orElse(0.0);
    }

    public void add(double value) {
        mValues.add(value);
    }

    public double calculate(double value) {
        mValues.add(value);
        return getAverage();
    }
}
