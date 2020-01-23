package frc.robot.drivers;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
