package frc.robot.driver;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
