package pathplanner.common;


public class Vehicle {
    public final double acceleration;
    public final double maxSpeed;
    public final double size;
    public final double minSpeed;
    
    public Vehicle(double acceleration, double minSpeed, double maxSpeed, double size){
        this.acceleration = acceleration;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.size = size;
    }
    
}
