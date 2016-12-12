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
    
    public double getAccTime(){
        return maxSpeed / acceleration;
    }
    
    public double getAccDist(){
        return acceleration *  Math.pow(getAccTime(), 2) / 2;

    }
    
    public double getMaxSpeedFromDistance(double dist){
        double time = Math.sqrt(2 * dist / acceleration);
        double maxSpeed = acceleration * time;
        return maxSpeed;
    }
}
