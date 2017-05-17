package pathplanner.common;


public class Vehicle {
    public final double acceleration;
    public final double maxSpeed;
    public final double size;
    public final double minSpeed;
    public final double maxJerk;
    
    
    public Vehicle(double acceleration, double minSpeed, double maxSpeed, double size){
        this(acceleration, minSpeed, maxSpeed, size, Double.NaN);
    }
    public Vehicle(double acceleration, double minSpeed, double maxSpeed, double size, double maxJerk){
        this.acceleration = acceleration;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.size = size;
        this.maxJerk = maxJerk;
    }
    
    public double getAccTime(){
        return getAccTime(maxSpeed);
    }
    
    public double getAccTime(double speed){
        return speed / acceleration;
    }
    
    public double getAccDist(){
        return getAccDist(maxSpeed);

    }
    
    public double getAccDist(double speed){
        return acceleration *  Math.pow(getAccTime(speed), 2) / 2;
    }
    
    public double getMaxSpeedFromDistance(double dist){
        double time = Math.sqrt(2 * dist / acceleration);
        double maxSpeed = acceleration * time;
        return maxSpeed;
    }
    
    public boolean hasMaxJerk(){
        return Double.isFinite(maxJerk);
    }
}
