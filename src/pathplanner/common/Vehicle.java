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
}
