package pathplanner.common;

import java.io.Serializable;
import java.text.DecimalFormat;
import java.text.NumberFormat;


public class Pos2D implements Serializable{
    
    /**
     * 
     */
    private static final long serialVersionUID = 4832617970975980395L;
    public final double x;
    public final double y;
    public static final NumberFormat formatter = new DecimalFormat("#0.00");
    
    public Pos2D(double x, double y){
        this.x = x;
        this.y = y;
    }
    
    public String toString(){
        return "[" + String.valueOf(x) + ", " + String.valueOf(y) + "]";
    }
    
    public String toPrettyString(){
        return "(" + formatter.format(x) + ", " + formatter.format(y) + ")";
    }
    
    public double distanceFrom(Pos2D other){
        return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2));
    }
    
    public double length(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
    
    public boolean equals(Object o){
        if(!(o instanceof Pos2D)) return false;
        Pos2D other = (Pos2D) o;
        
        return (other.x == x && other.y == y);
    }
    
    public boolean fuzzyEquals(Pos2D other, double delta){
        return this.distanceFrom(other) < delta;

    }
    
    public Pos2D minus(Pos2D other){
        return new Pos2D(x - other.x, y - other.y);
    }
    public Pos2D plus(Pos2D other){
        return new Pos2D(x + other.x, y + other.y);
    }
    
    public Pos2D middleBetween(Pos2D other){
        return new Pos2D((x + other.x)/2, (y + other.y)/2);
    }
    
    public Pos2D normalize(){
        double length = length();
        return new Pos2D(x / length, y / length);
    }
    
    public Pos2D multiply(double m){
        return new Pos2D(x*m, y*m);
    }
}
