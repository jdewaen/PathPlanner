package pathplanner.common;

import java.io.Serializable;
import java.text.DecimalFormat;
import java.text.NumberFormat;


public class Vector2D implements Serializable{
    
    /**
     * 
     */
    private static final long serialVersionUID = 5335500275102298280L;
    /**
     * 
     */
    public final double x;
    public final double y;
    public static final NumberFormat formatter = new DecimalFormat("#0.00");
    public static final double EPSILON = 0.0001;
    
    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }
    
    public String toString(){
        return "[" + String.valueOf(x) + ", " + String.valueOf(y) + "]";
    }
    
    public String toPrettyString(){
        return "(" + formatter.format(x) + ", " + formatter.format(y) + ")";
    }
    
    public double distanceFrom(Vector2D other){
        return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2));
    }
    
    public double length(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
    
    public boolean equals(Object o){
        if(!(o instanceof Vector2D)) return false;
        Vector2D other = (Vector2D) o;
        double x1 = x - x % EPSILON;
        double y1 = y - y % EPSILON;
        double x2 = other.x - other.x % EPSILON;
        double y2 = other.y - other.y % EPSILON;
        
        return ( Double.compare(x1, x2) == 0 && Double.compare(y1, y2) == 0 );
    }
    
    public boolean fuzzyEquals(Vector2D other, double delta){
        return this.distanceFrom(other) < delta;

    }
    
    public Vector2D minus(Vector2D other){
        return new Vector2D(x - other.x, y - other.y);
    }
    public Vector2D plus(Vector2D other){
        return new Vector2D(x + other.x, y + other.y);
    }
    
    public Vector2D middleBetween(Vector2D other){
        return new Vector2D((x + other.x)/2, (y + other.y)/2);
    }
    
    public Vector2D normalize(){
        double length = length();
        return new Vector2D(x / length, y / length);
    }
    
    public Vector2D multiply(double m){
        return new Vector2D(x*m, y*m);
    }
    
    public int hashCode(){
        return (String.valueOf(x) + String.valueOf(y)).hashCode();
    }
    
    public double dotProduct(Vector2D other){
        return x*other.x + y * other.y;
    }
}
