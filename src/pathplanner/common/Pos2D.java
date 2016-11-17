package pathplanner.common;


public class Pos2D {
    
    public final double x;
    public final double y;
    
    public Pos2D(double x, double y){
        this.x = x;
        this.y = y;
    }
    
    public String toString(){
        return "[" + String.valueOf(x) + ", " + String.valueOf(y) + "]";
    }
    
    public double distanceFrom(Pos2D other){
        return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2));
    }
    
    public boolean equals(Object o){
        if(!(o instanceof Pos2D)) return false;
        Pos2D other = (Pos2D) o;
        
        return (other.x == x && other.y == y);
    }

}
