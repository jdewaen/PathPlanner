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

}
