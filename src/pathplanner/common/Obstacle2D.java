package pathplanner.common;

import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class Obstacle2D implements Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = -1195749444314566799L;
    private final List<Vector2D> vertices;
    public final Rectangle2D boundingBox;
    public final Shape shape;
    
    public Obstacle2D(List<Vector2D> vertices){
        this.vertices = GeometryToolbox.quickHull(new ArrayList<Vector2D>(vertices));
        this.boundingBox = generateBoundingBox();
        this.shape = generateShape();
        
    }
    
    public Obstacle2D(Vector2D bottomLeftCorner, Vector2D topRightCorner){
        this(Arrays.asList(bottomLeftCorner, new Vector2D(topRightCorner.x, bottomLeftCorner.y), topRightCorner, new Vector2D(bottomLeftCorner.x, topRightCorner.y)));
    }
    
    public Obstacle2D(List<Vector2D> vertices, double height){
        this(vertices);
    }
    
    public Obstacle2D(double height, Vector2D... vertices){
        this(Arrays.asList(vertices));
    }
    
    public Obstacle2D(Vector2D... vertices){
        this(0, vertices);
    }
    
    public Obstacle2D(Vector2D p1, Vector2D p2, double height){
        this(p1, p2);
    }
    
    private Rectangle2D generateBoundingBox(){
        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = - Double.MAX_VALUE;
        double maxY = - Double.MAX_VALUE;
        for(Vector2D pos : vertices){
            if(pos.x < minX) minX = pos.x;
            if(pos.y < minY) minY = pos.y;
            if(pos.x > maxX) maxX = pos.x;
            if(pos.y > maxY) maxY = pos.y;
        }
        
        double width = maxX - minX;
        double height = maxY - minY;
        return new Rectangle2D.Double(minX, minY, width, height);
    }
    
    private Shape generateShape(){
        Path2D section =  new Path2D.Double();
        section.moveTo(vertices.get(0).x, vertices.get(0).y);
        for(int i = 1; i < vertices.size(); i++){
            section.lineTo(vertices.get(i).x, vertices.get(i).y);
        }
        section.lineTo(vertices.get(0).x, vertices.get(0).y);
        section.closePath();
        return section;
    }
    
    public boolean intersects(Vector2D p1, Vector2D p2, double buffer){
        Line2D other = new Line2D.Double(p1.x, p1.y, p2.x, p2.y);
        for(int i = 0; i < vertices.size(); i++){
            Vector2D v1 = vertices.get(i);
            Vector2D v2 = vertices.get((i + 1) % vertices.size());
            Line2D vertex = new Line2D.Double(v1.x, v1.y, v2.x, v2.y);
            if( vertex.intersectsLine(other)) return true;
        }
        return false;

    }
    
    public boolean boundingBoxOverlaps(Rectangle2D other){
        if(other.getMinX() > boundingBox.getMaxX()) return false;
        if(other.getMaxX() < boundingBox.getMinX()) return false;
        if(other.getMinY() > boundingBox.getMaxY()) return false;
        if(other.getMaxY() < boundingBox.getMinY()) return false;
        return true;
    }
    
    public boolean contains(Vector2D pos){
        return shape.contains(pos.x, pos.y);
    }
    
    public boolean fuzzyContains(Vector2D pos, double size){
//        return GeometryToolbox.overlapsObstacle(vertices, GeometryToolbox.approximateCircle(pos, size, 8));
        Rectangle2D other = new Rectangle2D.Double(pos.x - size, pos.y - size, 2*size, 2*size);
        return shape.intersects(other);
    }    
    public List<Vector2D> getVertices(){
        return new ArrayList<Vector2D>(vertices);
    }
    
}
