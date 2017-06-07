package pathplanner.ui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.util.List;

import javax.swing.JPanel;

import pathplanner.PlannerResult;
import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.Solution;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;
import pathplanner.preprocessor.PathNode;
import pathplanner.preprocessor.PathSegment;
import pathplanner.preprocessor.PathNode.PathNodeType;
import pathplanner.preprocessor.boundssolver.BoundsSolverDebugData;


class ResultSurface extends JPanel {

    /**
     * 
     */
    private static final long serialVersionUID = 1L;
    Scenario                  scenario;
    PlannerResult             result;
    double                    scale;
    double                    time;
    Pos2D                     offset;
    public Pos2D             mousePt = new Pos2D(0,0);
    private Point             lastMousePt;
    private final ResultSurface     surface          = this;
    static int                maxWidth         = 1000;
    static int                maxHeight        = 700;
    int activeRegionVertexSize = 3;
    final ResultWindow window;
    
    public final BooleanRef showObs = new BooleanRef(true);
    public final BooleanRef showColorsObs = new BooleanRef(true);
    public final BooleanRef showActiveRegion = new BooleanRef(true);
    public final BooleanRef showInitActiveRegion = new BooleanRef(false);
    public final BooleanRef showInitPath = new BooleanRef(true);
    public final BooleanRef showSegGoals = new BooleanRef(true);
    public final BooleanRef showSegTrans = new BooleanRef(true);
    public final BooleanRef showGoalCond = new BooleanRef(false);
    public final BooleanRef showVehicleTrace = new BooleanRef(true);
    public final BooleanRef showVehicle = new BooleanRef(true);

    public ResultSurface(Scenario scenario, PlannerResult result, ResultWindow window) {
        this.result = result;
        this.scenario = scenario;
        if (result.solution != null) {
            this.time = result.solution.maxTime;
        } else {
            this.time = 0;
        }
//        this.offset = new Pos2D(-1011, -2579);
//        this.scale = 5;
        this.offset = new Pos2D(0, 0);
        this.scale = calculateScale(scenario.world);
        this.window = window;
        repaint();

        MouseAdapter adapt = new MouseAdapter() {
            
            @Override
            public void mouseMoved(MouseEvent e){
                double x = (e.getX() - offset.x) / scale;
                double y = (getHeight() - e.getY() - offset.y) / scale;
                mousePt = new Pos2D(x, y);
                window.dataPanel.update(window.getTimeIndex(time));
            }

            @Override
            public void mouseClicked(MouseEvent e) {
                double x = (e.getX() - offset.x) / scale;
                double y = (getHeight() - e.getY() - offset.y) / scale;
                System.out.println(String.valueOf(x) + " " + String.valueOf(y));
            }

            @Override
            public void mousePressed(MouseEvent e) {
                lastMousePt = e.getPoint();
            }

            @Override
            public void mouseWheelMoved(MouseWheelEvent e) {
                double scaleAmount = (1 - 0.1 * e.getPreciseWheelRotation());
                surface.scale *= scaleAmount;
                double newX = e.getX() * (scaleAmount - 1) - scaleAmount
                        * offset.x;
                double newY = (surface.getHeight() - e.getY())
                        * (scaleAmount - 1) - scaleAmount * offset.y;
                // Pos2D mousePos = new Pos2D(e.getX(), e.getY());
                // Pos2D scaledMousePos = new Pos2D(e.getX() * scaleAmount, e.getY() * scaleAmount);
                //
                // Pos2D diff = scaledMousePos.minus(mousePos);
                // offset = offset.plus(new Pos2D(diff.x, diff.y));
                offset = new Pos2D(-newX, -newY);
//                System.out.println(surface.scale);
                repaint();
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                int dx = e.getX() - lastMousePt.x;
                int dy = e.getY() - lastMousePt.y;
                offset = offset.plus(new Pos2D(dx, -dy));
                lastMousePt = e.getPoint();
//                System.out.println(offset);
                repaint();
            }

        };

        this.addMouseListener(adapt);
        this.addMouseMotionListener(adapt);
        this.addMouseWheelListener(adapt);
    }
    

    private void doDrawing(Graphics g) {
        Vehicle vehicle = scenario.vehicle;
        World2D world = scenario.world;
        Solution sol = result.solution;

        Graphics2D g2d = (Graphics2D) g;
        g2d.translate(0, getHeight());
        g2d.scale(1.0, -1.0);
        int timeIndex = window.getTimeIndex(time);
        
        
        /***** ACTIVE REGION *****/
        if(showActiveRegion.value){
            if (sol != null && sol.activeArea != null && sol.activeArea.size() > timeIndex) {
                List<Pos2D> activeArea = sol.activeArea.get(timeIndex);
                g2d.setPaint(Color.lightGray);
                int[] xpts = new int[activeArea.size()];
                int[] ypts = new int[activeArea.size()];
                for(int i = 0; i < activeArea.size(); i++){
                    xpts[i] = (int) (offset.x + activeArea.get(i).x * scale);
                    ypts[i] = (int) (offset.y + activeArea.get(i).y * scale);
                }
                g2d.fillPolygon(xpts, ypts, activeArea.size());
                
                g2d.setColor(Color.darkGray);
                
                for(int i = 0; i < activeArea.size(); i++){
                    
                    xpts[i] = (int) (offset.x + activeArea.get(i).x * scale);
                    ypts[i] = (int) (offset.y + activeArea.get(i).y * scale);
                    g2d.fillOval(
                            xpts[i] - activeRegionVertexSize,
                            ypts[i] - activeRegionVertexSize,
                            (int) Math.round(activeRegionVertexSize * 2), (int) Math.round(activeRegionVertexSize * 2));
                }
            }
        }

        
        
        
        /***** GENETIC SEED *****/
        if(showInitActiveRegion.value){
            if (sol != null && sol.boundsDebugData != null && sol.boundsDebugData.size() > timeIndex) {
                BoundsSolverDebugData boundsDebug = sol.boundsDebugData.get(timeIndex);
                
                List<Pos2D> seed = boundsDebug.requiredPoints;
                g2d.setPaint(Color.orange);
                int[] xpts = new int[seed.size()];
                int[] ypts = new int[seed.size()];
                for(int i = 0; i < seed.size(); i++){
                    xpts[i] = (int) (offset.x + seed.get(i).x * scale);
                    ypts[i] = (int) (offset.y + seed.get(i).y * scale);
                }
                g2d.fillPolygon(xpts, ypts, seed.size());
                
    //            List<Rectangle2D> rects = boundsDebug.requiredRects;
    //            g2d.setPaint(Color.BLUE);
    //            for(Rectangle2D rect : rects){
    //                g2d.drawRect((int) (offset.x + rect.getX()* scale), 
    //                        (int) (offset.y + rect.getY()* scale), 
    //                        (int) (rect.getWidth()* scale), 
    //                        (int) (rect.getHeight()* scale));
    //            }
                
                List<Pos2D> points = boundsDebug.requiredPoints;
                g2d.setPaint(Color.MAGENTA);
                for(Pos2D point : points){
                    double size = 3;
                    g2d.fillOval(
                            (int) Math.round(offset.x + point.x * scale - size),
                            (int) Math.round(offset.y + point.y * scale - size),
                            (int) Math.round(size * 2), (int) Math.round(size * 2));
                }
                
            }
        }
        
        /***** OBSTACLES *****/

        if(showObs.value){
            for (Obstacle2DB obs : world.getObstacles()) {
                g2d.setStroke(new BasicStroke(1));
                g2d.setPaint(Color.blue);
                if (sol != null && sol.activeObstacles[timeIndex] != null && sol.activeObstacles[timeIndex].contains(obs)) {

                    for (int i = 0; i < obs.getVertices().size(); i++) {
                        if(showColorsObs.value){
//                            g2d.setPaint(Color.RED);
                            g2d.setPaint(Color.MAGENTA);
                            if (sol.slackVars.containsKey(obs)
                                    && sol.slackVars.get(obs).containsKey(timeIndex)) {
                                if (sol.slackVars.get(obs).get(timeIndex).get(i)) {
                                    g2d.setPaint(Color.RED);
                                } else {
                                    g2d.setPaint(Color.YELLOW);
                                }
                            }
                        }

                        Pos2D first = obs.getVertices().get(i);
                        Pos2D second = obs.getVertices().get(
                                (i + 1) % obs.getVertices().size());
                        int x1 = (int) (offset.x + first.x * scale);
                        int y1 = (int) (offset.y + first.y * scale);
                        int x2 = (int) (offset.x + second.x * scale);
                        int y2 = (int) (offset.y + second.y * scale);
                        g2d.drawLine(x1, y1, x2, y2);
                    }

                } else {
                    int[] xpts = new int[obs.getVertices().size()];
                    int[] ypts = new int[obs.getVertices().size()];
                    for (int i = 0; i < obs.getVertices().size(); i++) {
                        xpts[i] = (int) (offset.x + obs.getVertices().get(i).x
                                * scale);
                        ypts[i] = (int) (offset.y + obs.getVertices().get(i).y
                                * scale);
                    }
                    g2d.drawPolygon(xpts, ypts, obs.getVertices().size());
                }

            }
        }

        
        
        /***** SEGMENT TRANSITIONS *****/
        if(showSegTrans.value){
            if (sol != null) {
                g2d.setPaint(Color.cyan);
                for (Pos2D point : sol.highlightPoints) {
                    double size = vehicle.size * scale;
                    g2d.fillOval(
                            (int) Math.round(offset.x + point.x * scale - size),
                            (int) Math.round(offset.y + point.y * scale - size),
                            (int) Math.round(size * 2), (int) Math.round(size * 2));
                }
            }
        }

        if(showSegGoals.value){
            g2d.setPaint(Color.green);
            for (Pos2D point : PathSegment.toPositions(result.pathSegments)) {
                double size = 8;
                g2d.fillOval((int) Math.round(offset.x + point.x * scale - size),
                        (int) Math.round(offset.y + point.y * scale - size),
                        (int) Math.round(size * 2), (int) Math.round(size * 2));
            }
        }


        
        /***** THETA* PATH *****/
        if(showInitPath.value){
            PathNode last = null;
            if(result.heuristicPath != null){
                
                /* LINES */
                g2d.setStroke(new BasicStroke(2));
                for (PathNode node : result.heuristicPath.toArrayList()) {
                    if(last != null){            
                        if(node.type == PathNodeType.ESSENTIAL 
                                && last.type == PathNodeType.ESSENTIAL
                                && node.getChild() != null){
                            g2d.setPaint(Color.red);
                        }else{
                            g2d.setPaint(Color.darkGray);
                        }
                        g2d.drawLine((int) Math.round(offset.x + last.pos.x * scale),
                                (int) Math.round(offset.y + last.pos.y * scale),
                                (int) Math.round(offset.x + node.pos.x * scale),
                                (int) Math.round(offset.y + node.pos.y * scale));
                    }
                    last = node;
                }
                
                /* NODES */
                for (PathNode node : result.heuristicPath.toArrayList()) {
                    if(node.type == PathNodeType.ESSENTIAL){
                        g2d.setPaint(Color.red);
                    }else{
                        g2d.setPaint(Color.darkGray);
                        if(!showSegGoals.value) continue;
                    }   
                    double size = 4;
                    g2d.fillOval((int) Math.round(offset.x + node.pos.x * scale - size),
                            (int) Math.round(offset.y + node.pos.y * scale - size),
                            (int) Math.round(size * 2), (int) Math.round(size * 2));
                }
                
            }
        }
 
        
        /***** FINISH DATA *****/
        if(showGoalCond.value){
            if(result.scenarioSegments.size() > sol.segment[timeIndex]) {
                ScenarioSegment seg = result.scenarioSegments.get(sol.segment[timeIndex]);
                double tolerance = seg.positionTolerance;
                if(seg.path != null){
                    g2d.setPaint(Color.blue);
                    double size = 3;
                    g2d.setStroke(new BasicStroke((float) size));
                    Pos2D pos = seg.path.end.pos;
                    Pos2D delta = seg.path.getFinishVector();
                    double length = (tolerance + 2*vehicle.size) * Math.sqrt(2);
                    Pos2D linePos = pos.minus(delta.multiply(vehicle.size));
                    int x1, y1, x2, y2;
                    if(delta.y == 0){
                        x1 = (int) (offset.x + linePos.x * scale);
                        y1 = (int) (offset.y + (linePos.y + tolerance) * scale);
    
                        x2 = (int) (offset.x + linePos.x * scale);
                        y2 = (int) (offset.y + (linePos.y - tolerance) * scale);                
                    }else{
                        Pos2D perp = new Pos2D(delta.y, -delta.x);
                        Pos2D p1 = linePos.minus(perp.multiply(length));
                        Pos2D p2 = linePos.plus(perp.multiply(length));
                        
                        x1 = (int) (offset.x + p1.x * scale);
                        y1 = (int) (offset.y + p1.y * scale);
                        
                        x2 = (int) (offset.x + p2.x * scale);
                        y2 = (int) (offset.y + p2.y * scale);
                    }
                    if(result.planner.cplexConfig.useFinishLine){
                        g2d.drawLine(x1, y1, x2, y2);
                    }
                    
                    
                    if(result.planner != null){
                        g2d.setPaint(Color.GREEN);
                        double rectSize = tolerance + 2 * vehicle.size; 
                        g2d.drawRect((int) (offset.x + (pos.x - rectSize) * scale), 
                                (int) (offset.y + (pos.y - rectSize) * scale), 
                                (int) (rectSize * 2* scale), 
                                (int) (rectSize * 2* scale));
                    }
                }
            }
        }
        
        
        
        /***** VEHICLE TRAJECTORY *****/
        
        g2d.setStroke(new BasicStroke(1));
        if (sol != null) {
            g2d.setPaint(Color.black);
            for (int t = 0; t < sol.timeSteps; t++) {
                if (sol.fin[t]) break;
                if (sol.time[t] > time) break;
                Pos2D pos = sol.pos[t];
                
                if(showVehicle.value){
                    if((t+1 < sol.timeSteps && sol.time[t+1] > time) || t == sol.timeSteps - 1){
                        g2d.setPaint(Color.darkGray);
                        g2d.fillOval(
                                (int) Math.round(offset.x + (pos.x - vehicle.size)
                                        * scale),
                                (int) Math.round(offset.y + (pos.y - vehicle.size)
                                        * scale),
                                (int) Math.round(vehicle.size * 2 * scale),
                                (int) Math.round(vehicle.size * 2 * scale));
                    }
                }

                
                if(showVehicleTrace.value){
                    if(sol.nosol[t]){
                        g2d.setPaint(Color.red);
                    }else{
                        g2d.setPaint(Color.black);
                    }
                    g2d.drawOval(
                            (int) Math.round(offset.x + (pos.x - vehicle.size)
                                    * scale),
                            (int) Math.round(offset.y + (pos.y - vehicle.size)
                                    * scale),
                            (int) Math.round(vehicle.size * 2 * scale),
                            (int) Math.round(vehicle.size * 2 * scale));
                }

                
            }
        }
        
        
        
        
       

        
        g2d.setPaint(Color.BLACK);
        g2d.drawRect((int) offset.x, (int) offset.y, (int) (world.getMaxPos().x * scale), (int) (world.getMaxPos().y * scale));

        // g2d.scale(1.0, -1.0);
        // g2d.translate(0, getHeight());

    }

    @Override
    public void paintComponent(Graphics g) {

        super.paintComponent(g);
        
        doDrawing(g);
    }



    public double calculateScale(World2D world) {
        Pos2D diff = world.getMaxPos().minus(world.getMinPos());
        double xScale = ((double) maxWidth) / diff.x;
        double yScale = ((double) maxHeight) / diff.y;

        return Math.min(xScale, yScale);

    }
    
    public void resetScale(){
        this.scale = calculateScale(scenario.world);
        this.offset = new Pos2D(0, 0);
        repaint();
    }
}