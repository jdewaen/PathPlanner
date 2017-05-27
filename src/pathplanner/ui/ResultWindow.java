package pathplanner.ui;


import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.geom.Rectangle2D;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.Timer;
import javax.swing.border.EmptyBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import pathplanner.PlannerResult;
import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.Solution;
import pathplanner.common.StatisticsTracker;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;
import pathplanner.milpplanner.RegularLine;
import pathplanner.milpplanner.VerticalLine;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;
import pathplanner.preprocessor.PathNode.PathNodeType;
import pathplanner.preprocessor.PathSegment;
import pathplanner.preprocessor.boundssolver.BoundsSolverDebugData;


public class ResultWindow extends JFrame implements KeyListener {

    private static final long serialVersionUID = 1L;

    // int scale = 20;
    final Surface surface;
    final DataPanel dataPanel;
    public final PlannerResult result;

    public ResultWindow(Scenario scenario, PlannerResult result) {

        this.result = result;
        dataPanel = new DataPanel(this);
        JPanel mainPanel = new JPanel();
        mainPanel.setLayout(new BoxLayout(mainPanel, BoxLayout.Y_AXIS));
        NumberFormat formatter = new DecimalFormat("#0.00");
        JPanel slider;
        if (result.solution.score != 0) {
            surface = new Surface(scenario, result, this);
            double deltaT = result.solution.time[1] - result.solution.time[0];
            slider = new ControlsPanel(result.solution.maxTime, deltaT, surface, this);
            setTitle(formatter.format(StatisticsTracker.toSeconds(result.stats.totalTime)) + " score: "
                    + String.valueOf(result.solution.score * deltaT));
        } else {
            surface = new Surface(scenario, result, this);
            slider = new ControlsPanel(1, 1, surface, this);
            setTitle("No solution found.");

        }
        double scale = surface.calculateScale(scenario.world);
        int width = (int) Math.ceil(scale * scenario.world.getMaxPos().x) + 1;
        int height = (int) Math.ceil(scale * scenario.world.getMaxPos().y) + 1;
        surface.setPreferredSize(new Dimension(width, height));
        mainPanel.add(surface);
        mainPanel.add(slider);
        
        dataPanel.setPreferredSize(new Dimension(DataPanel.preferredWidth, mainPanel.getHeight()));
        
        JPanel containerPanel = new JPanel();
        containerPanel.setLayout(new BoxLayout(containerPanel, BoxLayout.X_AXIS));
        
        containerPanel.add(mainPanel);
        containerPanel.add(dataPanel);
        
        add(containerPanel);
        pack();
//        slider.setMaximumSize(slider.getPreferredSize());
        setLocationRelativeTo(null);

        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        this.addKeyListener(this);
        setFocusable(true);
        setFocusTraversalKeysEnabled(false);

    }

    @Override
    public void keyPressed(KeyEvent e) {

    }

    @Override
    public void keyReleased(KeyEvent e) {
        switch(e.getKeyCode()){
            case 82: // r
                surface.resetScale();
                
                break;
        }

    }

    @Override
    public void keyTyped(KeyEvent e) {

    }
    
    public void update(double currentTime){
        surface.time = currentTime;
        dataPanel.update(getTimeIndex(currentTime));
        surface.repaint();
    }
    
    public int getTimeIndex(double time) {
        for (int i = 1; i < result.solution.time.length; i++) {
            if (result.solution.time[i] > time) return i - 1;
        }
        return result.solution.timeSteps - 1;
    }

}




class Surface extends JPanel {

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
    private final Surface     surface          = this;
    static int                maxWidth         = 1000;
    static int                maxHeight        = 700;
    int activeRegionVertexSize = 3;
    final ResultWindow window;

    public Surface(Scenario scenario, PlannerResult result, ResultWindow window) {
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
        
        
        
        /***** GENETIC SEED *****/
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
        
        /***** OBSTACLES *****/

        for (Obstacle2DB obs : world.getObstacles()) {
            g2d.setStroke(new BasicStroke(1));
            g2d.setPaint(Color.blue);
            if (sol != null && sol.activeObstacles[timeIndex] != null && sol.activeObstacles[timeIndex].contains(obs)) {

                for (int i = 0; i < obs.getVertices().size(); i++) {
                    g2d.setPaint(Color.MAGENTA);
                    if (sol.slackVars.containsKey(obs)
                            && sol.slackVars.get(obs).containsKey(timeIndex)) {
                        if (sol.slackVars.get(obs).get(timeIndex).get(i)) {
                            g2d.setPaint(Color.RED);
                        } else {
                            g2d.setPaint(Color.YELLOW);
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
        
        
        /***** SEGMENT TRANSITIONS *****/
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

        g2d.setPaint(Color.green);
        for (Pos2D point : PathSegment.toPositions(result.pathSegments)) {
            double size = 8;
            g2d.fillOval((int) Math.round(offset.x + point.x * scale - size),
                    (int) Math.round(offset.y + point.y * scale - size),
                    (int) Math.round(size * 2), (int) Math.round(size * 2));
        }

        
        /***** THETA* PATH *****/
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
                }   
                double size = 4;
                g2d.fillOval((int) Math.round(offset.x + node.pos.x * scale - size),
                        (int) Math.round(offset.y + node.pos.y * scale - size),
                        (int) Math.round(size * 2), (int) Math.round(size * 2));
            }
            
        }
 
        
        /***** FINISH DATA *****/
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
        
        
        
        /***** VEHICLE TRAJECTORY *****/
        
        g2d.setStroke(new BasicStroke(1));
        if (sol != null) {
            g2d.setPaint(Color.black);
            for (int t = 0; t < sol.timeSteps; t++) {
                if (sol.fin[t]) break;
                if (sol.time[t] > time) break;
                Pos2D pos = sol.pos[t];
                
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


class DataPanel extends JPanel {

    /**
     * 
     */
    private static final long serialVersionUID = 1L;
    JSlider                   slider;
    JLabel                    timeLabel;
    JLabel                    timeStepLabel;
    JLabel                    segmentLabel;
    JLabel                    posLabel;
    JLabel                    velLabel;
    JLabel                    accLabel;
    JLabel                    jerkLabel;
    JLabel                    mouseLabel;
    NumberFormat              formatter        = new DecimalFormat("#0.00");
    ResultWindow window;
    public static final int preferredWidth = 200;

    public DataPanel(ResultWindow window) {
        super(true);
        this.window = window;
        this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
        this.setAlignmentY(TOP_ALIGNMENT);
        this.setAlignmentX(LEFT_ALIGNMENT);

        
        // ROW 1
        
        JPanel row1 = new JPanel();
        row1.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        row1.setLayout(new BoxLayout(row1, BoxLayout.X_AXIS));
        
        JPanel row1Left = new JPanel();
        row1Left.setLayout(new BoxLayout(row1Left, BoxLayout.Y_AXIS));
        
        JPanel row1Middle = new JPanel();
        row1Middle.setLayout(new BoxLayout(row1Middle, BoxLayout.Y_AXIS));
        
        JPanel row1Right = new JPanel();
        row1Right.setLayout(new BoxLayout(row1Right, BoxLayout.Y_AXIS));


        
        row1Left.add(new JLabel("Time"));
        timeLabel = new JLabel();
        row1Left.add(timeLabel);
        
        row1Middle.add(new JLabel("Timestep"));
        timeStepLabel = new JLabel();
        row1Middle.add(timeStepLabel);
        
        row1Right.add(new JLabel("Segment"));
        segmentLabel = new JLabel();
        row1Right.add(segmentLabel);
        
        row1.add(row1Left);
        row1.add(Box.createHorizontalGlue());
        row1.add(row1Middle);
        row1.add(Box.createHorizontalGlue());
        row1.add(row1Right);      
        add(row1);
        

        // ROW 2
        
        JPanel row2 = new JPanel();
        row2.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        row2.setLayout(new BoxLayout(row2, BoxLayout.X_AXIS));
        
        JPanel row2Left = new JPanel();
        row2Left.setLayout(new BoxLayout(row2Left, BoxLayout.Y_AXIS));
        
        JPanel row2Right = new JPanel();
        row2Right.setLayout(new BoxLayout(row2Right, BoxLayout.Y_AXIS));


        
        row2Left.add(new JLabel("Position"));
        posLabel = new JLabel();
        row2Left.add(posLabel);
        
        row2Right.add(new JLabel("Velocity"));
        velLabel = new JLabel();
        row2Right.add(velLabel);
        
        row2.add(row2Left);
        row2.add(Box.createHorizontalGlue());
        row2.add(row2Right);        
        add(row2);
        
        // ROW 3
        
        JPanel row3 = new JPanel();
        row3.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        row3.setLayout(new BoxLayout(row3, BoxLayout.X_AXIS));
        
        JPanel row3Left = new JPanel();
        row3Left.setLayout(new BoxLayout(row3Left, BoxLayout.Y_AXIS));
        
        JPanel row3Right = new JPanel();
        row3Right.setLayout(new BoxLayout(row3Right, BoxLayout.Y_AXIS));


        
        row3Left.add(new JLabel("Acceleration"));
        accLabel = new JLabel();
        row3Left.add(accLabel);
        
        row3Right.add(new JLabel("Jerk"));
        jerkLabel = new JLabel();
        row3Right.add(jerkLabel);
        
        row3.add(row3Left);
        row3.add(Box.createHorizontalGlue());
        row3.add(row3Right);        
        add(row3);
        
        // ROW 4
        
        JPanel row4 = new JPanel();
        row4.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        row4.setLayout(new BoxLayout(row4, BoxLayout.X_AXIS));
        
        JPanel row4Left = new JPanel();
        row4Left.setLayout(new BoxLayout(row4Left, BoxLayout.Y_AXIS));
        
        JPanel row4Right = new JPanel();
        row4Right.setLayout(new BoxLayout(row4Right, BoxLayout.Y_AXIS));


        
        row4Left.add(new JLabel("Mouse"));
        mouseLabel = new JLabel();
        row4Left.add(mouseLabel);
//        
//        row3Right.add(new JLabel("Jerk"));
//        jerkLabel = new JLabel();
//        row3Right.add(jerkLabel);
        
        row4.add(row4Left);
        row4.add(Box.createHorizontalGlue());
//        row3.add(row3Right);        
        add(row4);
        
        if( window.result.solution == null) return;
        update(window.getTimeIndex(window.result.solution.maxTime));
        
    }

    public void update(int timeIndex) {
        if( window.result.solution == null) return;
        timeLabel.setText(formatter.format(window.result.solution.time[timeIndex]));
        timeStepLabel.setText(String.valueOf(timeIndex));
        segmentLabel.setText(String.valueOf(window.result.solution.segment[timeIndex]));
        if(window.surface != null) mouseLabel.setText(window.surface.mousePt.toPrettyString());
        if(!window.result.solution.nosol[timeIndex]){
            posLabel.setText(window.result.solution.pos[timeIndex].toPrettyString());
            velLabel.setText(window.result.solution.vel[timeIndex].toPrettyString());
            accLabel.setText(window.result.solution.acc[timeIndex].toPrettyString());
            jerkLabel.setText(window.result.solution.jerk[timeIndex].toPrettyString());
        }

    }
}


class ControlsPanel extends JPanel {

    /**
     * 
     */
    private static final long serialVersionUID = 1L;
    JSlider                   slider;
    JLabel                    currentTimeLabel;
    NumberFormat              formatter        = new DecimalFormat("#0.00");
    public Surface            surface;
    public double             deltaTime;
    public Timer              timer;
    ResultWindow window;

    public ControlsPanel(double maxTime, double deltaTime, Surface surface, ResultWindow window) {
        super(true);

        this.surface = surface;
        this.deltaTime = deltaTime;
        this.window = window;
        timer = new Timer((int) (deltaTime * 1000), new AnimationListener(this));

        this.setLayout(new BorderLayout());
        setBorder(new EmptyBorder(10, 10, 10, 10));
        slider = new JSlider(JSlider.HORIZONTAL, 0,
                (int) Math.ceil(maxTime) * 100, (int) Math.ceil(maxTime) * 100);
        slider.setBorder(new EmptyBorder(0, 10, 0, 10));
        slider.setMajorTickSpacing(100);
        slider.setPaintTicks(true);

        currentTimeLabel = new JLabel();
        currentTimeLabel.setHorizontalAlignment(JLabel.LEFT);

        JButton playButton = new JButton("PLAY");
        playButton.addActionListener(new PlayButtonListener(this, playButton));

        add(slider, BorderLayout.CENTER);
        add(currentTimeLabel, BorderLayout.WEST);
        add(playButton, BorderLayout.EAST);
        slider.addChangeListener(new SliderListener(surface, this, formatter));

        Dimension d1 = currentTimeLabel.getPreferredSize();
        d1.width = 50;
        currentTimeLabel.setPreferredSize(d1);

        Dimension d2 = playButton.getPreferredSize();
        d2.width = 100;
        playButton.setPreferredSize(d2);

        updateWithSlider(maxTime);

    }

    public void update(double currentTime) {

        currentTimeLabel.setText(formatter.format(currentTime));
        window.update(currentTime);
//        surface.time = currentTime;
//        surface.repaint();
    }

    public void updateWithSlider(double currentTime) {
        slider.setValue((int) Math.ceil(currentTime * 100));
        update(currentTime);

    }
}




class SliderListener implements ChangeListener {

    final Surface surface;
    ControlsPanel panel;
    NumberFormat  formatter;

    public SliderListener(Surface surface, ControlsPanel panel,
            NumberFormat formatter) {
        this.surface = surface;
        this.panel = panel;
        this.formatter = formatter;
    }

    public void stateChanged(ChangeEvent e) {
        JSlider source = (JSlider) e.getSource();
        int time = (int) source.getValue();
        double timeDouble = ((double) time) / 100;
        panel.update(timeDouble);
    }
}




class PlayButtonListener implements ActionListener {

    ControlsPanel panel;
    JButton       button;

    public PlayButtonListener(ControlsPanel panel, JButton button) {
        this.panel = panel;
        this.button = button;
    }

    @Override
    public void actionPerformed(ActionEvent event) {
        if (panel.timer.isRunning()) {
            panel.timer.stop();
            button.setText("PLAY");
        } else {
            panel.timer.start();
            button.setText("PAUSE");
        }

    }

}




class AnimationListener implements ActionListener {

    ControlsPanel panel;

    public AnimationListener(ControlsPanel panel) {
        this.panel = panel;
    }

    @Override
    public void actionPerformed(ActionEvent event) {
        double nextTime = panel.surface.time + panel.deltaTime;
        if (nextTime > panel.surface.result.solution.maxTime) {
            nextTime = 0;
        }
        panel.updateWithSlider(nextTime);

    }

}
