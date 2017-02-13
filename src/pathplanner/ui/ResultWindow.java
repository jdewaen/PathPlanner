package pathplanner.ui;


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

import pathplanner.common.Pos2D;
import pathplanner.common.Region2D;
import pathplanner.common.Scenario;
import pathplanner.common.Solution;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.Node;


public class ResultWindow extends JFrame implements KeyListener {

    private static final long serialVersionUID = 1L;

    // int scale = 20;
    final Surface surface;
    final DataPanel dataPanel;
    public final Solution sol;

    public ResultWindow(Solution sol, Scenario scenario, double totalTime,
            List<Node> prePath, List<Pos2D> preCheckpoints,
            List<CornerEvent> corners) {

        this.sol = sol;
        dataPanel = new DataPanel(this);
        JPanel mainPanel = new JPanel();
        mainPanel.setLayout(new BoxLayout(mainPanel, BoxLayout.Y_AXIS));
        NumberFormat formatter = new DecimalFormat("#0.00");
        JPanel slider;
        if (sol.score != 0) {
            surface = new Surface(sol, scenario, prePath, preCheckpoints,
                    corners, this);
            double deltaT = sol.time[1] - sol.time[0];
            slider = new ControlsPanel(sol.maxTime, deltaT, surface, this);
            setTitle(formatter.format(totalTime) + " score: "
                    + String.valueOf(sol.score * deltaT));
        } else {
            surface = new Surface(null, scenario, prePath, preCheckpoints,
                    corners, this);
            slider = new ControlsPanel(1, 1, surface, this);
            setTitle("No solution found.");

        }
        double scale = surface.calculateScale(scenario.world);
        int width = (int) Math.ceil(scale * scenario.world.getMaxPos().x) + 1;
        int height = (int) Math.ceil(scale * scenario.world.getMaxPos().y) + 1;
        surface.setPreferredSize(new Dimension(width, height));
        mainPanel.add(surface);
        mainPanel.add(slider);
        
        dataPanel.setPreferredSize(new Dimension(dataPanel.preferredWidth, mainPanel.getHeight()));
        
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
        // TODO Auto-generated method stub

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
        // TODO Auto-generated method stub

    }
    
    public void update(double currentTime){
        surface.time = currentTime;
        dataPanel.update(getTimeIndex(currentTime));
        surface.repaint();
    }
    
    public int getTimeIndex(double time) {
        for (int i = 1; i < sol.time.length; i++) {
            if (sol.time[i] > time) return i - 1;
        }
        return sol.timeSteps - 1;
    }

}




class Surface extends JPanel {

    /**
     * 
     */
    private static final long serialVersionUID = 1L;
    Solution                  sol;
    Scenario                  scenario;
    List<Node>                prePath;
    List<Pos2D>               preCheckpoints;
    List<CornerEvent>         corners;
    double                    scale;
    double                    time;
    Pos2D                     offset;
    private Point             mousePt;
    private final Surface     surface          = this;
    static int                maxWidth         = 1000;
    static int                maxHeight        = 700;
    final ResultWindow window;

    public Surface(Solution sol, Scenario scenario, List<Node> prePath,
            List<Pos2D> preCheckpoints, List<CornerEvent> corners, ResultWindow window) {
        this.sol = sol;
        this.scenario = scenario;
        if (sol != null) {
            this.time = sol.maxTime;
        } else {
            this.time = 0;
        }
        this.prePath = prePath;
        this.preCheckpoints = preCheckpoints;
        this.corners = corners;
        this.offset = new Pos2D(0, 0);
        this.scale = calculateScale(scenario.world);
        this.window = window;
        repaint();

        MouseAdapter adapt = new MouseAdapter() {

            @Override
            public void mouseClicked(MouseEvent e) {
                double x = (e.getX() - offset.x) / scale;
                double y = (getHeight() - e.getY() - offset.y) / scale;
                System.out.println(String.valueOf(x) + " " + String.valueOf(y));
            }

            @Override
            public void mousePressed(MouseEvent e) {
                mousePt = e.getPoint();
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
                repaint();
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                int dx = e.getX() - mousePt.x;
                int dy = e.getY() - mousePt.y;
                offset = offset.plus(new Pos2D(dx, -dy));
                mousePt = e.getPoint();
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

        Graphics2D g2d = (Graphics2D) g;
        g2d.translate(0, getHeight());
        g2d.scale(1.0, -1.0);
        // g2d.drawRect(0, 0, (int) world.getMaxPos().x * scale, (int) world.getMaxPos().y * scale);

        int timeIndex = window.getTimeIndex(time);

        if (sol != null) {
            double width = sol.activeArea[timeIndex].topLeftCorner.x
                    - sol.activeArea[timeIndex].bottomRightCorner.x;
            double height = sol.activeArea[timeIndex].topLeftCorner.y
                    - sol.activeArea[timeIndex].bottomRightCorner.y;
            g2d.setPaint(Color.lightGray);
            g2d.fillRect(
                    (int) (offset.x + sol.activeArea[timeIndex].bottomRightCorner.x
                            * scale),
                    (int) (offset.y + sol.activeArea[timeIndex].bottomRightCorner.y
                            * scale),
                    (int) (width * scale), (int) (height * scale));
        }

        for (Region2D obs : world.getRegions()) {
            if (obs.isCheckPoint()) {
                g2d.setPaint(Color.green);
            } else {
                if (sol.activeObstacles[timeIndex].contains(obs)) {
                    g2d.setPaint(Color.red);
                } else {
                    g2d.setPaint(Color.blue);
                }
            }

            if (time < obs.startTime || time > obs.endTime) continue;

            double width = obs.topLeftCorner.x - obs.bottomRightCorner.x;
            double height = obs.topLeftCorner.y - obs.bottomRightCorner.y;

            g2d.drawRect((int) (offset.x + obs.bottomRightCorner.x * scale),
                    (int) (offset.y + obs.bottomRightCorner.y * scale),
                    (int) (width * scale), (int) (height * scale));
        }

        if (sol != null) {
            g2d.setPaint(Color.cyan);
            for (Pos2D point : sol.highlightPoints) {
                double size = 6;
                g2d.fillOval(
                        (int) Math.round(offset.x + point.x * scale - size),
                        (int) Math.round(offset.y + point.y * scale - size),
                        (int) Math.round(size * 2), (int) Math.round(size * 2));
            }
        }
        g2d.setPaint(Color.green);
        for (Pos2D point : preCheckpoints) {
            double size = 6;
            g2d.fillOval((int) Math.round(offset.x + point.x * scale - size),
                    (int) Math.round(offset.y + point.y * scale - size),
                    (int) Math.round(size * 2), (int) Math.round(size * 2));
        }

        // Collections.sort(corners);
        for (Node node : prePath) {
            g2d.setPaint(Color.darkGray);
            for (int i = 0; i < corners.size(); i++) {
                if (node.cost >= corners.get(i).start.cost
                        && node.cost <= corners.get(i).end.cost) {
                    g2d.setPaint(Color.red);
                }
            }
            Pos2D point = node.pos;
            double size = 2;
            g2d.fillOval((int) Math.round(offset.x + point.x * scale - size),
                    (int) Math.round(offset.y + point.y * scale - size),
                    (int) Math.round(size * 2), (int) Math.round(size * 2));
        }

        if (sol != null) {
            g2d.setPaint(Color.black);
            for (int t = 0; t < sol.timeSteps; t++) {
                if (sol.nosol[t]) break;
                if (sol.fin[t]) break;
                if (sol.time[t] > time) break;
                Pos2D pos = sol.pos[t];
                g2d.drawOval(
                        (int) Math.round(offset.x + (pos.x - vehicle.size)
                                * scale),
                        (int) Math.round(offset.y + (pos.y - vehicle.size)
                                * scale),
                        (int) Math.round(vehicle.size * 2 * scale),
                        (int) Math.round(vehicle.size * 2 * scale));
            }
        }

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
    JLabel                    horiThrotLabel;
    JLabel                    vertThrotLabel;
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


        
        row3Left.add(new JLabel("Hori throttle"));
        horiThrotLabel = new JLabel();
        row3Left.add(horiThrotLabel);
        
        row3Right.add(new JLabel("Vert throttle"));
        vertThrotLabel = new JLabel();
        row3Right.add(vertThrotLabel);
        
        row3.add(row3Left);
        row3.add(Box.createHorizontalGlue());
        row3.add(row3Right);        
        add(row3);
        
        
        update(window.getTimeIndex(window.sol.maxTime));
        
    }

    public void update(int timeIndex) {
        timeLabel.setText(formatter.format(window.sol.time[timeIndex]));
        timeStepLabel.setText(String.valueOf(timeIndex));
        segmentLabel.setText(String.valueOf(window.sol.segment[timeIndex]));
        posLabel.setText(window.sol.pos[timeIndex].toPrettyString());
        velLabel.setText(window.sol.vel[timeIndex].toPrettyString());
        horiThrotLabel.setText(formatter.format(window.sol.horiThrottle[timeIndex]));
        vertThrotLabel.setText(formatter.format(window.sol.vertThrottle[timeIndex]));


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
        if (nextTime > panel.surface.sol.maxTime) {
            nextTime = 0;
        }
        panel.updateWithSlider(nextTime);

    }

}
