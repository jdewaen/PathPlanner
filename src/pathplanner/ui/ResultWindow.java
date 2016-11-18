package pathplanner.ui;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JSlider;
import javax.swing.JTextField;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Locale;

import javax.swing.JPanel;
import javax.swing.border.EmptyBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.Timer;

import pathplanner.common.CheckPoint2D;
import pathplanner.common.Region2D;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario2D;
import pathplanner.common.Solution;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;


public class ResultWindow extends JFrame{
    int scale = 20;
    public ResultWindow(Solution sol, World2D world, Vehicle vehicle, double totalTime){
        
        double deltaT = sol.time[1] - sol.time[0];
        JPanel mainPanel = new JPanel();
        mainPanel.setLayout(new BoxLayout(mainPanel, BoxLayout.Y_AXIS));
        
        final Surface surface = new Surface(sol, world, vehicle, scale);
        
        JPanel slider = new ControlsPanel(sol.maxTime, deltaT, surface);
        int width = (int) Math.ceil(scale * world.getMaxPos().x) + 1;
        int height = (int) Math.ceil(scale * world.getMaxPos().y) + 1;
        surface.setPreferredSize(new Dimension(width, height));
        mainPanel.add(surface);
        mainPanel.add(slider);
        add(mainPanel);
        pack();
        StringBuilder time = new StringBuilder();
        NumberFormat formatter = new DecimalFormat("#0.00");  
        setTitle(formatter.format(totalTime) + " score: " + String.valueOf(sol.score * deltaT));
        int xSize = (int) world.getMaxPos().x;
        int ySize = (int) world.getMaxPos().y;
//        setSize(width, height + 100);
        pack();
        setLocationRelativeTo(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);    
        }

}


class Surface extends JPanel {
    Solution sol;
    World2D world;
    Vehicle vehicle;
    int scale;
    double time;

    public Surface(Solution sol, World2D world, Vehicle vehicle, int scale) {
        this.sol = sol;
        this.world = world;
        this.scale = scale;
        this.vehicle = vehicle;
        this.time = sol.maxTime;
        
        repaint();
        this.addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
               System.out.println(e);
            }
         });
    }

    private void doDrawing(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;


        int w = getWidth();
        int h = getHeight();
        
        g2d.drawRect(0, 0, (int) world.getMaxPos().x * scale, (int)  world.getMaxPos().y * scale);    

        
        for( Region2D obs : world.getRegions()){            
            if(obs.isCheckPoint()){
                g2d.setPaint(Color.green);
            }else{
                g2d.setPaint(Color.blue);
            }
            
            if(time < obs.startTime || time > obs.endTime) continue;
            
            double width = obs.topLeftCorner.x - obs.bottomRightCorner.x;
            double height = obs.topLeftCorner.y - obs.bottomRightCorner.y;

            g2d.drawRect((int) obs.bottomRightCorner.x * scale, (int) obs.bottomRightCorner.y * scale, (int) width * scale, (int) height * scale);    
        }
        
        g2d.setPaint(Color.green);
        for( Pos2D point : sol.highlightPoints){
            g2d.fillOval((int) Math.round((point.x - vehicle.size) * scale), (int) Math.round((point.y - vehicle.size) * scale),
                    (int) Math.round(vehicle.size * 2 * scale),(int) Math.round(vehicle.size * 2 * scale));
        }

        
        g2d.setPaint(Color.black);
        
        for( int t = 0; t < sol.timeSteps; t++){
            if(sol.fin[t]) break;
            if(sol.time[t] > time) break;
            Pos2D pos = sol.pos[t];
            g2d.drawOval((int) Math.round((pos.x - vehicle.size) * scale), (int) Math.round((pos.y - vehicle.size) * scale),
                    (int) Math.round(vehicle.size * 2 * scale),(int) Math.round(vehicle.size * 2 * scale));
        }
        

    }

    @Override
    public void paintComponent(Graphics g) {

        super.paintComponent(g);
        doDrawing(g);
    }
}

class ControlsPanel extends JPanel {
    JSlider slider;
    JLabel currentTimeLabel;
    NumberFormat formatter = new DecimalFormat("#0.00");  
    public Surface surface;
    public double deltaTime;
    public Timer timer;
    public ControlsPanel(double maxTime, double deltaTime, Surface surface) {
        super(true);

        this.surface = surface;
        this.deltaTime = deltaTime;
        timer = new Timer((int) (deltaTime * 1000), new AnimationListener(this));

        
        this.setLayout(new BorderLayout());
        setBorder(new EmptyBorder(10, 10, 10, 10));
        slider = new JSlider(JSlider.HORIZONTAL, 0, (int) Math.ceil(maxTime)*100, (int) Math.ceil(maxTime)*100);
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
    
    public void update(double currentTime){
        
        currentTimeLabel.setText(formatter.format(currentTime));
        surface.time = currentTime;
        surface.repaint(); 
    }
    
    public void updateWithSlider(double currentTime){
        slider.setValue((int) Math.ceil(currentTime*100));
        update(currentTime);

    }
  }


class SliderListener implements ChangeListener {
    final Surface surface;
    ControlsPanel panel;
    NumberFormat formatter;
    public SliderListener(Surface surface, ControlsPanel panel, NumberFormat formatter){
        this.surface = surface;
        this.panel = panel;
        this.formatter = formatter;
    }
    public void stateChanged(ChangeEvent e) {
        JSlider source = (JSlider)e.getSource();
            int time = (int) source.getValue();
            double timeDouble = ((double) time) / 100;
            panel.update(timeDouble);
    }
}

class PlayButtonListener implements ActionListener {
    
    ControlsPanel panel;
    JButton button;
    
    public PlayButtonListener(ControlsPanel panel, JButton button) {
        this.panel = panel;
        this.button = button;
    }
    @Override
    public void actionPerformed(ActionEvent event) {
        if(panel.timer.isRunning()){
            panel.timer.stop();
            button.setText("PLAY");
        }else{
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
        if( nextTime > panel.surface.sol.maxTime){
            nextTime = 0;
        }
        panel.updateWithSlider(nextTime);
        
    }
    
}
