package pathplanner.ui;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JSlider;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.Formatter;
import java.util.Locale;

import javax.swing.JPanel;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import pathplanner.common.CheckPoint2D;
import pathplanner.common.Region2D;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario2D;
import pathplanner.common.Solution;


public class ResultWindow extends JFrame{
    int scale = 40;
    public ResultWindow(Solution sol, Scenario2D scen, double totalTime){
        JPanel mainPanel = new JPanel();
        mainPanel.setLayout(new BoxLayout(mainPanel, BoxLayout.Y_AXIS));
        
        final Surface surface = new Surface(sol, scen, scale);
        
        JPanel slider = new SwingSliderExample(scen, surface);
        surface.setPreferredSize(new Dimension(1200, 800));
        mainPanel.add(surface);
        mainPanel.add(slider);
        add(mainPanel);
        StringBuilder time = new StringBuilder();
        Formatter formatter = new Formatter(time, Locale.US);
        formatter.format("%.3f%ns", totalTime);
        setTitle(time.toString() + " score: " + String.valueOf(sol.score * scen.deltaT));
        int xSize = (int) scen.world.getMaxPos().x;
        int ySize = (int) scen.world.getMaxPos().y;
        setSize((xSize + 1) * scale, (ySize + 2) * scale);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);    
        }

}


class Surface extends JPanel {
    Solution sol;
    Scenario2D scen;
    int scale;
    double time;

    public Surface(Solution sol, Scenario2D scen, int scale) {
        this.sol = sol;
        this.scen = scen;
        this.scale = scale;
        this.time = scen.maxTime;
        
        repaint();
        this.addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
               System.out.println(e);
            }
         });
    }

    private void doDrawing(Graphics g) {

        int shapeSize = scale / 2;
        Graphics2D g2d = (Graphics2D) g;


        int w = getWidth();
        int h = getHeight();
        
        for( Region2D obs : scen.world.getRegions()){            
            if(obs.isCheckPoint()){
                g2d.setPaint(Color.green);
            }else{
                g2d.setPaint(Color.blue);
            }
            
            if(time < obs.startTime || time > obs.endTime) continue;
            
            double width = obs.bottomRightCorner.x - obs.topLeftCorner.x;
            double height = obs.bottomRightCorner.y - obs.topLeftCorner.y;

            g2d.drawRect((int) obs.topLeftCorner.x * scale, (int) obs.topLeftCorner.y * scale, (int) width * scale, (int) height * scale);    
        }
        

        
        g2d.setPaint(Color.black);
        
        for( int t = 0; t < scen.timeSteps; t++){
            if(sol.fin[t]) break;
            if(t * scen.deltaT > time) break;
            Pos2D pos = sol.pos[t];
            g2d.drawOval((int) (pos.x * scale) - shapeSize / 2,(int) (pos.y * scale) - shapeSize / 2, shapeSize, shapeSize);
        }

        g2d.setPaint(Color.green);
        g2d.drawOval((int) scen.startPos.x * scale - shapeSize / 2,(int) scen.startPos.y * scale - shapeSize / 2, shapeSize, shapeSize);
        
        g2d.setPaint(Color.red);
        g2d.drawOval((int) scen.goal.x * scale - shapeSize / 2,(int) scen.goal.y * scale - shapeSize / 2, shapeSize, shapeSize);

    }

    @Override
    public void paintComponent(Graphics g) {

        super.paintComponent(g);
        doDrawing(g);
    }
}

class SwingSliderExample extends JPanel {

    public SwingSliderExample(Scenario2D scen, Surface surface) {

      super(true);
      this.setLayout(new BorderLayout());
      JSlider slider = new JSlider(JSlider.HORIZONTAL, 0, (int) scen.maxTime*100, (int) scen.maxTime*100);

      slider.setMinorTickSpacing(1);
      slider.setMajorTickSpacing(10);
      slider.setPaintTicks(true);
      slider.setPaintLabels(true);

      // We'll just use the standard numeric labels for now...
      slider.setLabelTable(slider.createStandardLabels(100));

      add(slider, BorderLayout.CENTER);
      slider.addChangeListener(new SliderListener(surface));
    }
  }


class SliderListener implements ChangeListener {
    final Surface surface;
    public SliderListener(Surface surface){
        this.surface = surface;
    }
    public void stateChanged(ChangeEvent e) {
        JSlider source = (JSlider)e.getSource();
            int time = (int)source.getValue();
            surface.time = ((double) time) / 100;
            surface.repaint();
    }
}
