package pathplanner.ui;


import java.awt.Dimension;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import pathplanner.PlannerResult;
import pathplanner.common.Scenario;


public class ResultWindow extends JFrame implements KeyListener {

    private static final long serialVersionUID = 1L;

    // int scale = 20;
    final ResultSurface surface;
    final DataPanel dataPanel;
    public final PlannerResult result;
    NumberFormat              formatter        = new DecimalFormat("#0.00");

    public ResultWindow(Scenario scenario, PlannerResult result) {

        this.result = result;
        JPanel mainPanel = new JPanel();
        mainPanel.setLayout(new BoxLayout(mainPanel, BoxLayout.Y_AXIS));
        NumberFormat formatter = new DecimalFormat("#0.00");
        JPanel slider;
        if (result.solution.score != 0) {
            surface = new ResultSurface(scenario, result, this);
            double deltaT = result.solution.time[1] - result.solution.time[0];
            slider = new ControlsPanel(result.solution.maxTime, deltaT, surface, this);
            if(result.failed){
                setTitle("FAILED - Partial solution found - Score: " + formatter.format(result.solution.score * deltaT) + "s");
            }else{
                setTitle("SUCCESS - Solution found - Score: " + formatter.format(result.solution.score * deltaT) + "s");
            }
            
        } else {
            surface = new ResultSurface(scenario, result, this);
            slider = new ControlsPanel(1, 1, surface, this);
            setTitle("FAILED - No solution found");

        }
        double scale = surface.calculateScale(scenario.world);
        int width = (int) Math.ceil(scale * scenario.world.getMaxPos().x) + 1;
        int height = (int) Math.ceil(scale * scenario.world.getMaxPos().y) + 1;
        surface.setPreferredSize(new Dimension(width, height));
        mainPanel.add(surface);
        mainPanel.add(slider);
        
        dataPanel = new DataPanel(this);
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
        if(dataPanel != null) dataPanel.update(getTimeIndex(currentTime));
        surface.repaint();
    }
    
    public int getTimeIndex(double time) {
        for (int i = 1; i < result.solution.time.length; i++) {
            if (result.solution.time[i] > time) return i - 1;
        }
        return result.solution.timeSteps - 1;
    }

}











