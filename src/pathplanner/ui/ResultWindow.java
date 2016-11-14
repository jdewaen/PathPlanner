package pathplanner.ui;

import javax.swing.JFrame;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Formatter;
import java.util.Locale;

import javax.swing.JPanel;

import pathplanner.common.Region2D;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario2D;
import pathplanner.common.Solution;


public class ResultWindow extends JFrame{
    int scale = 20;
    public ResultWindow(Solution sol, Scenario2D scen, double totalTime){
        final Surface surface = new Surface(sol, scen, scale);
        add(surface);
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

    public Surface(Solution sol, Scenario2D scen, int scale) {
        this.sol = sol;
        this.scen = scen;
        this.scale = scale;
        repaint();
    }

    private void doDrawing(Graphics g) {

        int shapeSize = scale / 2;
        Graphics2D g2d = (Graphics2D) g;

        g2d.setPaint(Color.blue);

        int w = getWidth();
        int h = getHeight();
        
        for( Region2D obs : scen.world.getRegions()){
            double width = obs.bottomRightCorner.x - obs.topLeftCorner.x;
            double height = obs.bottomRightCorner.y - obs.topLeftCorner.y;

            g2d.drawRect((int) obs.topLeftCorner.x * scale, (int) obs.topLeftCorner.y * scale, (int) width * scale, (int) height * scale);    
        }
        

        
        g2d.setPaint(Color.black);
        
        for( int t = 0; t < scen.timeSteps; t++){
            if(sol.fin[t]) break;
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