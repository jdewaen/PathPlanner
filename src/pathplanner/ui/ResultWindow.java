package pathplanner.ui;

import javax.swing.JFrame;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Formatter;
import java.util.Locale;

import javax.swing.JPanel;

import pathplanner.common.Obstacle2D;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario2D;
import pathplanner.common.Solution;


public class ResultWindow extends JFrame{
    public ResultWindow(Solution sol, Scenario2D scen, double totalTime){
        final Surface surface = new Surface(sol, scen);
        add(surface);
        StringBuilder sb = new StringBuilder();
        Formatter formatter = new Formatter(sb, Locale.US);
        formatter.format("%.3f%ns", totalTime);
        setTitle(sb.toString());
        setSize(400, 440);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);    
        }

}


class Surface extends JPanel {
    Solution sol;
    Scenario2D scen;

    public Surface(Solution sol, Scenario2D scen) {
        this.sol = sol;
        this.scen = scen;
        repaint();
    }

    private void doDrawing(Graphics g) {

        int scale = 40;
        int shapeSize = 20;
        Graphics2D g2d = (Graphics2D) g;

        g2d.setPaint(Color.blue);

        int w = getWidth();
        int h = getHeight();
        
        for( Obstacle2D obs : scen.world.getObstacles()){
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