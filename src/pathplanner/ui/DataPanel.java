package pathplanner.ui;

import java.awt.event.ItemEvent;
import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;

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
    JLabel                    segmentTimeLabel;
    JLabel                    totalTimeLabel;
    JLabel                    segmentObstacles;
    JLabel                    segmentEdges;
    
    JCheckBox showObsCheck;
    JCheckBox showObsColsCheck;
    JCheckBox showActiveCheck;
    JCheckBox showInitActiveCheck;
    JCheckBox showInitPath;
    JCheckBox showSegGoals;
    JCheckBox showSegTransitions;
    JCheckBox showGoalConditions;
    JCheckBox showVehicleTrace;
    JCheckBox showVehicle;
    
    NumberFormat              formatter        = new DecimalFormat("#0.00");
    ResultWindow window;
    public static final int preferredWidth = 300;

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
        
        // ROW 5
        
        JPanel row5 = new JPanel();
        row5.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        row5.setLayout(new BoxLayout(row5, BoxLayout.X_AXIS));
        
        JPanel row5Left = new JPanel();
        row5Left.setLayout(new BoxLayout(row5Left, BoxLayout.Y_AXIS));
        
        JPanel row5Right = new JPanel();
        row5Right.setLayout(new BoxLayout(row5Right, BoxLayout.Y_AXIS));


        
        row5Left.add(new JLabel("Segment Solve Time"));
        segmentTimeLabel = new JLabel();
        row5Left.add(segmentTimeLabel);
        
        row5Right.add(new JLabel("Total Solve Time"));
        totalTimeLabel = new JLabel();
        row5Right.add(totalTimeLabel);
        
        row5.add(row5Left);
        row5.add(Box.createHorizontalGlue());
        row5.add(row5Right);        
        add(row5);
        
        // ROW 6
        
        JPanel row6 = new JPanel();
        row6.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        row6.setLayout(new BoxLayout(row6, BoxLayout.X_AXIS));
        
        JPanel row6Left = new JPanel();
        row6Left.setLayout(new BoxLayout(row6Left, BoxLayout.Y_AXIS));
        
        JPanel row6Right = new JPanel();
        row6Right.setLayout(new BoxLayout(row6Right, BoxLayout.Y_AXIS));


        
        row6Left.add(new JLabel("Segment # Obs"));
        segmentObstacles = new JLabel();
        row6Left.add(segmentObstacles);
        
        row6Right.add(new JLabel("Segment # Edges"));
        segmentEdges = new JLabel();
        row6Right.add(segmentEdges);
        
        row6.add(row6Left);
        row6.add(Box.createHorizontalGlue());
        row6.add(row6Right);        
        add(row6);    
        
        
        // OPTIONS
        
        JPanel rowOp = new JPanel();
        rowOp.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        rowOp.setLayout(new BoxLayout(rowOp, BoxLayout.X_AXIS));
        
        JPanel rowOpLeft = new JPanel();
        rowOpLeft.setLayout(new BoxLayout(rowOpLeft, BoxLayout.Y_AXIS));
        
        JPanel rowOpRight = new JPanel();
        rowOpRight.setLayout(new BoxLayout(rowOpRight, BoxLayout.Y_AXIS));


        
        showObsCheck = new JCheckBox("Obstacles");
        showObsColsCheck = new JCheckBox("Obs. Edge Colors");
        showInitPath = new JCheckBox("Initial Path");
        showSegTransitions = new JCheckBox("Seg. Transitions");
        showVehicleTrace = new JCheckBox("UAV Trace");
        
        rowOpLeft.add(showObsCheck);
        rowOpLeft.add(showObsColsCheck);
        rowOpLeft.add(showInitPath);
        rowOpLeft.add(showSegTransitions);
        rowOpLeft.add(showVehicleTrace);
        
        showObsCheck.addItemListener(e -> setToggleValue(e, window.surface.showObs));
        showObsColsCheck.addItemListener(e -> setToggleValue(e, window.surface.showColorsObs));
        showInitPath.addItemListener(e -> setToggleValue(e, window.surface.showInitPath));
        showSegTransitions.addItemListener(e -> setToggleValue(e, window.surface.showSegTrans));
        showVehicleTrace.addItemListener(e -> setToggleValue(e, window.surface.showVehicleTrace));
        
        showObsCheck.setSelected(window.surface.showObs.value);
        showObsColsCheck.setSelected(window.surface.showColorsObs.value);
        showInitPath.setSelected(window.surface.showInitPath.value);
        showSegTransitions.setSelected(window.surface.showSegTrans.value);
        showVehicleTrace.setSelected(window.surface.showVehicleTrace.value);
        
        
        showActiveCheck = new JCheckBox("Active Region");
        showInitActiveCheck = new JCheckBox("Init Act. Region");
        showSegGoals = new JCheckBox("Segment Goals");
        showGoalConditions = new JCheckBox("Goal Conditions");
        showVehicle = new JCheckBox("UAV");
        
        rowOpRight.add(showActiveCheck);
        rowOpRight.add(showInitActiveCheck);
        rowOpRight.add(showSegGoals);
        rowOpRight.add(showGoalConditions);
        rowOpRight.add(showVehicle);
        
        showActiveCheck.addItemListener(e -> setToggleValue(e, window.surface.showActiveRegion));
        showInitActiveCheck.addItemListener(e -> setToggleValue(e, window.surface.showInitActiveRegion));
        showSegGoals.addItemListener(e -> setToggleValue(e, window.surface.showSegGoals));
        showGoalConditions.addItemListener(e -> setToggleValue(e, window.surface.showGoalCond));
        showVehicle.addItemListener(e -> setToggleValue(e, window.surface.showVehicle));
        
        showActiveCheck.setSelected(window.surface.showActiveRegion.value);
        showInitActiveCheck.setSelected(window.surface.showInitActiveRegion.value);
        showSegGoals.setSelected(window.surface.showSegGoals.value);
        showGoalConditions.setSelected(window.surface.showGoalCond.value);
        showVehicle.setSelected(window.surface.showVehicle.value);
        
        
        rowOp.add(rowOpLeft);
        rowOp.add(Box.createHorizontalGlue());
        rowOp.add(rowOpRight);        
        add(rowOp);  
        
        
        if( window.result.solution == null) return;
        update(window.getTimeIndex(window.result.solution.maxTime));
        
    }
    
    private void setToggleValue(ItemEvent e, BooleanRef ref){
        ref.value = (e.getStateChange() == ItemEvent.SELECTED);
        window.surface.repaint();
    }

    public void update(int timeIndex) {
        if( window.result.solution == null) return;
        int segment = window.result.solution.segment[timeIndex];
        timeLabel.setText(formatter.format(window.result.solution.time[timeIndex]) + "s");
        timeStepLabel.setText(String.valueOf(timeIndex));
        segmentLabel.setText(String.valueOf(segment));
        totalTimeLabel.setText(formatter.format(((double) window.result.stats.totalTime)/1000) + "s");
        if(window.surface != null) mouseLabel.setText(window.surface.mousePt.toPrettyString());
        if(!window.result.solution.nosol[timeIndex]){
            posLabel.setText(window.result.solution.pos[timeIndex].toPrettyString());
            velLabel.setText(window.result.solution.vel[timeIndex].toPrettyString());
            accLabel.setText(window.result.solution.acc[timeIndex].toPrettyString());
            jerkLabel.setText(window.result.solution.jerk[timeIndex].toPrettyString());
            segmentTimeLabel.setText(formatter.format(((double) window.result.stats.solveTime.get(segment))/1000) + "s");
            segmentObstacles.setText(String.valueOf(window.result.scenarioSegments.get(segment).obstacleCount()));
            segmentEdges.setText(String.valueOf(window.result.scenarioSegments.get(segment).edgeCount()));
        }

    }
}