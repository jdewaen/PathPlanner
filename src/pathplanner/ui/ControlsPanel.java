package pathplanner.ui;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.Timer;
import javax.swing.border.EmptyBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

class ControlsPanel extends JPanel {

    /**
     * 
     */
    private static final long serialVersionUID = 1L;
    JSlider                   slider;
    JLabel                    currentTimeLabel;
    NumberFormat              formatter        = new DecimalFormat("#0.00");
    public ResultSurface            surface;
    public double             deltaTime;
    public Timer              timer;
    ResultWindow window;

    public ControlsPanel(double maxTime, double deltaTime, ResultSurface surface, ResultWindow window) {
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

    final ResultSurface surface;
    ControlsPanel panel;
    NumberFormat  formatter;

    public SliderListener(ResultSurface surface, ControlsPanel panel,
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