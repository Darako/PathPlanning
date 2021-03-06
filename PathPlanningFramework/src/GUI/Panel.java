package GUI;

import PathPlanning.Map;
import java.util.ArrayList;
import javax.swing.JOptionPane;
import javax.swing.JSlider;

/**
 *
 * @author Pablo Muñoz
 */
public class Panel extends javax.swing.JFrame {

    /**
     * Creates new form Panel
     */
    public Panel(Map map, ArrayList path, int zoom, String data) {
        initComponents();
        info = data;
        this.setTitle(map.toString());
        sc = new java.awt.ScrollPane(java.awt.ScrollPane.SCROLLBARS_ALWAYS);
        lienzo = new Lienzo(map, path, zoom);
        lienzo.setBounds(0, 0, lienzo.getHeight(), lienzo.getWidth());
        lienzo.showAltitude(checkAlts.isSelected());
        lienzo.showCost(checkCosts.isSelected());
        this.add(sc);
        sc.add(lienzo);
        if(zoom < sliderZoom.getMinimum() || zoom > sliderZoom.getMaximum())
            sliderZoom.setValue(sliderZoom.getMinimum());
        else
            sliderZoom.setValue(zoom); // Call event handler
        changeLienzo();
    }
    
    public Panel() { // Need empty constructor for offline application   
    }

    /**
     * Repaint the map after GUI configuration changed.
     */
    private void changeLienzo()
    {
        sc.setSize(lienzo.getWidth(), lienzo.getHeight());
        sc.setBounds(0, 25, this.getWidth()-15, this.getHeight()-60);
        sc.repaint();
    }
    
    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        sliderZoom = new javax.swing.JSlider();
        labelZoom = new javax.swing.JLabel();
        labelAux = new javax.swing.JLabel();
        checkAlts = new javax.swing.JCheckBox();
        checkCosts = new javax.swing.JCheckBox();
        labelShow = new javax.swing.JLabel();
        buttonRefresh = new javax.swing.JButton();
        checkImage = new javax.swing.JCheckBox();
        buttonInfo = new javax.swing.JButton();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setTitle("Path-Planning");
        setBackground(new java.awt.Color(204, 204, 204));
        setCursor(new java.awt.Cursor(java.awt.Cursor.DEFAULT_CURSOR));
        setMinimumSize(new java.awt.Dimension(800, 800));
        setName("Form");
        setPreferredSize(new java.awt.Dimension(800, 800));
        getContentPane().setLayout(null);

        sliderZoom.setMajorTickSpacing(10);
        sliderZoom.setMinimum(1);
        sliderZoom.setMinorTickSpacing(1);
        sliderZoom.setToolTipText("");
        sliderZoom.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                sliderZoomStateChanged(evt);
            }
        });
        getContentPane().add(sliderZoom);
        sliderZoom.setBounds(50, 0, 200, 23);

        labelZoom.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        labelZoom.setText("Zoom");
        getContentPane().add(labelZoom);
        labelZoom.setBounds(10, 0, 40, 20);
        getContentPane().add(labelAux);
        labelAux.setBounds(260, 0, 50, 20);

        checkAlts.setSelected(true);
        checkAlts.setText("altitude");
        getContentPane().add(checkAlts);
        checkAlts.setBounds(360, 0, 80, 23);

        checkCosts.setText("costs");
        getContentPane().add(checkCosts);
        checkCosts.setBounds(440, 0, 60, 23);

        labelShow.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        labelShow.setHorizontalAlignment(javax.swing.SwingConstants.RIGHT);
        labelShow.setText("Show:");
        getContentPane().add(labelShow);
        labelShow.setBounds(300, 0, 50, 20);

        buttonRefresh.setText("Refresh");
        buttonRefresh.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonRefreshActionPerformed(evt);
            }
        });
        getContentPane().add(buttonRefresh);
        buttonRefresh.setBounds(610, 0, 90, 23);

        checkImage.setSelected(true);
        checkImage.setText("image mode");
        getContentPane().add(checkImage);
        checkImage.setBounds(510, 0, 100, 23);

        buttonInfo.setText("Info");
        buttonInfo.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonInfoActionPerformed(evt);
            }
        });
        getContentPane().add(buttonInfo);
        buttonInfo.setBounds(700, 0, 60, 23);

        getAccessibleContext().setAccessibleDescription("");

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void sliderZoomStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_sliderZoomStateChanged
       JSlider js = (JSlider)evt.getSource();
       labelAux.setText(String.valueOf(js.getValue()));
       lienzo.changeScale(js.getValue());
       lienzo.setBounds(0, 0, lienzo.getHeight(), lienzo.getWidth());
       changeLienzo();
    }//GEN-LAST:event_sliderZoomStateChanged

    private void buttonRefreshActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_buttonRefreshActionPerformed
        boolean aux = checkImage.isSelected();
        lienzo.changeVisualization(aux);
        checkAlts.setEnabled(!aux);
        checkCosts.setEnabled(!aux);
        if(!aux)
        {
            lienzo.showAltitude(checkAlts.isSelected());
            lienzo.showCost(checkCosts.isSelected());
        }
        changeLienzo();
    }//GEN-LAST:event_buttonRefreshActionPerformed

    private void buttonInfoActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_buttonInfoActionPerformed
        JOptionPane.showMessageDialog(this, info, "Info", JOptionPane.INFORMATION_MESSAGE);        
    }//GEN-LAST:event_buttonInfoActionPerformed

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /*
         * Create and display the form
         */
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                new Panel().setVisible(true);
            }
        });
    }
    
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JButton buttonInfo;
    private javax.swing.JButton buttonRefresh;
    private javax.swing.JCheckBox checkAlts;
    private javax.swing.JCheckBox checkCosts;
    private javax.swing.JCheckBox checkImage;
    private javax.swing.JLabel labelAux;
    private javax.swing.JLabel labelShow;
    private javax.swing.JLabel labelZoom;
    private javax.swing.JSlider sliderZoom;
    // End of variables declaration//GEN-END:variables
    private java.awt.ScrollPane sc;
    private Lienzo lienzo;
    private String info;
}
