/*
 * Bluetooth Starter Kit Android app. For use with the Microchip Bluetooth 
 * Starter Kit DM320018
 *
 * Copyright (C) 2014  Microchip Technology Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package com.mchp.android.PIC32_BTSK;

import java.util.ArrayList;
import java.util.LinkedList;

import com.jjoe64.graphview.*;
import com.jjoe64.graphview.GraphView.GraphViewData;
import com.mchp.android.PIC32_BTSK.R;

import android.support.v4.app.Fragment;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.TextView;

/*
 * This is the Temperature fragment. It has a graph with recently received temperature values, an interface to 
 * start/stop temperature updates, and an interface to select and send an update rate.
 */
public class TemperatureFragment extends Fragment implements
SeekBar.OnSeekBarChangeListener {
    
    // Constants
    static String timerCmd = "252";             // Command to set the temperature update rate
    static String enableCmd = "253";            // Command to start/stop temperature updates
    static int timerDefault = 1000;             // Default update rate, in ms

    // Frequency spec table, used to define possible update rates settable by the slider bar. 
    // Each row in the table defines a range of values. Given {min,max,incr}, the range would be from 
    // min to max with increments of incr.
    // With this table, the following values are possible:
    // 250, 255, 260, ..., 490, 495ms
    // 500, 525, 550, ..., 950, 975ms
    // 1000, 2000, ..., 7000, 8000ms
    static int[][] freqSpec = {
            // min,  max, incr
            {  250,  500,    5 },
            {  500, 1000,   25 },
            { 1000, 8000, 1000 },
    };

    static int tempMin = 60;                    // Minimum temperature shown on graph's y-axis
    static int tempMax = 100;                   // Maximum temperature shown on graph's y-axis
    static int tempInterval = 5;                // Interval between ticks on graph's y-axis    

    // UI elements
    // Graph and text to show recently received temperature values
    private TextView mLastTemp;
    private GraphView graphView;
    private GraphViewSeries graphViewSeries;
    private GraphViewData graphViewData[];
    private GraphViewStyle graphViewStyle;
    private int tickSize = tempInterval;
    private int minY = tempMin;
    private int maxY = tempMax;
    
    // Interface for sending start/stop command
    private Button mStart;
    
    // Interface for selecting and sending update rate command
    private Button mFrequency;
    private SeekBar mSeekBarFreq;
    private TextView mProgressTextFreq;
    private Button mUpFreq;
    private Button mDownFreq;    
    
    int mFreq = timerDefault;
    int mSeekBarValue;
    
    ArrayList<Integer> freqValues;

    // Callback functions
    // These callbacks allow a fragment to call a function defined in its parent Activity.
    
    // This callback allows this fragment to send a string over Bluetooth, using its parent Activity's Bluetooth 
    // connection. This is used to send the start/stop or timer update commands over Bluetooth, when the 
    // user presses the respective buttons.
    public interface OnSendRequestListener {
        public void onSendRequest(String s);
    }
    OnSendRequestListener mSendRequestCallback;

    // This callback allows this fragment to get the temperature log, which is stored by its parent Activity. This 
    // is used to get the temperature log when the fragment is created.
    public interface OnTempRequestListener {
        public LinkedList<Integer> onTempRequest();
    }
    OnTempRequestListener mTempRequestCallback;

    // Action callbacks
    // These callbacks happen when the user interacts with the UI elements

    // Called when the user changes the seekbar. Gets the new frequency from the seekbar, and updates
    // the other UI elements with the new frequency.
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromTouch) {
        mSeekBarValue = seekBar.getProgress();
        mFreq = freqValues.get(mSeekBarValue);
        mProgressTextFreq.setText(mFreq+" ms");
    }

    // Called when the user starts changing the seekbar
    public void onStartTrackingTouch(SeekBar seekBar) {
    }

    // Called when the user stops changing the seekbar
    public void onStopTrackingTouch(SeekBar seekBar) {
    }
    
    // Lifecycle callbacks
    // These are called by the system when starting, closing, or navigating through the app

    // Called when this fragment is attached to its container activity (when its tab is selected)
    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        
        // This checks that the container activity (BluetoothChat) has implemented
        // the OnSendRequestListener interface. If not, it throws an exception. If it
        // has been implemented, sets mSendRequestCallback to the implementation in the
        // container activity.
        try {
            mSendRequestCallback = (OnSendRequestListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString() + " must implement OnSendRequestListener");
        }

        // This checks that the container activity (BluetoothChat) has implemented
        // the OnTempRequestListener interface. If not, it throws an exception.  If it
        // has been implemented, sets mTempRequestCallback to the implementation in the
        // container activity.
        try {
            mTempRequestCallback = (OnTempRequestListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString() + " must implement OnTempRequestListener");
        }

        return;
    }
    
    // Called when this fragment is created
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);        
        createFreqValues();
    }

    // Called when this fragment View is created. Performs setup of the UI.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, 
        Bundle savedInstanceState) {
        
        super.onCreateView(inflater, container, savedInstanceState);

        // Inflate the layout for this fragment
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_temperature, container, false);

        // Get each UI object from the layout
        mLastTemp = (TextView)rootView.findViewById(R.id.last_temp);
        mStart = (Button)rootView.findViewById(R.id.temp_start);
        mFrequency = (Button)rootView.findViewById(R.id.temp_freq);
        mSeekBarFreq = (SeekBar)rootView.findViewById(R.id.seek_freq);
        mProgressTextFreq = (TextView)rootView.findViewById(R.id.progress_freq);
        mDownFreq = (Button)rootView.findViewById(R.id.freq_down);
        mUpFreq = (Button)rootView.findViewById(R.id.freq_up);

        // Initialize the states of the UI objects
        mSeekBarFreq.setMax(freqValues.size()-1);
        mSeekBarFreq.setProgress(freqValues.indexOf(mFreq));
        
        mLastTemp.setTextSize(100.0f);
        mLastTemp.setText("--\u00b0");
        mStart.setText("Start/Stop");
        mFrequency.setText("Set Timer");
        mProgressTextFreq.setText(mFreq+" ms");        
        mDownFreq.setText("-");
        mUpFreq.setText("+");        

        // Set listeners for the UI objects
        mSeekBarFreq.setOnSeekBarChangeListener(this);

        // Define a click listener for the start/stop button. When clicked, it sends the start/stop command.
        mStart.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                mSendRequestCallback.onSendRequest(enableCmd);
            }
        });
        
        // Define a click listener for the timer button. When clicked, it sends the timer command with the current 
        // frequency.
        mFrequency.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                mSendRequestCallback.onSendRequest(timerCmd+","+mFreq);
            }
        });
        
        // Define click listeners for the fine tune buttons. When clicked, they update the current frequency and 
        // update other UI elements with the new frequency.
        mDownFreq.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                mSeekBarValue=mSeekBarFreq.getProgress();
                mSeekBarValue--;
                mSeekBarFreq.setProgress(mSeekBarValue);
            }
        });
        mUpFreq.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                mSeekBarValue=mSeekBarFreq.getProgress();
                mSeekBarValue++;
                mSeekBarFreq.setProgress(mSeekBarValue);
            }
        });
        
        // Set up the graph
        // Create a new bar graph
        graphView = new BarGraphView(
            this.getActivity().getApplicationContext(), ""
        );

        // Set the graph data to the values in the temperature log
        graphViewData = new GraphViewData[PIC32_BTSK.numTemperatures];
        setGraphViewData(mTempRequestCallback.onTempRequest());
        
        // Add the graph data to the graph.
        graphViewSeries = new GraphViewSeries(graphViewData);
        graphView.addSeries(graphViewSeries);
        
        // Setup the graph axis ranges, zoom and scroll
        graphView.setManualYAxisBounds(maxY,minY);
        graphView.setScalable(true);
        graphView.setScrollable(true);
        graphView.setViewPort(2, 40);

        // Setup the graph axes labels
        graphViewStyle = new GraphViewStyle();
        graphViewStyle.setHorizontalLabelsColor(0);
        graphViewStyle.setNumHorizontalLabels(2);
        graphViewStyle.setNumVerticalLabels(1+(maxY-minY)/tickSize);
        graphView.setGraphViewStyle(graphViewStyle);        

        // Add the graph to the layout
        LinearLayout g = (LinearLayout)rootView.findViewById(R.id.temp_graph);
        g.addView(graphView);
                
        graphView.scrollToEnd();

        // Return the layout for this fragment
        return rootView;
    }
    
    // Called when returning to this fragment (when its tab is reselected). Restores the state of UI objects.
    @Override
    public void onViewStateRestored (Bundle savedInstanceState) {
        super.onViewStateRestored(savedInstanceState);

        Double d = graphViewData[graphViewData.length-1].getY();
        mLastTemp.setText(String.valueOf(d.intValue())+"\u00b0");

        return;
    }
    
    // Looks for the string "253,<int>" where <int> is a temperature value. If there is a match,
    // updates the temperature log with the temperature value.
    public static void parseTemperature(String s, LinkedList<Integer> l) {
        int i;
        s=s.trim();
        String splitS[] = s.split(",", 2);
        if (splitS.length != 2) {
            return;
        }
        if (!splitS[0].contentEquals(enableCmd)) {
            return;
        }
        try {
            i=Integer.parseInt(splitS[1]);
        } catch (NumberFormatException e) {
            return;
        }
        l.add(i);
        l.remove();
            
        return;
    }

    // Initializes the temperature log with all 0s.
    public static void initTemperatureList(LinkedList<Integer> l, int size) {
        int i;
        for (i=0;i<size;i++) {
            l.add(0);
        }
    }
    
    // Sets the graph data to the values in the temperature log.
    void setGraphViewData(LinkedList<Integer> l) {
        int i;
        for (i=0;i<l.size();i++) {
            graphViewData[i] = new GraphViewData(i, l.get(i));
        }        
    }
        
    // Updates the graph with the temperature log.
    public void updateGraph(LinkedList<Integer> l) {
        setGraphViewData(l);
        graphViewSeries.resetData(graphViewData);
        graphViewSeries.removeGraphView(graphView);
        graphViewSeries.addGraphView(graphView);
        
        graphView.scrollToEnd();
        mLastTemp.setText(l.getLast().toString()+"\u00b0");   
    }
    
    // Create the frequency values from the frequency spec table.
    void createFreqValues() {
        int start, end, incr;
        int i, j, count;
        int total = 0;
        count = 0;
        j=0;
        for (i=0;i<freqSpec.length;i++) {
            total = total + (freqSpec[i][1] - freqSpec[i][0])/freqSpec[i][2];
        }
        total++;
        freqValues = new ArrayList<Integer>();
        
        for (i=0;i<freqSpec.length;i++) {
            start = freqSpec[i][0];
            end = freqSpec[i][1];
            incr = freqSpec[i][2];
            for (j=start;j<end;j+=incr) {
                freqValues.add(j);
                Log.d(PIC32_BTSK.TAG,""+freqValues.get(count));
                count++;
            }
        }
        freqValues.add(j);
        Log.d(PIC32_BTSK.TAG,""+freqValues.get(count));
        
        return;
    }
}