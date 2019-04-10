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

import net.margaritov.preference.colorpicker.ColorPickerPanelView;
import net.margaritov.preference.colorpicker.ColorPickerView;

import android.support.v4.app.Fragment;
import android.support.v4.view.MotionEventCompat;

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import com.mchp.android.PIC32_BTSK.R;

/*
 * This is the Color fragment. It has two user interfaces to select a color: individual R G and B values, and a color picker. 
 * A color panel shows the currently selected color. The color command is sent every time any of these interfaces 
 * changes.
 */
public class ColorFragment extends Fragment implements 
SeekBar.OnSeekBarChangeListener,
Button.OnClickListener,
ColorPickerView.OnColorChangedListener {

    // Constants
    static String colorCmd = "255,03";             // Command to set the LED color
    static int colorMax = 254;                     // Maximum color value
    static int colorDefault = 127;                 // Default color value

    // UI elements
    // Seekbars, fine tune buttons and text for R G and B
    private SeekBar mSeekBarR;
    private TextView mProgressTextR;
    private Button mUpR;
    private Button mDownR;
    
    private SeekBar mSeekBarG;
    private TextView mProgressTextG;
    private Button mUpG;
    private Button mDownG;

    private SeekBar mSeekBarB;
    private TextView mProgressTextB;
    private Button mUpB;
    private Button mDownB;

    // Color picker
    private ColorPickerView mColorPicker;

    // Color panel shows the currently selected color
    private ColorPickerPanelView mNewColor;

    // Maximum value for each seekbar
    int seekBarMax = colorMax;

    // Currently selected color
    int mColor;

    // Currently selected R G and B values
    int r = colorDefault;
    int g = colorDefault;
    int b = colorDefault;
    
    
    // Callback functions
    // These callbacks allow a fragment to call a function defined in its parent Activity.

    // This callback allows this fragment to send a string over Bluetooth, using its parent Activity's Bluetooth 
    // connection. This is used to send a color command over Bluetooth when the currently selected color changes.
    public interface OnSendRequestListener {
        public void onSendRequest(String s);
    }
    OnSendRequestListener mSendRequestCallback;

    // This callback allows this fragment to send a queued string over Bluetooth, using its parent Activity's Bluetooth 
    // connection. A queued request means that it will be sent at the next opportunity. This is used to send a color 
    // command over Bluetooth when the user lifts their finger from the color slider or color picker.
    public interface OnLastColorSendRequestListener {
        public void onLastColorSendRequest(String s);
    }
    OnLastColorSendRequestListener mLastColorSendRequestCallback;

    // This callback allows this fragment to cancel the last color send request. It is called when a new send request is sent,
    // to tell the parent that the last color is no longer the last color.
    public interface OnCancelLastColorRequestListener {
        public void onCancelLastColorRequest();
    }
    OnCancelLastColorRequestListener mCancelLastColorRequestCallback;

    // Action callbacks
    // These callbacks happen when the user interacts with the UI elements

    // Called when the user changes the color in the color picker. Gets the new RGB values from the color picker,
    // updates the other UI elements with the new color, and sends the color command.
    @Override
    public void onColorChanged(int color) {
        mColor = color;        

        r=Color.red(mColor);
        g=Color.green(mColor);
        b=Color.blue(mColor);

        r = (r <= seekBarMax) ? r : seekBarMax;
        g = (g <= seekBarMax) ? g : seekBarMax;
        b = (b <= seekBarMax) ? b : seekBarMax;

        mColor = Color.rgb(r,g,b);        
        setProgressText();
        setSeekBarProgress();
        mNewColor.setColor(mColor);
                
        sendLedCmd(r,g,b);
    }
    
    // Called when the user changes any of the three seekbars. Gets the new RGB values from the seekbars,
    // updates the other UI elements with the new color, and sends the color command.
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromTouch) {
        if (!fromTouch) {
            return;
        }
        
        getSeekBarProgress();

        mColor = Color.rgb(r,g,b);  
        setProgressText();
        mColorPicker.setColor(mColor);
        mNewColor.setColor(mColor);

        sendLedCmd(r,g,b);
    }

    // Called when the user starts changing any of the three seekbars
    public void onStartTrackingTouch(SeekBar seekBar) {
    }

    // Called when the user stops changing any of the three seekbars. Sends the color command.
    public void onStopTrackingTouch(SeekBar seekBar) {
        sendLastColorLedCmd(r,g,b);
    }
    
    // Called when the user clicks any of the fine tune buttons. Updates the appropriate current RGB value,
    // updates all of the UI elements with the new color, and sends the color command.
    @Override
    public void onClick(View v) {

        int id = v.getId();
		if (id == R.id.red_down) {
			if (r > 0) r--;
		} else if (id == R.id.red_up) {
			if (r < seekBarMax) r++;
		} else if (id == R.id.green_down) {
			if (g > 0) g--;
		} else if (id == R.id.green_up) {
			if (g < seekBarMax) g++;
		} else if (id == R.id.blue_down) {
			if (b > 0) b--;
		} else if (id == R.id.blue_up) {
			if (b < seekBarMax) b++;
		} else {
		}
                
        mColor = Color.rgb(r,g,b);
        setProgressText();        
        setSeekBarProgress();
        mColorPicker.setColor(mColor);
        mNewColor.setColor(mColor);

        sendLastColorLedCmd(r,g,b);

        return;
    }
    
    // Lifecycle callbacks
    // These are called by the system when starting, closing, or navigating through the app
    
    // Called when this fragment is attached to its container activity (when its tab is selected)
    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        
        // This checks that the container activity (PIC32_BTSK) has implemented
        // the OnSendRequestListener interface. If not, it throws an exception. If it
        // has been implemented, sets mSendRequestCallback to the implementation in the
        // container activity.
        try {
            mSendRequestCallback = (OnSendRequestListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString() + " must implement OnSendRequestListener");
        }

        // This checks that the container activity (PIC32_BTSK) has implemented
        // the OnLastColorSendRequestListener interface. If not, it throws an exception. If it
        // has been implemented, sets mLastColorSendRequestCallback to the implementation in the
        // container activity.
        try {
            mLastColorSendRequestCallback = (OnLastColorSendRequestListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString() + " must implement OnLastColorSendRequestListener");
        }

        // This checks that the container activity (PIC32_BTSK) has implemented
        // the OnCancelLastColorRequestListener interface. If not, it throws an exception. If it
        // has been implemented, sets mCancelLastColorRequestCallback to the implementation in the
        // container activity.
        try {
            mCancelLastColorRequestCallback = (OnCancelLastColorRequestListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString() + " must implement OnCancelLastColorRequestListener");
        }
        return;
    }
    
    // Called when this fragment is created
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);        
    }

    // Called when this fragment View is created. Performs setup of the UI.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, 
        Bundle savedInstanceState) {
        
        super.onCreateView(inflater, container, savedInstanceState);
        
        // Inflate the layout for this fragment
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_color, container, false);

        // Get each UI object from the layout
        mSeekBarR = (SeekBar)rootView.findViewById(R.id.seekR);
        mSeekBarG = (SeekBar)rootView.findViewById(R.id.seekG);
        mSeekBarB = (SeekBar)rootView.findViewById(R.id.seekB);
        
        mProgressTextR = (TextView)rootView.findViewById(R.id.progressR);
        mProgressTextG = (TextView)rootView.findViewById(R.id.progressG);
        mProgressTextB = (TextView)rootView.findViewById(R.id.progressB);

        mDownR = (Button)rootView.findViewById(R.id.red_down);
        mUpR = (Button)rootView.findViewById(R.id.red_up);
        mDownG = (Button)rootView.findViewById(R.id.green_down);
        mUpG = (Button)rootView.findViewById(R.id.green_up);
        mDownB = (Button)rootView.findViewById(R.id.blue_down);
        mUpB = (Button)rootView.findViewById(R.id.blue_up);

        mColorPicker = (ColorPickerView) rootView.findViewById(R.id.color_picker_view);
        mNewColor = (ColorPickerPanelView) rootView.findViewById(R.id.new_color_panel);
        
        // Set the maximum value for each seekbar
        mSeekBarR.setMax(seekBarMax);
        mSeekBarG.setMax(seekBarMax);
        mSeekBarB.setMax(seekBarMax);

        // Initialize the states of the UI objects
        setSeekBarProgress();
        setProgressText();
                
        mDownR.setText("-");
        mUpR.setText("+");
        mDownG.setText("-");
        mUpG.setText("+");
        mDownB.setText("-");
        mUpB.setText("+");

        mColor = Color.rgb(r,g,b);
        
        mColorPicker.setColor(mColor);
        mNewColor.setColor(mColor);

        // Set listeners for the UI objects
        mSeekBarR.setOnSeekBarChangeListener(this);
        mSeekBarG.setOnSeekBarChangeListener(this);
        mSeekBarB.setOnSeekBarChangeListener(this);

        mDownR.setOnClickListener(this);
        mUpR.setOnClickListener(this);
        mDownG.setOnClickListener(this);
        mUpG.setOnClickListener(this);
        mDownB.setOnClickListener(this);
        mUpB.setOnClickListener(this);

        mColorPicker.setOnColorChangedListener(this);

        // Define a touch listener for the color picker. When released, it sends the color 
        // command with the current color.
        mColorPicker.setOnTouchListener(new OnTouchListener () {
            public boolean onTouch(View v, MotionEvent event) {
                int action = MotionEventCompat.getActionMasked(event);
                switch (action) {
                case MotionEvent.ACTION_UP:
                    sendLastColorLedCmd(r,g,b);
                    return true;
                default:
                    return false;
                }
            }
        });
        
        // Define a touch listener for the color panel. When pressed, its transparency
        // changes to indicate that it has been pressed. When released, it sends the color 
        // command with the current color.
        mNewColor.setOnTouchListener(new OnTouchListener () {
            public boolean onTouch(View v, MotionEvent event) {
                int action = MotionEventCompat.getActionMasked(event);
                switch (action) {
                case MotionEvent.ACTION_DOWN:
                    try {
                        mNewColor.setAlpha((float) 0.5);
                    } catch (NoSuchMethodError e) {
                    }
                    return true;
                case MotionEvent.ACTION_UP:
                    try {
                        mNewColor.setAlpha((float) 1.0);
                    } catch (NoSuchMethodError e) {
                    }
                    sendLedCmd(r,g,b);

                    return true;
                default:
                    return false;
                }
            }
        });
                
        
        // Return the layout for this fragment
        return rootView;
    }
    
    // Called when returning to this fragment (when its tab is reselected). Restores the state of UI objects.
    @Override
    public void onViewStateRestored (Bundle savedInstanceState) {
        super.onViewStateRestored(savedInstanceState);

        getSeekBarProgress();

        setProgressText();
        mColor = Color.rgb(r, g, b);
        mColorPicker.setColor(mColor);
        mNewColor.setColor(mColor);

        return;
    }
    
    // Updates the R G B values from the seekbars.
    void getSeekBarProgress() {
        r = mSeekBarR.getProgress();
        g = mSeekBarG.getProgress();
        b = mSeekBarB.getProgress();        
    }

    // Updates the seekbars from the current R G B values.
    void setSeekBarProgress() {
        mSeekBarR.setProgress(r);
        mSeekBarG.setProgress(g);
        mSeekBarB.setProgress(b);
    }

    // Updates the text from the current R G B values.
    void setProgressText() {
        mProgressTextR.setText(r+"");
        mProgressTextG.setText(g+"");
        mProgressTextB.setText(b+"");        
    }
    
    // Sends an LED control command
    void sendLedCmd(int r, int g, int b) {
        String message=colorCmd+",";
        message = message+r+",";
        message = message+g+",";
        message = message+b;
        mCancelLastColorRequestCallback.onCancelLastColorRequest();
        mSendRequestCallback.onSendRequest(message);

        return;
    }
    
    // Sends an LED control command for the last color
    void sendLastColorLedCmd(int r, int g, int b) {
        String message=colorCmd+",";
        message = message+r+",";
        message = message+g+",";
        message = message+b;
        mLastColorSendRequestCallback.onLastColorSendRequest(message);

        return;
    }
}