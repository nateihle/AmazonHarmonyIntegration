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
/*
 * This file incorporates work covered by the following copyright and permission
 * notice:
 */
/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * Modifications to original (BluetoothChat):
 * Changed class name to BluetoothActivity
 * Moved all UI elements to a child fragment (TextFragment).
 * Added tab interface. 
 * Added interfaces to three child fragments.
 * Added delay to sendMessage().
 * Enabled clicking on app icon to open options menu.
 * Added support down to API level 6.
 * Changed some function and variable names.
 * 
 */

package com.mchp.android.PIC32_BTSK;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.StreamCorruptedException;
import java.util.LinkedList;
import java.util.Set;

import android.support.v7.app.ActionBar;
import android.support.v7.app.ActionBarActivity;
import android.support.v7.widget.Toolbar;

import android.support.v4.app.DialogFragment;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentTabHost;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager.NameNotFoundException;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.Window;
import android.view.inputmethod.InputMethodManager;
import android.widget.ArrayAdapter;
import android.widget.Toast;
import com.mchp.android.PIC32_BTSK.R;

/*
 * This is the main Activity. It handles all the underlying Bluetooth connection and communication, and also communicates with
 * the Color, Temperature and Text tabs.
 */
public class PIC32_BTSK extends ActionBarActivity 
implements 
FragmentTabHost.OnTabChangeListener,
ColorFragment.OnSendRequestListener,
ColorFragment.OnLastColorSendRequestListener,
ColorFragment.OnCancelLastColorRequestListener,
TemperatureFragment.OnSendRequestListener, 
TemperatureFragment.OnTempRequestListener,
TextFragment.OnSendRequestListener, 
TextFragment.OnTextLogRequestListener
{
    // Constants
    static long sendDelay = 200;        // Send delay in ms. After a send request, this time must elapse before another will be 
                                        // sent. All requests on the interim will be ignored.
    static int numTemperatures = 100;   // Number of temperature readings to keep. Older readings will be discarded.      
    
    // Support for hiding the soft keyboard
    private InputMethodManager mgr;
    
    // Support for tabs
    private android.app.ActionBar actionBar;
    private Toolbar toolbar;
    private FragmentTabHost mTabHost;
    private FragmentManager mFragmentManager;
    
    // Fragments
    private ColorFragment colorFrag;               // Select and send a color
    private TemperatureFragment temperatureFrag;   // View temperature log, enable temperature updates, select and send
                                                   // an update rate
    private TextFragment textFrag;                 // Enter and send text strings, view transmit/receive log
    
    // Transmit/receive log
    private ArrayAdapter<String> mConversationArrayAdapter;

    // Temperature log
    private LinkedList<Integer> mTemperatures;
    int maxCount = numTemperatures;

    // Last color sent
    private String mLastColor = null;    
    
    // Send delay count down timer
    CountDownTimer mSendTimer;
    boolean mSendTimeElapsed = true;
    long mSendDelay = sendDelay;

    // Bluetooth utilities
    // Debugging
    public static final String TAG = "PIC32_BTSK";
    public static final boolean D = true;

    // Message types sent from the BluetoothService Handler
    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    public static final int MESSAGE_WRITE = 3;
    public static final int MESSAGE_DEVICE_NAME = 4;
    public static final int MESSAGE_TOAST = 5;

    // Key names received from the BluetoothService Handler
    public static final String DEVICE_NAME = "device_name";
    public static final String DEVICE_ADDR = "device_addr";
    public static final String TOAST = "toast";

    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE = 1;
    private static final int REQUEST_ENABLE_BT = 2;
    
    // Name of the connected device
    private String mConnectedDeviceName = null;

    // Local Bluetooth adapter
    private BluetoothAdapter mBluetoothAdapter = null;
    
    // Member object for the Bluetooth services
    public BluetoothService mBluetoothService = null;
    
    // Last connected device
    private String mLastConnectedDeviceAddr = null;
    private static final String PREF_DEVICE_ADDR = "pref_device_addr";
    
    private static final String LAST_CONNECTED_DEVICE = "last_connected_device";
    
    // Toast
    private Toast mToast;
        
    // Callback functions. The fragments can call these functions.
    
    // Allows the fragment to send a string over Bluetooth, using this Activity's Bluetooth connection.
    // The message request is sent immediately. If it occurs within the send delay, it is discarded.
    @Override
    public void onSendRequest(String s) {
        sendMessage(s);
        return;
    }

    // Allows the color fragment to send a the last color over Bluetooth, using this Activity's Bluetooth connection.
    // If the request occurs within the send delay, it will be sent after the send delay.
    // This ensures that the last color selected by the user will be sent.
    @Override
    public void onLastColorSendRequest(String s) {
        if (mSendTimeElapsed) {
            sendMessage(s);            
        }
        else {
            mLastColor = s;
        }
        return;
    }

    // Allows the color fragment to cancel the last color send request.
    @Override
    public void onCancelLastColorRequest() {
        mLastColor = null;
        return;
    }

    // Allows the fragment to get the temperature log.
    @Override
    public LinkedList<Integer> onTempRequest() {    
        return mTemperatures;
    }

    // Allows the fragment to get the transmit/receive log.
    @Override
    public ArrayAdapter<String> onTextLogRequest() {
        return mConversationArrayAdapter;
    }

    // Lifecycle callbacks
    // These are called by the system when starting, closing, or navigating through the app

    // Called when the app is started. Performs setup of the UI and initializes variables.
	@Override
    public void onCreate(Bundle savedInstanceState) {
		
        
        
        super.onCreate(savedInstanceState);
        if(D) Log.e(TAG, "+++ ON CREATE +++");
        
     // delegated in Android 5.0
     // Request the Action Bar.
//        requestWindowFeature(Window.FEATURE_ACTION_BAR);
        
        // Set up the action bar
//           actionBar = getSupportActionBar();
//           actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_STANDARD);
//           actionBar.setHomeButtonEnabled(true);
        
        // Get the window layout
        setContentView(R.layout.main);

        // Use toolbar instead of action bar
        toolbar =  (Toolbar) findViewById(R.id.my_toolbar);
        setSupportActionBar(toolbar);
        getSupportActionBar().setHomeButtonEnabled(true);
        getSupportActionBar().setIcon(R.drawable.app_icon);
        
        

        // Get local Bluetooth adapter
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported on this device. Exit the app.
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        // Initialize the temperature log
        mTemperatures = new LinkedList<Integer>();
        TemperatureFragment.initTemperatureList(mTemperatures, maxCount);

        
        
        // Set up the tab interface. Add one tab for each of the three fragments.
        mFragmentManager = this.getSupportFragmentManager();
        
        mTabHost = (FragmentTabHost) findViewById(R.id.tabhost);
        mTabHost.setup(this, mFragmentManager, R.id.tabFrameLayout);

        mTabHost.addTab(
                mTabHost.newTabSpec("color").setIndicator("Color"),
                ColorFragment.class, null);
        mTabHost.addTab(
                mTabHost.newTabSpec("temperature").setIndicator("Temperature"),
                TemperatureFragment.class, null);
        mTabHost.addTab(
                mTabHost.newTabSpec("text").setIndicator("Text"),
                TextFragment.class, null);
        mTabHost.setOnTabChangedListener(this);

        // Set up the send delay count down timer. While it's counting down, no messages can be sent over Bluetooth.
        mSendTimer = new CountDownTimer(mSendDelay, mSendDelay) {
            public void onTick(long millisUntilFinished) {
            }

            public void onFinish() {
                mSendTimeElapsed = true;
                // Send the last color
                if (mLastColor != null) {
                    sendMessage(mLastColor);
                    mLastColor = null;
                    return;
                }
            }
        };
        
        // Get the input method manager for hiding the soft keyboard
        mgr = (InputMethodManager) getApplicationContext().getSystemService(Context.INPUT_METHOD_SERVICE);
        
        // Setup toasts
        mToast = Toast.makeText(this, "", Toast.LENGTH_SHORT);
    }
    
    // Called when the app becomes visible to the user. Checks if Bluetooth is enabled and initializes the Bluetooth service.
    @Override
    public void onStart() {
        super.onStart();
        if(D) Log.e(TAG, "++ ON START ++");

        // If BT is not on, request that it be enabled.
        // setupBTService() will then be called during onActivityResult
        if (!mBluetoothAdapter.isEnabled()) {
            Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
        // Otherwise, setup the Bluetooth session
        } else {
            if (mBluetoothService == null) setupBTService();
        }
    }

    // Called when the user can start interacting with the app. Starts the Bluetooth service.
    @Override
    public synchronized void onResume() {
        super.onResume();
        if(D) Log.e(TAG, "+ ON RESUME +");

        // Performing this check in onResume() covers the case in which BT was
        // not enabled during onStart(), so we were paused to enable it...
        // onResume() will be called when ACTION_REQUEST_ENABLE activity returns.
        if (mBluetoothService != null) {
            // Only if the state is STATE_NONE, do we know that we haven't started already
            if (mBluetoothService.getState() == BluetoothService.STATE_NONE) {
              // Start the Bluetooth service
              mBluetoothService.start();
            }
            
            // Reconnect last connected device
            if (mBluetoothService.getState() != BluetoothService.STATE_CONNECTED)
            {
    	        SharedPreferences settings = getSharedPreferences(PREF_DEVICE_ADDR, 0);
    	        mLastConnectedDeviceAddr = settings.getString(DEVICE_ADDR, null);
    	        
    	        if (mLastConnectedDeviceAddr != null)
    	        {
    	        	BluetoothAdapter btAdapter = BluetoothAdapter.getDefaultAdapter();
    	        	// Get a set of currently paired devices
    	        	Set<BluetoothDevice> pairedDevices = btAdapter.getBondedDevices();
    		        for (BluetoothDevice device : pairedDevices){
    		        	if (device.getAddress().equals(mLastConnectedDeviceAddr))
    		        	{
    		        		mBluetoothService.connect(device, false);
    		        		break;
    		        	}
    		        }
    	        }
    	        
            }
        }
    }
    
    // Called when the user starts another activity, such as by navigating away from the app.
    @Override
    public synchronized void onPause() {
        super.onPause();
        if(D) Log.e(TAG, "- ON PAUSE -");
    }

    // Called when the app is no longer visible to the user.
    @Override
    public void onStop() {
        super.onStop();
        if(D) Log.e(TAG, "-- ON STOP --");
    }

    // Called when the app is destroyed. Stops the Bluetooth service.
    @Override
    public void onDestroy() {
        super.onDestroy();
        // Stop the Bluetooth service
        if (mBluetoothService != null) mBluetoothService.stop();
        if(D) Log.e(TAG, "--- ON DESTROY ---");
    }

    // Initializes the Bluetooth interface
    private void setupBTService() {
        // Initialize the array adapter for the conversation thread
        mConversationArrayAdapter = new ArrayAdapter<String>(this, R.layout.message);
        
        // Initialize the BluetoothService to perform bluetooth connections
        mBluetoothService = new BluetoothService(this, mHandler);
    }

    // Puts this device into broadcast mode so it is discoverable by other Bluetooth devices
//    private void ensureDiscoverable() {
//        if(D) Log.d(TAG, "ensure discoverable");
//        if (mBluetoothAdapter.getScanMode() !=
//            BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE) {
//            Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
//            discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
//            startActivity(discoverableIntent);
//        }
//    }

    // Sends a message over Bluetooth.
    // @param message  A string of text to send.
    private void sendMessage(String message) {
        // If we're not connected, ignore the request.
        if (mBluetoothService.getState() != BluetoothService.STATE_CONNECTED) {
            mToast.setText(R.string.not_connected);
            mToast.setDuration(Toast.LENGTH_SHORT);
            mToast.show();
            return;
        }

        // If the send delay hasn't elapsed since sending the last command, ignore the request.
        if (!mSendTimeElapsed) {
            return;
        }
        
        // If the string is nonzero, send the message.
        if (message.length() > 0) {
            // Get the message bytes and tell the BluetoothService to write
            byte[] send = message.getBytes();
            mBluetoothService.write(send);

            // Start the send delay timer
            mSendTimer.start();
            mSendTimeElapsed = false;
            
            // Reset out string buffer to zero and clear the edit text field
            try {
                textFrag.resetStringBuffer();
            } catch (NullPointerException e) {}
        }
    }

    // Sets the subtitle in the actionbar to a string, using a resource id as a lookup. Mostly used to show Bluetooth
    // connection status.
    private final void setStatus(int resId) {
//        actionBar.setSubtitle(resId);
    	toolbar.setSubtitle(resId);
    }

    // Sets the subtitle in the actionbar to a string, using a character sequence. Mostly used to show Bluetooth
    // connection status.
    private final void setStatus(CharSequence subTitle) {
    	toolbar.setSubtitle(subTitle);
    }
    
    
    // The Handler that gets information back from the BluetoothService. Called when any of the following happens:
    // Bluetooth connection state changes. Updates the status and/or name of the connected device in the action bar. Sometimes
    //   sends additional information, which is displayed as a pop up message.
    // A message was sent over Bluetooth. Prints the message to the transmit/receive log.
    // A message was received over Bluetooth. Prints the message to the transmit/receive log. Also parses it to see if it was
    //   a temperature update.
    private final Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
            case MESSAGE_STATE_CHANGE:
                if(D) Log.i(TAG, "MESSAGE_STATE_CHANGE: " + msg.arg1);
                switch (msg.arg1) {
                case BluetoothService.STATE_CONNECTED:
                    setStatus(getString(R.string.title_connected_to, mConnectedDeviceName));
                    try {
                        mConversationArrayAdapter.clear();
                    } catch (NullPointerException e) {}
                    break;
                case BluetoothService.STATE_CONNECTING:
                    setStatus(R.string.title_connecting);
                    break;
                case BluetoothService.STATE_LISTEN:
                case BluetoothService.STATE_NONE:
                    setStatus(R.string.title_not_connected);
                    break;
                }
                break;
            case MESSAGE_WRITE:
                byte[] writeBuf = (byte[]) msg.obj;
                // construct a string from the buffer
                String writeMessage = new String(writeBuf);
                try {
                    mConversationArrayAdapter.add("TX: " + writeMessage);
                } catch (NullPointerException e) {}
                break;
            case MESSAGE_READ:
                byte[] readBuf = (byte[]) msg.obj;
                // construct a string from the valid bytes in the buffer
                String readMessage = new String(readBuf, 0, msg.arg1);
                try {
                    mConversationArrayAdapter.add("RX:  " + readMessage);
                } catch (NullPointerException e) {}
                
                try {
                    TemperatureFragment.parseTemperature(readMessage, mTemperatures);
                    temperatureFrag.updateGraph(mTemperatures);
                } catch (NullPointerException e) {}

                break;
            case MESSAGE_DEVICE_NAME:
                // save the connected device's name
                mConnectedDeviceName = msg.getData().getString(DEVICE_NAME);
                mLastConnectedDeviceAddr = msg.getData().getString(DEVICE_ADDR);
                
                if(D) Log.e(TAG, "connect to "+mLastConnectedDeviceAddr);
                
                SharedPreferences settings = getSharedPreferences(PREF_DEVICE_ADDR, Context.MODE_PRIVATE);
                SharedPreferences.Editor editor = settings.edit();
                editor.putString(DEVICE_ADDR, mLastConnectedDeviceAddr);
                editor.commit();
                     
                Toast.makeText(getApplicationContext(), "Connected to "
                               + mConnectedDeviceName, Toast.LENGTH_SHORT).show();
                break;
            case MESSAGE_TOAST:
                Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST),
                               Toast.LENGTH_SHORT).show();
                break;
            }
        }
    };

    // Called when any of the following happens:
    // The user has selected a Bluetooth device to connect with. Connects with that device. 
    // The app has requested that Bluetooth has been enabled on our device. Initialize Bluetooth service if it was successful,
    //    exit the app otherwise
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        if(D) Log.d(TAG, "onActivityResult " + resultCode);
        switch (requestCode) {
        case REQUEST_CONNECT_DEVICE:
        	// Connect device insecure
        	if (resultCode == Activity.RESULT_OK){
        		connectDevice(data, false);
        	}
        	break;
        case REQUEST_ENABLE_BT:
            // When the request to enable Bluetooth returns
            if (resultCode == Activity.RESULT_OK) {
                // Bluetooth is now enabled, so set up a session
                setupBTService();
            } else {
                // User did not enable Bluetooth or an error occurred
                Log.d(TAG, "BT not enabled");
                Toast.makeText(this, R.string.bt_not_enabled_leaving, Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }

    // Connects to the requested device over bluetooth
    private void connectDevice(Intent data, boolean secure) {
        // Get the device MAC address
        String address = data.getExtras()
            .getString(DeviceListActivity.EXTRA_DEVICE_ADDRESS);
        // Get the BluetoothDevice object
        BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
        // Attempt to connect to the device
        mBluetoothService.connect(device, secure);
    }
    
    // Called when the user opens the Options menu
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.option_menu, menu);

        return true;
    }

    // Called when the user selects an item in the Options menu
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        Intent serverIntent = null;
        mToast.cancel();
        int itemId = item.getItemId();
		if (itemId == android.R.id.home) {
			openOptionsMenu();
			return true;
		}
		else if (itemId == R.id.connect_scan){
		
			serverIntent = new Intent(this, DeviceListActivity.class);
			startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
			return true;
		}else if (itemId == R.id.about) {
			// Show the About dialog
            showAbout();
			return true;
		}
        return false;
    }

    // Called when the user navigates away from the app. Saves the state of the app so it can be restored
    // when the user comes back to it.
    @Override
    public void onSaveInstanceState(Bundle outState)
    {
        super.onSaveInstanceState(outState);
        
        return;
    }

    // Called when each tab is selected for the first time.
    @Override
    public void onAttachFragment(Fragment fragment) {
        super.onAttachFragment(fragment);
        if (fragment.getClass().equals(TextFragment.class)) {
            textFrag = (TextFragment)fragment;
        }
        else if (fragment.getClass().equals(TemperatureFragment.class)) {
            temperatureFrag = (TemperatureFragment)fragment;
        }
        else if (fragment.getClass().equals(ColorFragment.class)) {
            colorFrag = (ColorFragment)fragment;
        }
    }

    // Called when the user switches tabs. Hides the soft keyboard.
    @Override
    public void onTabChanged(String tabId) {
        try {
            textFrag.hideKeyboard(mgr);
        } catch (NullPointerException e) {}
    }
    
    // Called when the user selects About.
    public void showAbout() {
        // Get Version Name and Display in About dialog
        Context context = getApplicationContext();
        try {
			String versionName = context.getPackageManager()
				    .getPackageInfo(getApplicationContext().getPackageName(), 0).versionName;
			
			DialogFragment newFragment = new AboutDialogFragment(versionName);
	        newFragment.show(getSupportFragmentManager(), "about");
		} catch (NameNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        
    }    
}