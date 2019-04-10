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
 * This is a new class, which is a fragment that contains only the UI elements of the original class.
 * Added function to hide soft keyboard.
 * Changed some function and variable names.
 */

package com.mchp.android.PIC32_BTSK;

import android.support.v4.app.Fragment;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.View.OnClickListener;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;
import com.mchp.android.PIC32_BTSK.R;

/*
 * This is the Text fragment. It has an interface to enter and send text strings over Bluetooth, and a log of all 
 * transmit/receive data from the current Bluetooth session.
 */
public class TextFragment extends Fragment {
    
    // UI elements
    // Layout Views
    private ListView mConversationView;
    private EditText mOutEditText;
    private Button mSendButton;

    // String buffer for outgoing messages
    private StringBuffer mOutStringBuffer;

    // Callback functions
    // These callbacks allow a fragment to call a function defined in its parent Activity.

    // This callback allows this fragment to send a string over Bluetooth, using its parent Activity's Bluetooth 
    // connection. This is used to send the user-entered string over Bluetooth when the user presses the Send button.
    public interface OnSendRequestListener {
        public void onSendRequest(String s);
    }
    OnSendRequestListener mSendRequestCallback;
 
    // This callback allows this fragment to get the transmit/receive log, which is stored by its parent Activity. This 
    // is used to get the log when the fragment is created.
    public interface OnTextLogRequestListener {
        public ArrayAdapter<String> onTextLogRequest();
    }
    OnTextLogRequestListener mTextLogRequestCallback;

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
        // the OnTextLogRequestListener interface. If not, it throws an exception. If it
        // has been implemented, sets mTextLogRequestCallback to the implementation in the
        // container activity.
        try {
            mTextLogRequestCallback = (OnTextLogRequestListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString() + " must implement OnTextLogRequestListener");
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
        ViewGroup rootView = (ViewGroup) inflater.inflate(R.layout.fragment_text, container, false);

        // Initialize the array adapter for the conversation thread
        mConversationView = (ListView) rootView.findViewById(R.id.in);
        mConversationView.setAdapter(mTextLogRequestCallback.onTextLogRequest());

        // Initialize the compose field with a listener for the return key
        mOutEditText = (EditText) rootView.findViewById(R.id.edit_text_out);
        mOutEditText.setOnEditorActionListener(mWriteListener);
        
        // Initialize the send button with a listener that for click events
        mSendButton = (Button) rootView.findViewById(R.id.button_send);
        mSendButton.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                // Send a message using content of the edit text widget
                String message = mOutEditText.getText().toString();
                mSendRequestCallback.onSendRequest(message);
            }
        });

        // Initialize the buffer for outgoing messages
        mOutStringBuffer = new StringBuffer("");

        // Return the layout for this fragment
        return rootView;
    }

    // Resets the textbox after a message has been sent.
    public void resetStringBuffer() {
        // Reset out string buffer to zero and clear the edit text field
        mOutStringBuffer.setLength(0);        
        mOutEditText.setText(mOutStringBuffer);
    }

    // The action listener for the EditText widget, to listen for the return key
    private TextView.OnEditorActionListener mWriteListener =
        new TextView.OnEditorActionListener() {
        public boolean onEditorAction(TextView view, int actionId, KeyEvent event) {
            // If the action is a key-up event on the return key, send the message
            if (actionId == EditorInfo.IME_NULL && event.getAction() == KeyEvent.ACTION_UP) {
                String message = view.getText().toString();
                mSendRequestCallback.onSendRequest(message);
            }
            if(PIC32_BTSK.D) Log.i(PIC32_BTSK.TAG, "END onEditorAction");
            return true;
        }
    };
    
    // Hides the soft keyboard.
    public void hideKeyboard(InputMethodManager mgr) {
        mgr.hideSoftInputFromWindow(mOutEditText.getWindowToken(), 0);
    }
}