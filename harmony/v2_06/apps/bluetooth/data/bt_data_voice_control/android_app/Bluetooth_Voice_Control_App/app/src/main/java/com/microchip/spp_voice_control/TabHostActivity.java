
package com.microchip.spp_voice_control;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.ShortBuffer;
import java.util.Set;

import android.app.Activity;
import android.app.ActivityManager;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.pm.PackageManager.NameNotFoundException;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.Environment;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.text.TextUtils;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.inputmethod.InputMethodManager;
import android.widget.ArrayAdapter;
import android.widget.Toast;

import android.support.v4.app.DialogFragment;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentTabHost;
import android.support.v7.app.ActionBarActivity;
import android.support.v7.widget.Toolbar;

public class TabHostActivity extends ActionBarActivity implements
		FragmentTabHost.OnTabChangeListener,
		TextFragment.OnSendRequestListener,
		TextFragment.OnTextLogRequestListener,
		SpeechCMDFragment.OnSendRequestListener
		{
	// Constants
	static long sendDelay = 200; // Send delay in ms. After a send request, this
									// time must elapse before another will be
									// sent. All requests on the interim will be
									// ignored.
	static final int BUFFER_TIME = 3; // buffering 3 seconds voice data
	static final int SAMPLE_RATE = 16000;
	static final int BYTES_PER_SECOND = SAMPLE_RATE*2;
	// Support for hiding the soft keyboard
	private InputMethodManager mgr;

	// Support for tabs
	private Toolbar toolbar;
	private FragmentTabHost mTabHost;
	private FragmentManager mFragmentManager;

	// Send delay count down timer
	CountDownTimer mSendTimer;
	boolean mSendTimeElapsed = true;
	long mSendDelay = sendDelay;

	public static final String TAG = "SPP_BOOTLOADER";
	public static final boolean D = true;

	private TextFragment textFrag; // Enter and send text strings, view
									// transmit/receive log

	private SpeechCMDFragment speechCmdFrag; // Send led turn on/off cmd
	private DataRateFragment dataRateFrag;

	// Transmit/receive log
	private ArrayAdapter<String> mConversationArrayAdapter;


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
	private static final String PREFS = "prefs";
	private String mLastConnectedDeviceAddr = null;

	private SharedPreferences mSharedPrefSettings = null;
	private SharedPreferences.Editor mPrefEditor = null;

	// Toast
	private Toast mToast;

	// Audio data file
	private static final String AUDIO_DATA_FILE = "audio_data.raw";
	private FileOutputStream fileOutputStream1;
	private FileOutputStream fileOutputStream2;
	private FileOutputStream fileOutputStreamDebug;
	private int bytesInFileDebug=0;
	private boolean STREAM2FILE1 = true;
	private boolean STREAM2BUFFER1 = true;
	private final int bytesPerSample = 32000; // 16Khz, mono, 16bit data
	private byte[] voiceBuffer1;
	private byte[] voiceBuffer2;


	public Handler getHandler()
	{
		return mHandler;
	}

	// Create the Handler object for periodically send data to board.
	Handler send2Boardhandler = new Handler();
	// Define the code block to be executed
	private Runnable hb2boardRunnable = new Runnable() {
		@Override
		public void run() {
			// Do something here on the main thread
			Log.d("Handlers", "Called on main thread");
			sendMessage("hb"); // send heartbeat
			// Repeat this the same runnable code block again another 4 seconds
			// 'this' is referencing the Runnable object
			send2Boardhandler.postDelayed(this, 4000);
		}
	};

	private int dataReceived = 0;
	Handler transferRateCalHandler = new Handler();
	private Runnable transferRateCalRunnable = new Runnable()
	{
		@Override
		public void run() {
			int kbps = dataReceived*8/1024;
			if(dataRateFrag!=null)
			{
				dataRateFrag.updateDataRateText(String.valueOf(kbps)+"kbps");
			}
			dataReceived = 0;
			transferRateCalHandler.postDelayed(this, 1000);//per second
		}
	};

	//////Speech Service////////////
	private SpeechService mSpeechService;
	private final ServiceConnection mServiceConnection = new ServiceConnection() {

		@Override
		public void onServiceConnected(ComponentName componentName, IBinder binder) {
			mSpeechService = SpeechService.from(binder);
			mSpeechService.addListener(mSpeechServiceListener);
			// Connected
			//mSpeechService.startRecognizing(16000); // fixed rate
		}

		@Override
		public void onServiceDisconnected(ComponentName componentName) {
			mSpeechService = null;
		}

	};
	private boolean sent2Board = false;
	private final SpeechService.Listener mSpeechServiceListener =
			new SpeechService.Listener() {
				@Override
				public void onSpeechRecognized(final String text, final float likehood, final boolean isFinal) {
					Log.e("Recognized","callback");
					if (isFinal) {
						finishRecognition();
					}
					if(STREAM2FILE1 == false)
					{
						// file1 is for recognition at this point
						try {
							PrintWriter pw = new PrintWriter(fileRootPath+AUDIOFILENAME1); // clear file1 content
							pw.close();
						} catch (FileNotFoundException e) {
							e.printStackTrace();
						}
						try {
							fileOutputStream1 = new FileOutputStream(file1);// recreated file reference
						} catch (FileNotFoundException e) {
							e.printStackTrace();
						}

						STREAM2FILE1 = true;
					}else {
						try {
							PrintWriter pw = new PrintWriter(fileRootPath+AUDIOFILENAME2);
							pw.close();
						} catch (FileNotFoundException e) {
							e.printStackTrace();
						}
						try {
							fileOutputStream2 = new FileOutputStream(file2);
						} catch (FileNotFoundException e) {
							e.printStackTrace();
						}
						STREAM2FILE1 = false;
					}

					if (text != null && !TextUtils.isEmpty(text)) {
						runOnUiThread(new Runnable() {
							@Override
							public void run() {
								if (isFinal) {
									mConversationArrayAdapter.add(text+"(likehood:"+likehood+")");
									if(text.contains("light") )
									{
										if(text.contains("on"))
										{
											sendMessage("L31");// all lights on
											if(speechCmdFrag!=null)
												speechCmdFrag.updateVoiceCmdRecognizedField("Light On");
											Log.d("VOICE RECOGNIZE:","Send L31");
											if(!sent2Board)
											{
												send2Boardhandler.post(hb2boardRunnable);
												sent2Board = true;
											}
										}else if(text.contains("off"))
										{
											sendMessage("L0");// all lights off
											if(speechCmdFrag!=null)
												speechCmdFrag.updateVoiceCmdRecognizedField("Light Off");
											Log.d("VOICE RECOGNIZE:","Send L0");
											if(!sent2Board)
											{
												send2Boardhandler.post(hb2boardRunnable);
												sent2Board = true;
											}
										}
									}
									Log.d("Speech Recog Final:", text);
								} else {
//                                    mText.setText(text);
									mConversationArrayAdapter.add(text+"(likehood:"+likehood+")");
									Log.d("Speech Recognition:--->", text);
								}
							}
						});
					}
				}

			};



	private void finishRecognition()
	{
		if (mSpeechService != null) {
			mSpeechService.finishRecognizing();
		}
	}
	// Callback functions. The fragments can call these functions.

	// Allows the fragment to send a string over Bluetooth, using this
	// Activity's Bluetooth connection.
	// The message request is sent immediately. If it occurs within the send
	// delay, it is discarded.
	@Override
	public void onSendRequest(String s) {
		sendMessage(s);
		return;
	}

	// Allows the fragment to get the transmit/receive log.
	@Override
	public ArrayAdapter<String> onTextLogRequest() {
		return mConversationArrayAdapter;
	}


	byte[] decompressADPCM(byte[] input, int size)
	{
		byte[] output = new byte[size*4];
		ADPCMDecoder.getADPCMDecoderInst().adpcm_decode_frame(input, size, output);
		return output;
	}
	public boolean isExternalStorageWritable() {
		String state = Environment.getExternalStorageState();
		if (Environment.MEDIA_MOUNTED.equals(state)) {
			return true;
		}
		return false;
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		if (D)
			Log.e(TAG, "+++ ON CREATE +++");

		transferRateCalHandler.post(transferRateCalRunnable);
//		voiceBuffer1=new byte[bytesPerSample*BUFFER_TIME+1024];
//		voiceBuffer2=new byte[bytesPerSample*BUFFER_TIME+1024];
		/*
		 *  Using Toolbar instead of using action bar
		 *  Android 5.0 (v21) doesn't support action bar.
		*/
		// Request the Action Bar.
//		getWindow().requestFeature(Window.FEATURE_ACTION_BAR);

		// Set up the action bar
//		actionBar = getSupportActionBar();
//		actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_STANDARD);
//		actionBar.setHomeButtonEnabled(true);
		
		setContentView(R.layout.activity_tab_host);
		
		toolbar =  (Toolbar) findViewById(R.id.my_toolbar);
        setSupportActionBar(toolbar);
        getSupportActionBar().setHomeButtonEnabled(true);
        getSupportActionBar().setIcon(R.drawable.app_icon);

		// Get local Bluetooth adapter
		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

		// If the adapter is null, then Bluetooth is not supported on this
		// device. Exit the app.
		if (mBluetoothAdapter == null) {
			Toast.makeText(this, "Bluetooth is not available",
					Toast.LENGTH_LONG).show();
			finish();
			return;
		}

		// Get shared pref setting and editor
		if (mSharedPrefSettings == null) {
			mSharedPrefSettings = getSharedPreferences(PREFS,
					Context.MODE_PRIVATE);
			mPrefEditor = mSharedPrefSettings.edit();
		}

		

		// Set up the tab interface. Add one tab for each of the three
		// fragments.
		mFragmentManager = this.getSupportFragmentManager();

		mTabHost = (FragmentTabHost) findViewById(android.R.id.tabhost);
		mTabHost.setup(this, mFragmentManager, android.R.id.tabcontent);

		// Disable bootloader function for now
		mTabHost.addTab(mTabHost.newTabSpec("voice").setIndicator("VOICE CMD"),
				SpeechCMDFragment.class, null);
		mTabHost.addTab(mTabHost.newTabSpec("text").setIndicator("Text"),
				TextFragment.class, null);
		mTabHost.addTab(mTabHost.newTabSpec("rate").setIndicator("Data Rate"),
				DataRateFragment.class, null);
		mTabHost.setOnTabChangedListener(this);

		Intent intent = getIntent();
		//mTabHost.setCurrentTab(intent.getIntExtra(OpenScreen.EXTRA_TAB_NAME, 0));

		mSendTimer = new CountDownTimer(mSendDelay, mSendDelay) {
			public void onTick(long millisUntilFinished) {
			}

			public void onFinish() {
				mSendTimeElapsed = true;
				
			}
		};

		// Get the input method manager for hiding the soft keyboard
		mgr = (InputMethodManager) getApplicationContext().getSystemService(
				Context.INPUT_METHOD_SERVICE);

		// Setup toasts
		mToast = Toast.makeText(this, "", Toast.LENGTH_SHORT);

		ActivityManager.MemoryInfo memoryInfo= getAvailableMemory();
		Log.d(TAG, String.valueOf(memoryInfo.totalMem));

		//			File audioFile = new File("audio_data.raw");
//			audioFile.createNewFile();
//			fileOutputStream = openFileOutput(AUDIO_DATA_FILE,	Context.MODE_PRIVATE);
		BYTES_IN_FILE = 0;
		File root = Environment.getExternalStorageDirectory();
		fileRootPath = root.getAbsolutePath() + "/AudioFiles";
		audioFileDir = new File (root.getAbsolutePath() + "/AudioFiles");
		audioFileDir.mkdir();
		file1 = new File(audioFileDir, AUDIOFILENAME1);
		file2 = new File(audioFileDir, AUDIOFILENAME2);
		fileDebug = new File(audioFileDir, AUDIODEBUGFILENAME);
		try {
			fileOutputStream1 = new FileOutputStream(file1);
			fileOutputStream2 = new FileOutputStream(file2);
            fileOutputStreamDebug = new FileOutputStream(fileDebug);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}
	private String fileRootPath;
	private static final String AUDIOFILENAME1 = "audio_data_sd1.raw";
	private static final String AUDIOFILENAME2 = "audio_data_sd2.raw";
	private static final String AUDIODEBUGFILENAME = "audio_debug.raw";

	private File audioFileDir;
	private File file1;
	private File file2;
	private File fileDebug;
	// Called when the app becomes visible to the user. Checks if Bluetooth is
	// enabled and initializes the Bluetooth service.
	@Override
	public void onStart() {
		super.onStart();
		if (D)
			Log.e(TAG, "++ ON START ++");
		// Prepare Cloud Speech API
		bindService(new Intent(this, SpeechService.class), mServiceConnection, BIND_AUTO_CREATE);
		
		// If BT is not on, request that it be enabled.
		// setupBTService() will then be called during onActivityResult
		if (!mBluetoothAdapter.isEnabled()) {
			Intent enableIntent = new Intent(
					BluetoothAdapter.ACTION_REQUEST_ENABLE);
			startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
			// Otherwise, setup the Bluetooth session
		} else {
			if (mBluetoothService == null)
				setupBTService();
		}

	}

	// Called when the user can start interacting with the app. Starts the
	// Bluetooth service.
	@Override
	public synchronized void onResume() {
		super.onResume();
		if (D)
			Log.e(TAG, "+ ON RESUME +");

		// Performing this check in onResume() covers the case in which BT was
		// not enabled during onStart(), so we were paused to enable it...
		// onResume() will be called when ACTION_REQUEST_ENABLE activity
		// returns.
		if (mBluetoothService != null) {
			// Only if the state is STATE_NONE, do we know that we haven't
			// started already
			if (mBluetoothService.getState() == BluetoothService.STATE_NONE) {
				// Start the Bluetooth service
				mBluetoothService.start();
			}

			// if onResume is called after device list activity
			// do not reconnect.
			if (!backFromDeviceList) {
				// Reconnect last connected device
				if (mBluetoothService.getState() != BluetoothService.STATE_CONNECTED) {
					reconnect();
				}
			} else {
				backFromDeviceList = false;
			}
		}
	}

	// auto reconnect to last connected device
	private void reconnect() {
		if(mLastConnectedDeviceAddr == null )
			mLastConnectedDeviceAddr = mSharedPrefSettings.getString(DEVICE_ADDR, null);

		if (mLastConnectedDeviceAddr != null) {
			final BluetoothDevice device = mBluetoothAdapter
					.getRemoteDevice(mLastConnectedDeviceAddr);
			if (device == null) {
				Log.d(TAG, "no last connected device");
				return;
			}
			Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
			// If last connected device is still a paired device, reconnect
			// otherwise, do not reconnect.
			if(pairedDevices.contains(device))
				mBluetoothService.connect(device, false);
		}
		
	}

	// Called when the user starts another activity, such as by navigating away
	// from the app.
	@Override
	public synchronized void onPause() {
		super.onPause();
		if (D)
			Log.e(TAG, "- ON PAUSE -");
	}

	// Called when the app is no longer visible to the user.
	@Override
	public void onStop() {
		super.onStop();
		if (D)
			Log.e(TAG, "-- ON STOP --");

		// Stop Cloud Speech API
		//finishRecognition();

	}



	public Handler getAppHandler()
	{
		return mHandler;
	}

	// Called when the app is destroyed. Stops the Bluetooth service.
	@Override
	public void onDestroy() {
		super.onDestroy();
		finishRecognition();
		if(mSpeechService != null)
		{
			mSpeechService.removeListener(mSpeechServiceListener);
			unbindService(mServiceConnection);
			mSpeechService = null;
		}
		// if onDestroy is called because of starting file browser activity
		// do not stop bluetoothservice

		// Stop the Bluetooth service
		if (mBluetoothService != null)
			mBluetoothService.stop();
		if (D)
			Log.e(TAG, "--- ON DESTROY ---");

	}


	// Get a MemoryInfo object for the device's current memory status.
	private ActivityManager.MemoryInfo getAvailableMemory() {
		ActivityManager activityManager = (ActivityManager) this.getSystemService(ACTIVITY_SERVICE);
		ActivityManager.MemoryInfo memoryInfo = new ActivityManager.MemoryInfo();
		activityManager.getMemoryInfo(memoryInfo);
		return memoryInfo;
	}

	// Initializes the Bluetooth interface
	private void setupBTService() {
//		Log.i(TAG, "SET UP Bluetooth");
		// Initialize the array adapter for the conversation thread
		mConversationArrayAdapter = new ArrayAdapter<String>(this,
				R.layout.message);

		// Initialize the BluetoothService to perform bluetooth connections
		mBluetoothService = new BluetoothService(this, mHandler);
	}

	// Sends a message over Bluetooth.
	// @param message A string of text to send.
	private void sendMessage(String message) {
		// If we're not connected, ignore the request.
		if (mBluetoothService.getState() != BluetoothService.STATE_CONNECTED) {
			Log.i("sendMessage", "NotConnected");
			mToast.setText(R.string.not_connected);
			mToast.setDuration(Toast.LENGTH_SHORT);
			mToast.show();
			return;
		}

		// If the send delay hasn't elapsed since sending the last command,
		// ignore the request.
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
			} catch (NullPointerException e) {
			}
		}
	}

	// Sets the subtitle in the actionbar to a string, using a resource id as a
	// lookup. Mostly used to show Bluetooth
	// connection status.
	private final void setStatus(int resId) {
		toolbar.setSubtitle(resId);
	}

	// Sets the subtitle in the actionbar to a string, using a character
	// sequence. Mostly used to show Bluetooth
	// connection status.
	private final void setStatus(CharSequence subTitle) {
		toolbar.setSubtitle(subTitle);
	}
	private final static char[] hexArray = "0123456789ABCDEF".toCharArray();
	private static String bytesToHex(byte[] bytes, int size) {
		char[] hexChars = new char[size * 2];
		for ( int j = 0; j < size; j++ ) {
			int v = bytes[j] & 0xFF;
			hexChars[j * 2] = hexArray[v >>> 4];
			hexChars[j * 2 + 1] = hexArray[v & 0x0F];
		}
		return new String(hexChars);
	}
//	private byte[] audioBufferedData1 = new byte[1280];
//	private byte[] audioBufferedData2 = new byte[1280];
//	private boolean fillingBuffer1 = true;
	private static int BYTES_IN_FILE = 0;
	// The Handler that gets information back from the BluetoothService. Called
	// when any of the following happens:
	// Bluetooth connection state changes. Updates the status and/or name of the
	// connected device in the action bar. Sometimes
	// sends additional information, which is displayed as a pop up message.
	// A message was sent over Bluetooth. Prints the message to the
	// transmit/receive log.
	// A message was received over Bluetooth. Prints the message to the
	// transmit/receive log. Also parses it to see if it was
	// a temperature update.
	private final Handler mHandler = new Handler() {
		@Override
		public void handleMessage(Message msg) {
			switch (msg.what) {
			case MESSAGE_STATE_CHANGE:
				if (D)
					Log.i(TAG, "MESSAGE_STATE_CHANGE: " + msg.arg1);
				switch (msg.arg1) {
				case BluetoothService.STATE_CONNECTED:
					setStatus(getString(R.string.title_connected_to,
							mConnectedDeviceName));
					try {
						mConversationArrayAdapter.clear();
					} catch (NullPointerException e) {
					}
					break;
				case BluetoothService.STATE_CONNECTING:
					setStatus(R.string.title_connecting);
					break;
				case BluetoothService.STATE_LISTEN:
				case BluetoothService.STATE_NONE:
					setStatus(R.string.title_not_connected);
					finishRecognition();
					break;
				}
				break;
			case MESSAGE_WRITE:
				byte[] writeBuf = (byte[]) msg.obj;
				// construct a string from the buffer
				String writeMessage = new String(writeBuf);

				try {
					mConversationArrayAdapter.add("TX: " + writeMessage);
				} catch (NullPointerException e) {
				}
				break;
			case MESSAGE_READ:
				byte[] readBuf = (byte[]) msg.obj;
				dataReceived += 4*msg.arg1;

				byte[] uncompress = decompressADPCM(readBuf, msg.arg1);

//				if(bytesInFileDebug < 32000*1) // 1s recording
//                {
//                    try {
//                        fileOutputStreamDebug.write(uncompress,0,4*msg.arg1);
//                        bytesInFileDebug+=4*msg.arg1;
//                    } catch (IOException e) {
//                        e.printStackTrace();
//                    }
//                }else
//                {
//
//                    try {
//                        fileOutputStreamDebug.close();
//                    } catch (IOException e) {
//                        e.printStackTrace();
//                    }
//                    Log.d("FileDebug", "Done");
//
//                }


				String readMessage = "RX Size: "+String.valueOf(4*msg.arg1)+"Bytes";
				Log.d("VOICE Data:",readMessage);
				//mConversationArrayAdapter.add( readMessage);
				try {
					if(BYTES_IN_FILE < BYTES_PER_SECOND*BUFFER_TIME)
					{
						if(STREAM2FILE1)
						{
							if(fileOutputStream1!=null) // means file1 is released by speech service
								fileOutputStream1.write(uncompress,0,4*msg.arg1);

						}else
						{
							if(fileOutputStream2!=null)
							{
								fileOutputStream2.write(uncompress,0,4*msg.arg1);
							}

						}
					}

					BYTES_IN_FILE+=4*msg.arg1;
					if(BYTES_IN_FILE >= BYTES_PER_SECOND*BUFFER_TIME) // BUFFER_TIME seconds voice recording
					{

						if(STREAM2FILE1)
						{
							if(fileOutputStream1 != null) // lost  data
							{
								fileOutputStream1.close();
								InputStream inputStream = new FileInputStream(file1);
								// send file1 to speech api
								mSpeechService.recognizeInputStream(inputStream);
//									mSpeechService.recognizeBytesStream(voiceBuffer1);
								fileOutputStream1 = null; // make it to null
								STREAM2FILE1 = false;
							}
//								mSpeechService.recognizeBytesStream(voiceBuffer1);
//								STREAM2FILE1 = false;
						}else
						{

							if(fileOutputStream2 != null ) // lost BUFFER_TIME seconds data
							{
								fileOutputStream2.close();
								InputStream inputStream = new FileInputStream(file2);
								// send file1 to speech api
								mSpeechService.recognizeInputStream(inputStream);
								// make it to null, only after speech service release this handle,
								// this pointer will be recreated again.
								fileOutputStream2 = null;
								STREAM2FILE1 = true;
							}
//								mSpeechService.recognizeBytesStream(voiceBuffer2);
//								STREAM2FILE1 = true;
						}
						if(fileOutputStream1==null && fileOutputStream2==null)
						{
							// speech api callback is not triggered properly
							// force two file handlers reopen.
							fileOutputStream1 = new FileOutputStream(file1);
							fileOutputStream2 = new FileOutputStream(file2);
						}
						// ready to recognize
						BYTES_IN_FILE = 0;
					}
				}
				catch (IOException e) {
					e.printStackTrace();
				}


				break;
			case MESSAGE_DEVICE_NAME:
				// save the connected device's name
				mConnectedDeviceName = msg.getData().getString(DEVICE_NAME);
				mLastConnectedDeviceAddr = msg.getData().getString(DEVICE_ADDR);

				if (D)
					Log.d(TAG, "connect to " + mLastConnectedDeviceAddr);

				// save last connected device address for reconnecting
				mPrefEditor.putString(DEVICE_ADDR, mLastConnectedDeviceAddr);
				mPrefEditor.commit();

				Toast.makeText(getApplicationContext(),
						"Connected to " + mConnectedDeviceName,
						Toast.LENGTH_SHORT).show();
				break;
			case MESSAGE_TOAST:
				Toast.makeText(getApplicationContext(),
						msg.getData().getString(TOAST), Toast.LENGTH_SHORT)
						.show();
				break;
			}
		}
	};

	// Called when any of the following happens:
	// The user has selected a Bluetooth device to connect with. Connects with
	// that device.
	// The app has requested that Bluetooth has been enabled on our device.
	// Initialize Bluetooth service if it was successful,
	// exit the app otherwise
	public void onActivityResult(int requestCode, int resultCode, Intent data) {
		if (D)
			Log.d(TAG, "onActivityResult " + resultCode);
		switch (requestCode) {
		case REQUEST_CONNECT_DEVICE:
			
			// Connect device insecure
			if (resultCode == Activity.RESULT_OK) {
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
				Toast.makeText(this, R.string.bt_not_enabled_leaving,
						Toast.LENGTH_SHORT).show();
				finish();
			}
			break;

		}
	}

	boolean backFromDeviceList = false;

	// Connects to the requested device over bluetooth
	private void connectDevice(Intent data, boolean secure) {
		backFromDeviceList = true;
		
		// Get the device MAC address
		String address = data.getExtras().getString(
				DeviceListActivity.EXTRA_DEVICE_ADDRESS);
		
		// Get the BluetoothDevice object
		BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
		if(mBluetoothService.getState() == BluetoothService.STATE_CONNECTED)
		{
			if(mLastConnectedDeviceAddr.equals(address))
			{
				Log.d(TAG, "already connected to bt");
				Toast.makeText(this, "already connected to " + device.getName(),
						Toast.LENGTH_SHORT).show();
				return;
			}
			
		}
		
		
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
		} else if (itemId == R.id.connect_scan) {

			serverIntent = new Intent(this, DeviceListActivity.class);
			startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
			return true;
		} else if (itemId == R.id.about) {
			// Show the About dialog
			showAbout();
			return true;
		}
		return false;
	}

	// Called when the user navigates away from the app. Saves the state of the
	// app so it can be restored
	// when the user comes back to it.
	@Override
	public void onSaveInstanceState(Bundle outState) {
		super.onSaveInstanceState(outState);

		return;
	}

	// Called when each tab is selected for the first time.
	@Override
	public void onAttachFragment(Fragment fragment) {
		super.onAttachFragment(fragment);
		if (fragment.getClass().equals(TextFragment.class)) {
			textFrag = (TextFragment) fragment;
		} else if (fragment.getClass().equals(SpeechCMDFragment.class)) {
			speechCmdFrag = (SpeechCMDFragment) fragment;
		}else if(fragment.getClass().equals(DataRateFragment.class))
		{
			dataRateFrag = (DataRateFragment)fragment;
		}
	}

	// Called when the user switches tabs. Hides the soft keyboard.
	@Override
	public void onTabChanged(String tabId) {
		try {
			textFrag.hideKeyboard(mgr);
		} catch (NullPointerException e) {
		}
	}

	// Called when the user selects About.
	public void showAbout() {
		// Get Version Name and Display in About dialog
		Context context = getApplicationContext();
		try {
			String versionName = context.getPackageManager().getPackageInfo(
					getApplicationContext().getPackageName(), 0).versionName;

			DialogFragment newFragment = new AboutDialogFragment(versionName);
			newFragment.show(getSupportFragmentManager(), "about");
		} catch (NameNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}
