package eu.intermodalics.tangoxros;

import android.app.Activity;
import android.app.FragmentManager;
import android.content.ComponentName;
import android.content.Context;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.IBinder;
import android.text.format.Formatter;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.CompoundButton;
import android.widget.Spinner;
import android.widget.Switch;

public class MainActivity extends Activity implements SetMasterUriDialog.CallbackListener, AdapterView.OnItemSelectedListener {
    private static final String TAG = MainActivity.class.getSimpleName();
    private static final String MASTER_URI_PREFIX = "__master:=";
    private static final String IP_PREFIX = "__ip:=";

    private JNIInterface mJniInterface;
    private String mMasterUri;
    private boolean mIsInitialised = false;

    private boolean mPublishDevicePose = false;
    private boolean mPublishPointCloud = false;
    private String mPublishCamera = "None";

    /**
     * Implements SetMasterUriDialog.CallbackListener.
     */
    @Override
    public void onMasterUriConnect(String uri) {
        mMasterUri = uri;
        // Save URI preference.
        SharedPreferences sharedPref = this.getPreferences(Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPref.edit();
        editor.putString(getString(R.string.saved_uri), mMasterUri);
        editor.commit();
        // Start ROS and node.
        init();
        onResume();
    }

    /**
     * Shows a dialog for setting the Master URI.
     */
    private void showSetMasterUriDialog() {
        // Get URI preference or default value if does not exist.
        SharedPreferences sharedPref = this.getPreferences(Context.MODE_PRIVATE);
        String defaultUriValue = getResources().getString(R.string.saved_uri_default);
        String uriValue = sharedPref.getString(getString(R.string.saved_uri), defaultUriValue);

        Bundle bundle = new Bundle();
        bundle.putString(getString(R.string.saved_uri), uriValue);
        FragmentManager manager = getFragmentManager();
        SetMasterUriDialog setMasterUriDialog = new SetMasterUriDialog();
        setMasterUriDialog.setArguments(bundle);
        setMasterUriDialog.show(manager, "MatserUriDialog");
    }

    /**
     * Implements AdapterView.OnItemSelectedListener callbacks.
     */
    public void onItemSelected(AdapterView<?> parent, View view,
                               int pos, long id) {
        mPublishCamera = parent.getItemAtPosition(pos).toString();
        Log.w(TAG, "Publish camera is " + mPublishCamera);
    }

    public void onNothingSelected(AdapterView<?> parent) {
        // Another interface callback
    }

    /**
     * Implements callback for Apply button.
     */
    public void applySettings(View view) {
        onPause();
        mJniInterface.initNode(this, mPublishDevicePose, mPublishPointCloud, mPublishCamera);
        mIsInitialised = true;
        onResume();
    }

    /**
     * Tango Service connection.
     */
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            // Synchronization around MeshBuilderActivity object is to avoid
            // Tango disconnect in the middle of the connecting operation.
            mJniInterface.onTangoServiceConnected(service);
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    public void init() {
        if (mMasterUri != null) {
            WifiManager wm = (WifiManager) getSystemService(WIFI_SERVICE);
            String ip_address = Formatter.formatIpAddress(wm.getConnectionInfo().getIpAddress());
            if (mJniInterface.initRos(MASTER_URI_PREFIX + mMasterUri, IP_PREFIX + ip_address)) {
                mJniInterface.initNode(this, mPublishDevicePose, mPublishPointCloud, mPublishCamera);
                mIsInitialised = true;
            } else {
                Log.e(TAG, "Unable to init ROS!");
            }
        } else {
            Log.e(TAG, "Master URI is null");
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_activity);
        // Set callback for device pose switch.
        Switch switchDevicePose = (Switch) findViewById(R.id.switch_device_pose);
        switchDevicePose.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    mPublishDevicePose = true;
                    Log.i(TAG, "Publish device pose is switched on");
                } else {
                    mPublishDevicePose = false;
                    Log.i(TAG, "Publish device pose i switched off");
                }
            }
        });
        // Set callback for point cloud switch.
        Switch switchPointCloud = (Switch) findViewById(R.id.switch_pointcloud);
        switchPointCloud.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    mPublishPointCloud = true;
                    Log.i(TAG, "Publish point cloud is switched on");
                } else {
                    mPublishPointCloud = false;
                    Log.i(TAG, "Publish point cloud is switched off");
                }
            }
        });
        // Set list of choices and callback for camera spinner.
        Spinner spinner = (Spinner) findViewById(R.id.cameras_spinner);
        // Create an ArrayAdapter using the string array and a default spinner layout
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
                R.array.cameras, android.R.layout.simple_spinner_item);
        // Specify the layout to use when the list of choices appears
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinner.setAdapter(adapter);
        spinner.setOnItemSelectedListener(this);
        // Request master URI from user.
        showSetMasterUriDialog();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mIsInitialised) {
            TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
            new Thread(new Runnable() {
                @Override
                public void run() {
                    while (mJniInterface.isRosOk()) {
                        mJniInterface.publish();
                    }
                }
            }).start();
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mIsInitialised) {
            mJniInterface.tangoDisconnect();
            unbindService(mTangoServiceConnection);
        }
    }
}