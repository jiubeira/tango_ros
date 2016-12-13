package eu.intermodalics.tangoxros;

import android.util.Log;

import org.ros.namespace.GraphName;


/**
 * Created by juan on 13/12/16.
 */


public class NativeNodeTest extends NativeNodeMain {

    private static final String nodeName = "NativeNodeTest";
    private static final String libraryName = "native_node_test";
    private static final java.lang.String TAG = NativeNodeTest.class.getSimpleName();

    public NativeNodeTest() {
        super(libraryName);
        Log.i(TAG, "Native node test created");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeName);
    }

    @Override
    public native void execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);

    @Override
    public native void shutdown();

}
