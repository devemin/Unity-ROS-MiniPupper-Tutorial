using System.Collections;
using System.Collections.Generic;
using UnityEngine;


using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;


using PoseMessage = RosMessageTypes.Geometry.PoseMsg;
using TwistMessage = RosMessageTypes.Geometry.TwistMsg;
using RosHeader = RosMessageTypes.Std.HeaderMsg;




public class JoyCommandTest : MonoBehaviour
{
    public float speed;
    public VariableJoystick vjoy1;
    public VariableJoystick vjoy2;




    // 送信するROSのトピック名
    private string topicName_cmdvel = "/cmd_vel";
    private string topicName_bodypose = "/body_pose";



    // ROS Connector
    private ROSConnection ros;

    private TwistMessage tmes = new TwistMessage();
    private PoseMessage pmes = new PoseMessage();




    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMessage>(topicName_cmdvel);
        ros.RegisterPublisher<PoseMessage>(topicName_bodypose);


        float[] senddata = new float[13];
        for (int a = 0; a < 13; a++)
        {
            senddata[a] = 0.0f;
        }

        SetMsgVal(senddata);
        Publish();
    }





    public void SetMsgVal(float[] jointdata)
    {
        float[] senddata = new float[13];


        tmes.linear.x = vjoy1.Vertical;
        tmes.linear.y = -vjoy1.Horizontal;
        tmes.linear.z = 0.0f;

        tmes.angular.x = 0.0f;
        tmes.angular.y = 0.0f;
        tmes.angular.z = 0.0f;


        Quaternion tmpquat = Quaternion.Euler(new Vector3(vjoy2.Horizontal * 45.0f, vjoy2.Vertical * 45.0f, 0.0f));

        pmes.position.x = 0.0f;
        pmes.position.y = 0.0f;
        pmes.position.z = 0.0f;

        pmes.orientation.x = tmpquat.x;
        pmes.orientation.y = tmpquat.y;
        pmes.orientation.z = tmpquat.z;
        pmes.orientation.w = tmpquat.w;



    }



    public void Publish()
    {
        ros.Publish(topicName_cmdvel, tmes);
        ros.Publish(topicName_bodypose, pmes);
    }



    // Update is called once per frame
    void FixedUpdate()
    {


        float[] senddata = new float[13];
        for (int a = 0; a < 13; a++)
        {
            senddata[a] = 0.0f;
        }

        SetMsgVal(senddata);
        Publish();
    }




}
