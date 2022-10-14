using System;
using System.Linq;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;

using JointTrajectory = RosMessageTypes.Trajectory.JointTrajectoryMsg;
using JointTrajectoryPoint = RosMessageTypes.Trajectory.JointTrajectoryPointMsg;
using RosHeader = RosMessageTypes.Std.HeaderMsg;



public class testwalk : MonoBehaviour
{
    public Transform targetBody;
    public Transform targetFL;
    public Transform targetFR;
    public Transform targetRL;
    public Transform targetRR;

    public Slider sli1;

    private float L1;
    private float L2;
    private float L3;

    float offset_x;
    float LegL2;
    float LegL3;

    public Slider[] sliders;
    public InputField[] inputs;


    public Transform traBody;
    public Transform[] tra_joints;
    public Transform traFLFoot;

    private string topicName = "joint_group_position_controller/command";

    // ROS Connector
    private ROSConnection ros;
    private JointTrajectory joiTraMsg = new JointTrajectory();





    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointTrajectory>(topicName);


        offset_x = tra_joints[1].localPosition.x + tra_joints[2].localPosition.x + traFLFoot.localPosition.x;
        LegL2 = tra_joints[2].localPosition.y;
        LegL3 = traFLFoot.localPosition.y;


        float[] sendjointdata = new float[12];
        for (int a=0; a<12; a++)
        {
            if ((a % 3) == 0)
            {
                sendjointdata[a] = tra_joints[a].localRotation.eulerAngles.z;
            }
            else
            {
                sendjointdata[a] = tra_joints[a].localRotation.eulerAngles.x;
            }
        }

        SetMsgVal(sendjointdata);
        Publish();




        L1 = tra_joints[1].position.x - tra_joints[0].position.x;
        L2 = tra_joints[2].position.y - tra_joints[1].position.y;
        L3 = traFLFoot.position.y - tra_joints[2].position.y;

    }


    public void SetMsgVal(float[] jointdata)
    {
        joiTraMsg = new JointTrajectory
        {
            header = new RosHeader { frame_id = "0" },
            points = new JointTrajectoryPoint[1],
        };
        joiTraMsg.joint_names = new string[12] { "base_lf1", "lf1_lf2", "lf2_lf3", "base_rf1", "rf1_rf2", "rf2_rf3", "base_lb1", "lb1_lb2", "lb2_lb3", "base_rb1", "rb1_rb2", "rb2_rb3" };
        joiTraMsg.points[0] = new JointTrajectoryPoint();
        //joiTraMsg.points[0].positions = new double[] { -0.015f, 0.8407264947891235f, -1.8289316892623901f, 0.07310621440410614f, 0.8409429788589478f, -1.8293825387954712f, -0.05534367263317108f, 0.5353963971138f, -1.2529590129852295f, 0.05534370243549347f, 0.5353963971138f, -1.2529590129852295f };

        joiTraMsg.points[0].positions = new double[12];
        for (int a = 0; a < 12; a++)
        {
            if ((a % 3) == 0)
            {
                joiTraMsg.points[0].positions[a] = (double)-jointdata[a];
            }
            else
            {
                joiTraMsg.points[0].positions[a] = (double)jointdata[a];
            }
        }

        joiTraMsg.points[0].velocities = new double[12];
        joiTraMsg.points[0].accelerations = new double[12];
        joiTraMsg.points[0].effort = new double[12];
        joiTraMsg.points[0].time_from_start = new DurationMsg(){ nanosec = 20000000 };
    }



    public void Publish()
    {
        ros.Publish(topicName, joiTraMsg);
    }

    public void slidarMove()
    {
        joiTraMsg = new JointTrajectory
        {
            header = new RosHeader { frame_id = "0" },
            points = new JointTrajectoryPoint[1],
        };
        joiTraMsg.joint_names = new string[12] { "base_lf1", "lf1_lf2", "lf2_lf3", "base_rf1", "rf1_rf2", "rf2_rf3", "base_lb1", "lb1_lb2", "lb2_lb3", "base_rb1", "rb1_rb2", "rb2_rb3" };
        joiTraMsg.points[0] = new JointTrajectoryPoint();
        joiTraMsg.points[0].positions = new double[] { -0.015f, 0.8407264947891235f, -1.8289316892623901f, 0.07310621440410614f, 0.8409429788589478f, -1.8293825387954712f, -0.05534367263317108f, 0.5353963971138f, -1.2529590129852295f, 0.05534370243549347f, 0.5353963971138f, -1.2529590129852295f };
        joiTraMsg.points[0].time_from_start = new DurationMsg() { nanosec = 20000000 };


        joiTraMsg.points[0].positions[0] = sli1.value;        


    }


    // Update is called once per frame
    void FixedUpdate()
    {

        float[] sendjointdata = new float[12];

        traBody.position = targetBody.position;
        traBody.rotation = targetBody.rotation;

        Vector3 tpos = Quaternion.Inverse(targetBody.rotation) * (targetFL.position - targetBody.position) - tra_joints[0].localPosition;
        //tpos.x = -tpos.x;
        float[] jointangledata = LegIK_MiniPupper(tpos);
        tra_joints[0].localRotation = Quaternion.Euler(new Vector3(0.0f, 0.0f, jointangledata[0] * 180.0f / Mathf.PI));
        tra_joints[1].localRotation = Quaternion.Euler(new Vector3(jointangledata[1] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        tra_joints[2].localRotation = Quaternion.Euler(new Vector3(jointangledata[2] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        sendjointdata[0] = jointangledata[0];
        sendjointdata[1] = jointangledata[1];
        sendjointdata[2] = jointangledata[2];



        tpos = Quaternion.Inverse(targetBody.rotation) * (targetFR.position - targetBody.position) - tra_joints[3].localPosition;
        tpos.x = -tpos.x;
        jointangledata = LegIK_MiniPupper(tpos); ;
        tra_joints[3].localRotation = Quaternion.Euler(new Vector3(0.0f, 0.0f, -jointangledata[0] * 180.0f / Mathf.PI));
        tra_joints[4].localRotation = Quaternion.Euler(new Vector3(jointangledata[1] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        tra_joints[5].localRotation = Quaternion.Euler(new Vector3(jointangledata[2] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        sendjointdata[3] = -jointangledata[0];
        sendjointdata[4] = jointangledata[1];
        sendjointdata[5] = jointangledata[2];


        tpos = Quaternion.Inverse(targetBody.rotation) * (targetRL.position - targetBody.position) - tra_joints[6].localPosition;
        //tpos.x = -tpos.x;
        jointangledata = LegIK_MiniPupper(tpos);
        tra_joints[6].localRotation = Quaternion.Euler(new Vector3(0.0f, 0.0f, jointangledata[0] * 180.0f / Mathf.PI));
        tra_joints[7].localRotation = Quaternion.Euler(new Vector3(jointangledata[1] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        tra_joints[8].localRotation = Quaternion.Euler(new Vector3(jointangledata[2] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        sendjointdata[6] = jointangledata[0];
        sendjointdata[7] = jointangledata[1];
        sendjointdata[8] = jointangledata[2];


        tpos = Quaternion.Inverse(targetBody.rotation) * (targetRR.position - targetBody.position) - tra_joints[9].localPosition;
        tpos.x = -tpos.x;
        jointangledata = LegIK_MiniPupper(tpos); ;
        tra_joints[9].localRotation = Quaternion.Euler(new Vector3(0.0f, 0.0f, -jointangledata[0] * 180.0f / Mathf.PI));
        tra_joints[10].localRotation = Quaternion.Euler(new Vector3(jointangledata[1] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        tra_joints[11].localRotation = Quaternion.Euler(new Vector3(jointangledata[2] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        sendjointdata[9] = -jointangledata[0];
        sendjointdata[10] = jointangledata[1];
        sendjointdata[11] = jointangledata[2];


        SetMsgVal(sendjointdata);
        Publish();
    }


    public float[] LegIK_MiniPupper(Vector3 fpos)
    {
        float[] res = new float[3];


        float qang1 = Vector3.SignedAngle(new Vector3(0.0f, -1.0f, 0.0f), new Vector3(fpos.x, fpos.y, 0.0f), new Vector3(0.0f, 0.0f, 1.0f));
        float Distance_L1toTP_xy = Vector3.Distance(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(fpos.x, fpos.y));
        float Distance_L1toTP_xy_y = Mathf.Sqrt(Mathf.Pow(Distance_L1toTP_xy, 2.0f) - Mathf.Pow(offset_x, 2.0f));
        float qang2 = Vector3.SignedAngle(new Vector3(0.0f, -1.0f, 0.0f), new Vector3(offset_x, -Distance_L1toTP_xy_y, 0.0f), new Vector3(0.0f, 0.0f, 1.0f));

        res[0] = (qang1- qang2) / 180.0f * Mathf.PI;


        Vector3 newfpos = Quaternion.AngleAxis(-((qang1 - qang2)), new Vector3(0.0f, 0.0f, 1.0f)) * fpos;
        float Distance_L2toTP_yz = Vector3.Distance(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(newfpos.z, newfpos.y));

        res[2] = Mathf.Acos((Mathf.Pow(LegL2, 2.0f) + Mathf.Pow(LegL3, 2.0f) - Mathf.Pow(Distance_L2toTP_yz, 2.0f)) / (2.0f * LegL2 * LegL3)) - Mathf.PI;


        float newTptoP2toTPy = Mathf.Atan2(newfpos.z, newfpos.y);
        float cosP3toP2tonewTP = (Mathf.Pow(L2, 2.0f) + Mathf.Pow(Distance_L2toTP_yz, 2.0f) - Mathf.Pow(L3, 2.0f)) / (2.0f * L2 * Distance_L2toTP_yz);
        float P3toP2tonewTP = Mathf.Acos(cosP3toP2tonewTP);

        res[1] = newTptoP2toTPy - P3toP2tonewTP;


        //Angle of the second link relative to the tilted negative z axis
        float knee_angle = res[1] - (Mathf.PI - res[2]) + Mathf.PI*2;

        //Debug.Log((knee_angle*180.0f/Mathf.PI).ToString());

        return res;
    }

}
