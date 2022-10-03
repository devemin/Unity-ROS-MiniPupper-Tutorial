using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;

using System;
using System.Linq;
using System.Text;




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


    // 送信するROSのトピック名
    private string topicName = "joint_group_position_controller/command";

    // 送信する速度指令の基準となる値（各関数でこの速度に所定の倍率をかける）
    private float linearVel;

    // 送信する角速度指令の基準となる値（各関数でこの角速度に所定の倍率をかける）
    private float angularVel;

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
        //本体のservo_interface との違いにより、関節１のマイナス付与

        joiTraMsg.points[0].velocities = new double[12];
        joiTraMsg.points[0].accelerations = new double[12];
        joiTraMsg.points[0].effort = new double[12];
        joiTraMsg.points[0].time_from_start = new DurationMsg(){ nanosec = 20000000 };
    }



    public void Publish()
    {
        // ROSのros_tcp_endpointのdefault_server_endpoint.pyにメッセージを送信
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


        //ros.Publish(topicName, joiTraMsg);

        /*
        var foot1 = this.tra_joints[0].xDrive;
        foot1.target = sli1.value * 180.0f / Mathf.PI;
        this.tra_joints[0].xDrive = foot1;
        */

    }


    // Update is called once per frame
    void FixedUpdate()
    {
        /*
        if (frameupflag)
        {
            framecount+=5;
        }
        else
        {
            framecount-=5;
        }
        if (framecount >= 0)
        {
            frameupflag = false;
        }
        else if (framecount <= -90)
        {
            frameupflag = true;
        }

        for (int a=0; a<12; a++)
        {
            if ((a % 3) == 1)
            {
                var foot1 = this.tra_joints[a].xDrive;
                foot1.target = -(float)framecount-15.0f;
                this.tra_joints[a].xDrive = foot1;
            }
            else if ((a % 3) == 2)
            {
                var foot1 = this.tra_joints[a].xDrive;
                foot1.target = (float)framecount-30.0f  ;
                this.tra_joints[a].xDrive = foot1;
            }
        }
        */




        /*
        float[,] targetpos = { 
            { targetFL.transform.localPosition.x, targetFR.transform.localPosition.x, targetRL.transform.localPosition.x, targetRR.transform.localPosition.x },
            { targetFL.transform.localPosition.y, targetFR.transform.localPosition.y, targetRL.transform.localPosition.y, targetRR.transform.localPosition.y }, 
            { targetFL.transform.localPosition.z, targetFR.transform.localPosition.z, targetRL.transform.localPosition.z, targetRR.transform.localPosition.z } };

        float[,] anglelist = four_legs_inverse_kinematics(targetpos);

        string test = "";
        for (int a = 0; a < 3; a++)
        {
            for (int b = 0; b < 4; b++)
            {
                test = test + anglelist[a,b].ToString() + ", ";
            }
        }
        Debug.Log(test);


        for (int a = 0; a < 3; a++)
        {
            for (int b = 0; b < 4; b++)
            {
                var foot1 = this.tra_joints[(3*b)+a].xDrive;
                foot1.target = (float)((anglelist[a,b]) * 180.0f / Mathf.PI);
                //Debug.Log(a.ToString() + ", " + b.ToString());
                this.tra_joints[(3*b) + a].xDrive = foot1;
            }
        }
        */



        /*
        var foot0 = this.tra_joints[0].xDrive;
        var foot1 = this.tra_joints[1].xDrive;
        var foot2 = this.tra_joints[2].xDrive;
        Vector3 tpos = targetFL.transform.position - RobotPos.transform.position - JointPosFL1.transform.localPosition;

        float[] jointangledata = MiniPupper_LegIK(tpos);

        foot0.target = jointangledata[0] * 180.0f / Mathf.PI;
        this.tra_joints[0].xDrive = foot0;
        foot1.target = jointangledata[1] * 180.0f / Mathf.PI;
        this.tra_joints[1].xDrive = foot1;
        foot2.target = jointangledata[2] * 180.0f / Mathf.PI;
        this.tra_joints[2].xDrive = foot2;


        */

        //targetFR..positon = tpos;

        //Debug.Log("fixedupdate.");


        float[] sendjointdata = new float[12];

        traBody.position = targetBody.position;
        traBody.rotation = targetBody.rotation;

        Vector3 tpos = Quaternion.Inverse(targetBody.rotation) * (targetFL.position - targetBody.position) - tra_joints[0].localPosition;
        //tpos.x = -tpos.x;
        float[] jointangledata = LegIK2(tpos);
        tra_joints[0].localRotation = Quaternion.Euler(new Vector3(0.0f, 0.0f, jointangledata[0] * 180.0f / Mathf.PI));
        tra_joints[1].localRotation = Quaternion.Euler(new Vector3(jointangledata[1] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        tra_joints[2].localRotation = Quaternion.Euler(new Vector3(jointangledata[2] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        sendjointdata[0] = jointangledata[0];
        sendjointdata[1] = jointangledata[1];
        sendjointdata[2] = jointangledata[2];



        tpos = Quaternion.Inverse(targetBody.rotation) * (targetFR.position - targetBody.position) - tra_joints[3].localPosition;
        tpos.x = -tpos.x;
        jointangledata = LegIK2(tpos); ;
        tra_joints[3].localRotation = Quaternion.Euler(new Vector3(0.0f, 0.0f, -jointangledata[0] * 180.0f / Mathf.PI));
        tra_joints[4].localRotation = Quaternion.Euler(new Vector3(jointangledata[1] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        tra_joints[5].localRotation = Quaternion.Euler(new Vector3(jointangledata[2] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        sendjointdata[3] = -jointangledata[0];
        sendjointdata[4] = jointangledata[1];
        sendjointdata[5] = jointangledata[2];


        tpos = Quaternion.Inverse(targetBody.rotation) * (targetRL.position - targetBody.position) - tra_joints[6].localPosition;
        //tpos.x = -tpos.x;
        jointangledata = LegIK2(tpos);
        tra_joints[6].localRotation = Quaternion.Euler(new Vector3(0.0f, 0.0f, jointangledata[0] * 180.0f / Mathf.PI));
        tra_joints[7].localRotation = Quaternion.Euler(new Vector3(jointangledata[1] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        tra_joints[8].localRotation = Quaternion.Euler(new Vector3(jointangledata[2] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        sendjointdata[6] = jointangledata[0];
        sendjointdata[7] = jointangledata[1];
        sendjointdata[8] = jointangledata[2];


        tpos = Quaternion.Inverse(targetBody.rotation) * (targetRR.position - targetBody.position) - tra_joints[9].localPosition;
        tpos.x = -tpos.x;
        jointangledata = LegIK2(tpos); ;
        tra_joints[9].localRotation = Quaternion.Euler(new Vector3(0.0f, 0.0f, -jointangledata[0] * 180.0f / Mathf.PI));
        tra_joints[10].localRotation = Quaternion.Euler(new Vector3(jointangledata[1] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        tra_joints[11].localRotation = Quaternion.Euler(new Vector3(jointangledata[2] * 180.0f / Mathf.PI, 0.0f, 0.0f));
        sendjointdata[9] = -jointangledata[0];
        sendjointdata[10] = jointangledata[1];
        sendjointdata[11] = jointangledata[2];



        SetMsgVal(sendjointdata);
        Publish();


        /*
        for (int a=0; a<12; a++)
        {
            var foot1 = this.tra_joints[a].xDrive;
            foot1.target = (float)((sliders[a].value) * 180.0f / Mathf.PI);
            this.tra_joints[a].xDrive = foot1;
        }
        */




    }


    public float[] LegIK2(Vector3 fpos)
    {
        float[] res = new float[3];



        //res[0] = Mathf.Atan2((float)-fpos.x, (float)fpos.y);


        float qang1 = Vector3.SignedAngle(new Vector3(0.0f, -1.0f, 0.0f), new Vector3(fpos.x, fpos.y, 0.0f), new Vector3(0.0f, 0.0f, 1.0f));

        float Distance_L1toTP_xy = Vector3.Distance(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(fpos.x, fpos.y));
        float Distance_L1toTP_xy_y = Mathf.Sqrt(Mathf.Pow(Distance_L1toTP_xy, 2.0f) - Mathf.Pow(offset_x, 2.0f));

        float qang2 = Vector3.SignedAngle(new Vector3(0.0f, -1.0f, 0.0f), new Vector3(offset_x, -Distance_L1toTP_xy_y, 0.0f), new Vector3(0.0f, 0.0f, 1.0f));


        res[0] = (qang1- qang2) / 180.0f * Mathf.PI;




        Vector3 newfpos = Quaternion.AngleAxis(-((qang1 - qang2)), new Vector3(0.0f, 0.0f, 1.0f)) * fpos;
        //targetFR.transform.position = testObjFL1.transform.position + new Vector3(newfpos.x, newfpos.y, 0.0f);

        float Distance_L2toTP_yz = Vector3.Distance(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(newfpos.z, newfpos.y));

        res[2] = Mathf.Acos((Mathf.Pow(LegL2, 2.0f) + Mathf.Pow(LegL3, 2.0f) - Mathf.Pow(Distance_L2toTP_yz, 2.0f)) / (2.0f * LegL2 * LegL3)) - Mathf.PI;




        float newTptoP2toTPy = Mathf.Atan2(newfpos.z, newfpos.y);
        float cosP3toP2tonewTP = (Mathf.Pow(L2, 2.0f) + Mathf.Pow(Distance_L2toTP_yz, 2.0f) - Mathf.Pow(L3, 2.0f)) / (2.0f * L2 * Distance_L2toTP_yz);
        float P3toP2tonewTP = Mathf.Acos(cosP3toP2tonewTP);

        res[1] = newTptoP2toTPy - P3toP2tonewTP;

        //Angle of the second link relative to the tilted negative z axis
        float knee_angle = res[1] - (Mathf.PI - res[2]) + Mathf.PI*2;

        Debug.Log((knee_angle*180.0f/Mathf.PI).ToString());

        return res;
    }





    float[] leg_explicit_inverse_kinematics(float[] r_body_foot, int leg_index)
    {
        /*
        Find the joint angles corresponding to the given body-relative foot position for a given leg and configuration

        Parameters
        ----------
        r_body_foot:[type]
        [description]
        leg_index:[type]
        [description]

        Returns
        ------ -
        float[3] array
        Array of corresponding joint angles.
        */

        const float config_abduction_offset = 0.026f;
        float[] config_abduction_offsets = { -config_abduction_offset, config_abduction_offset, -config_abduction_offset, config_abduction_offset };
        const float config_l1 = 0.060f;
        const float config_l2 = 0.050f;


        float x, y, z;
        z = r_body_foot[0];
        y = r_body_foot[1];
        x = r_body_foot[2];

        //Distance from the leg origin to the foot, projected into the y-z plane
        float R_body_foot_yz = Mathf.Sqrt((Mathf.Pow(y, 2.0f) + Mathf.Pow(z, 2.0f))) ;

        //Distance from the leg's forward/back point of rotation to the foot
        float R_hip_foot_yz = Mathf.Sqrt((Mathf.Pow(R_body_foot_yz, 2.0f) - Mathf.Pow(config_abduction_offset, 2.0f)));

        //Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
        //For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
        float arccos_argument = config_abduction_offsets[leg_index] / R_body_foot_yz;
        arccos_argument = Mathf.Clamp(arccos_argument, -0.99f, 0.99f);
        float phi = Mathf.Acos(arccos_argument);

        //Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
        float hip_foot_angle = Mathf.Atan2(z, y);

        //Ab/adduction angle, relative to the positive y-axis
        float abduction_angle = phi + hip_foot_angle;

        //theta: Angle between the tilted negative z-axis and the hip-to-foot vector
        float theta = Mathf.Atan2(-x, R_hip_foot_yz);

        //Distance between the hip and foot
        float R_hip_foot = Mathf.Sqrt((Mathf.Pow(R_hip_foot_yz, 2.0f) + Mathf.Pow(x, 2.0f)));

        //Angle between the line going from hip to foot and the link L1
        arccos_argument = (Mathf.Pow(config_l1, 2.0f) + Mathf.Pow(R_hip_foot, 2.0f) - Mathf.Pow(config_l1, 2.0f)) / (2 * config_l1 * R_hip_foot);

        arccos_argument = Mathf.Clamp(arccos_argument, -0.99f, 0.99f);
        float trident = Mathf.Acos(arccos_argument);

        //Angle of the first link relative to the tilted negative z axis
        float hip_angle = theta + trident;

        //Angle between the leg links L1 and L2
        arccos_argument = (Mathf.Pow(config_l1, 2.0f) + Mathf.Pow(config_l2, 2.0f) - Mathf.Pow(R_hip_foot, 2.0f)) / (2 * config_l1 * config_l2);

        arccos_argument = Mathf.Clamp(arccos_argument, -0.99f, 0.99f);
        float beta = Mathf.Acos(arccos_argument);

        //Angle of the second link relative to the tilted negative z axis
        float knee_angle = hip_angle - (Mathf.PI - beta);

        

        float[] ret = {abduction_angle, hip_angle, knee_angle };
        //float[] ret = { abduction_angle, 0.0f, 0.0f };

        return ret;

    }


    public float[] LegIK(Vector3 fpos)
    {
        //https://gitlab.com/custom_robots/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics
        //https://mathtrain.jp/kangenkoushiki
        //https://study-line.com/sanhi-90/
        //http://w3e.kanazawa-it.ac.jp/math/category/sankakukansuu/henkan-tex.cgi?target=/math/category/sankakukansuu/yogenteiri.html
        //https://twitter.com/devemin/status/1292105975457931265


        //ノート参照

        //float L1;
        //float L2;

        //L1 = tra_joint_FL1.position.x - tra_joint_FL2.position.x;
        //L2 = tra_joint_FL2.position.y - tra_joint_FL3.position.y;

        //L1 = 0.05f; //tra_joints[1].transform.position.y - tra_joints[0].transform.position.y;
        //L2 = 0.06f; //tra_joints[2].transform.position.y - tra_joints[1].transform.position.y;
        Debug.Log(L1.ToString() + ", " + L2.ToString());


        float[] res = new float[3];



        //res[0] = Mathf.Atan2((float)-fpos.x, (float)fpos.y);


        float P1toTP = Mathf.Sqrt(Mathf.Pow((float)fpos.x, 2.0f) + Mathf.Pow((float)fpos.y, 2.0f));
        float p2toTpxLen = Mathf.Sqrt(Mathf.Pow(P1toTP, 2.0f) - Mathf.Pow(L1, 2.0f));
        float cosp1tri = (Mathf.Pow(L1, 2.0f) + Mathf.Pow(P1toTP, 2.0f) - Mathf.Pow(p2toTpxLen, 2.0f)) / (2.0f * L1 * P1toTP);
        float p1triangle = Mathf.Acos(cosp1tri);


        Quaternion rotTP1 = new Quaternion();
        rotTP1 = Quaternion.Euler(0, 0, -p1triangle * 180.0f / Mathf.PI);
        Vector3 newTP1 = rotTP1 * fpos;
        float p1angle = Mathf.Atan2(newTP1.y, newTP1.x);
        res[0] = p1angle;

        return res;
    }



    public float[] testMyIK(Vector3 targetpos)
    {
        float[] res = new float[3];

        return res;
    }

    public float[] LegOldIK(Vector3 fpos)
    {
        //https://gitlab.com/custom_robots/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics
        //https://mathtrain.jp/kangenkoushiki
        //https://study-line.com/sanhi-90/
        //http://w3e.kanazawa-it.ac.jp/math/category/sankakukansuu/henkan-tex.cgi?target=/math/category/sankakukansuu/yogenteiri.html
        //https://twitter.com/devemin/status/1292105975457931265


        //ノート参照


        float[] res = new float[3];


        //res[0] = Mathf.Atan2((float)-fpos.x, (float)fpos.y);


        float P1toTP = Mathf.Sqrt(Mathf.Pow((float)fpos.x, 2.0f) + Mathf.Pow((float)fpos.y, 2.0f));
        float p2toTpxLen = Mathf.Sqrt(Mathf.Pow(P1toTP, 2.0f) - Mathf.Pow(L1, 2.0f));
        float cosp1tri = (Mathf.Pow(L1, 2.0f) + Mathf.Pow(P1toTP, 2.0f) - Mathf.Pow(p2toTpxLen, 2.0f)) / (2.0f * L1 * P1toTP);
        float p1triangle = Mathf.Acos(cosp1tri);


        Quaternion rotTP1 = new Quaternion();
        rotTP1 = Quaternion.Euler(0, 0, -p1triangle * 180.0f / Mathf.PI);
        Vector3 newTP1 = rotTP1 * fpos;
        float p1angle = Mathf.Atan2(newTP1.y, newTP1.x);
        res[0] = p1angle;
        //Debug.Log("newTP: " + newTP.ToString());
        //Debug.Log("p1triangle: " + (p1triangle * 180.0f / Mathf.PI).ToString() + ", p1angle: " + (p1angle * 180.0f / Mathf.PI).ToString() );


        Quaternion rotTP2 = new Quaternion();
        rotTP2 = Quaternion.Euler(0, 0, -res[0] * 180.0f / Mathf.PI);
        Vector3 newTP2 = rotTP2 * fpos;

        //Debug.Log("x: " + newTP2.x.ToString() + ", y: " + newTP2.y.ToString() + ", z: " + newTP2.z.ToString());


        //float P2tonewTP = Mathf.Sqrt(Mathf.Pow((float)(newTP2.z), 2.0f) + Mathf.Pow((float)(newTP2.y), 2.0f));
        float P2tonewTP = Vector2.Distance(new Vector2(0, 0), new Vector2(newTP2.z, newTP2.y));


        //Debug.Log(P2tonewTP.ToString());

        float cosP3 = (Mathf.Pow(L2, 2.0f) + Mathf.Pow(L3, 2.0f) - Mathf.Pow(P2tonewTP, 2.0f)) / (2.0f * L2 * L3);
        res[2] = Mathf.Acos(cosP3) - Mathf.PI; //(マイナス180度することで、常にヒザを後ろ曲げ)


        float newTptoP2toTPy = Mathf.Atan2(newTP2.z, newTP2.y);
        float cosP3toP2tonewTP = (Mathf.Pow(L2, 2.0f) + Mathf.Pow(P2tonewTP, 2.0f) - Mathf.Pow(L3, 2.0f)) / (2.0f * L2 * P2tonewTP);
        float P3toP2tonewTP = Mathf.Acos(cosP3toP2tonewTP);

        res[1] = newTptoP2toTPy + P3toP2tonewTP;

        res[0] = res[0] + Mathf.PI;
        res[1] = -res[1];
        res[2] = -res[2];


        return res;
    }



    public float[] MiniPupper_LegIK(Vector3 targetpos)
    {
        float[] res = new float[3];
        /*
        Vector3 def_legpos1 = new Vector3(   -0.0235f,          0.0171f,  0.06014f);
        Vector3 def_legpos2 = new Vector3(   -0.0197f,          0.0f,    -0.00003999844f);
        Vector3 def_legpos3 = new Vector3(   -0.004749998f,    -0.05f,    0.0f);
        Vector3 def_legposFoot = new Vector3( 0.00005000085f,  -0.056f,   0.0f);


        float L1 = tra_joints[1].transform.localPosition.y + tra_joints[2].transform.localPosition.y;
        float L2 = JointPosFLFoot.transform.localPosition.y;


        float x = targetpos[0];
        float y = targetpos[1];
        float z = targetpos[2];

        float tpos_offset_x = def_legpos2.x + def_legpos3.x + def_legposFoot.x;
        float tpos_offset_y = def_legpos2.y + def_legpos3.y + def_legposFoot.y;
        float Distance_L1toOffset_xyplane = Mathf.Sqrt(Mathf.Pow(tpos_offset_x, 2.0f) + Mathf.Pow(y, 2.0f));
        float Distance_L1toTpos_xyplane = Mathf.Sqrt(Mathf.Pow(x, 2.0f) + Mathf.Pow(y, 2.0f));



        //float cos_offsetangle_L1 = (Mathf.Pow(Distance_L1toOffset_xyplane, 2.0f) + Mathf.Pow(y, 2.0f) - Mathf.Pow(tpos_offset_x, 2.0f)) / (2.0f * Distance_L1toOffset_xyplane * y);
        float Distance_L1toConvert_y = Mathf.Sqrt(Mathf.Pow(Distance_L1toTpos_xyplane, 2.0f) - Mathf.Pow(tpos_offset_x, 2.0f));
        float cos_offsetangle_L1 = Distance_L1toConvert_y / Distance_L1toTpos_xyplane;
        float offsetangle_L1 = Mathf.Acos(cos_offsetangle_L1);


        //float cos_angle_L1 = (Mathf.Pow(Distance_L1toTpos_xyplane, 2.0f) + Mathf.Pow(y, 2.0f) - Mathf.Pow(x, 2.0f)) / (2.0f * Distance_L1toTpos_xyplane * y);
        float cos_angle_L1 = y / -Distance_L1toTpos_xyplane;
        float angle_L1 = Mathf.Acos(cos_angle_L1);


        Debug.Log((angle_L1*180.0f/Mathf.PI).ToString() + ", " + (offsetangle_L1 * 180.0f / Mathf.PI).ToString());
        res[0] = angle_L1 ;
        if (x>0.0f)
        {
            res[0] = -res[0];
        }
        res[0] = res[0] - offsetangle_L1;



        Quaternion rotTP = new Quaternion();
        rotTP = Quaternion.Euler(0, 0, (res[0]+ offsetangle_L1) * 180.0f / Mathf.PI);
        Vector3 newTP = rotTP * targetpos;

        targetFR.transform.position = newTP + tra_joints[0].transform.position +new Vector3(tpos_offset_x, 0.0f, 0.0f);

        float P2tonewTP = Vector2.Distance(new Vector2(0, 0), new Vector2(newTP.y, newTP.z));

        float cosP3 = (Mathf.Pow(L1, 2.0f) + Mathf.Pow(L2, 2.0f) - Mathf.Pow(P2tonewTP, 2.0f)) / (2.0f * L1 * L2);
        res[2] = Mathf.Acos(cosP3) - Mathf.PI; //(マイナス180度することで、常にヒザを後ろ曲げ)



        float newTptoP2toTPy = Mathf.Atan2(newTP.y, -newTP.z);
        float cosP3toP2tonewTP = (Mathf.Pow(L1, 2.0f) + Mathf.Pow(P2tonewTP, 2.0f) - Mathf.Pow(L2, 2.0f)) / (2.0f * L1 * P2tonewTP);
        float P3toP2tonewTP = Mathf.Acos(cosP3toP2tonewTP);

        res[1] = newTptoP2toTPy + P3toP2tonewTP;



        inputs[0].text = "targetpos: " + targetpos.ToString("f3");
        inputs[1].text = "tpos_offset_x: " + tpos_offset_x.ToString("f3");
        inputs[2].text = "tpos_offset_y: " + tpos_offset_y.ToString("f3");
        inputs[3].text = "Distance_L1toOffset_xyplane: " + Distance_L1toOffset_xyplane.ToString("f3");
        inputs[4].text = "Distance_L1toTpos_xyplane: " + Distance_L1toTpos_xyplane.ToString("f3");
        inputs[5].text = "cos_offsetangle_L1: " + cos_offsetangle_L1.ToString("f3");
        inputs[6].text = "offsetangle_L1(deg): " + (offsetangle_L1 * 180.0f / Mathf.PI).ToString("f3");
        inputs[7].text = "cos_angle_L1: " + cos_angle_L1.ToString("f13");
        inputs[8].text = "angle_L1(deg): " + (angle_L1*180.0f/Mathf.PI).ToString("f3");
        inputs[9].text = "x: " + x.ToString("f3");
        inputs[10].text = "y: " + y.ToString("f3");
        inputs[11].text = "z: " + z.ToString("f3");

        */
        return res;
    }


    float[,] four_legs_inverse_kinematics(float[,] r_body_foot)
    {
        /*
        Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.
    
        Parameters
        ----------
        r_body_foot : float[3,4] array
        Matrix of the body-frame foot positions.Each column corresponds to a separate foot.

        Returns
        -------
        float[3,4] array
        Matrix of corresponding joint angles.
        */
        
        const float config_fb = 0.059f;
        const float config_lr = 0.0235f;
        const float config_ud = 0.0171f;
        float[,] config_leg_origins = { { config_lr, -config_lr, config_lr, -config_lr }, { -config_ud , -config_ud , -config_ud , -config_ud }, { config_fb, config_fb, -config_fb, -config_fb } };


        float[,] alpha = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };


        float[] local_r_body_foot = { 0.0f, 0.0f, 0.0f};

        for (int a=0; a<4; a++)
        {
            for (int b = 0; b < 3; b++)
            {
                local_r_body_foot[b] = r_body_foot[b,a] - config_leg_origins[b,a];
            }

            float[] getdata = leg_explicit_inverse_kinematics(local_r_body_foot, a);
            for (int b = 0; b < 3; b++)
            {
                alpha[b,a] = getdata[b];
            }
        }


        return alpha;
    }



}
