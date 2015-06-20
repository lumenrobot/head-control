using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Newtonsoft.Json;
using RabbitMQ.Client;
using RabbitMQ.Client.Events;
using RabbitMQ.Client.MessagePatterns;
using RabbitMQ.Util;
using Aldebaran.Proxies;
using System.Numerics;
using System.Collections;
namespace LumenMotion
{
    class HeadControl
    {
        mf locXNeg, locXZer, locXPos, locXSNeg, locXSPos;//
        mf locYNeg, locYZer, locYPos;
        mf errXNeg, errXZer, errXPos;
        mf errYNeg, errYZer, errYPos;
        mf angXNeg, angXZer, angXPos, angXSNeg, angXSPos;
        mf angYNeg, angYZer, angYPos;
        double[] xAng,xErr, yAng,yErr, xloc, yloc;
        double X, Y, prevX, prevY;
        MotionProxy motion;
        RobotPostureProxy posture;
        public HeadControl()
        {
            Console.WriteLine("creating membership function...");
            //create universe
            xloc = new double[321];
            for (int i = 0; i < xloc.Length; i++)
            {
                xloc[i] =  i;
            }
            yloc = new double[241];
            for (int i = 0; i < yloc.Length; i++)
            {
                yloc[i] = i;
            }

            xErr = new double[321];
            for (int i = 0; i < xErr.Length; i++)
            {
                xErr[i] = (double)-160 + i;
            }
            yErr = new double[241];
            for (int i = 0; i < yErr.Length; i++)
            {
                yErr[i] = (double)-120 + i;
            }

            xAng = new double[31];
            for (int i = 0; i < xAng.Length; i++)
            {
                xAng[i] = (double)-15 + i;
            }
            yAng = new double[15];
            for (int i = 0; i < yAng.Length; i++)
            {
                yAng[i] = (double)-7 + i; ;
            }

            //make membership function 
            locXNeg = new mf("locXNeg", xloc, 0, 80);
            //locXSNeg = new mf("locXSNeg", xloc, 64, 40);
            locXZer = new mf("locXZer", xloc, 160, 50);
            //locXSPos = new mf("locXSPos", xloc, 254, 40);
            locXPos = new mf("locXPos", xloc, 320, 80);

            locYNeg = new mf("locYNeg", yloc, 0, 70);
            locYZer = new mf("locYZer", yloc, 120, 40);
            locYPos = new mf("locYPos", yloc, 240, 70);

            errXNeg = new mf("", xErr, -160, 50);
            errXZer = new mf("", xErr, 0, 50);
            errXPos = new mf("", xErr, 160, 50);

            errYNeg = new mf("", yErr, -160, 50);
            errYZer = new mf("", yErr, 0, 50);
            errYPos = new mf("", yErr, 160, 50);

            angXNeg = new mf("angXNeg", xAng, -15,10);
            //angXSNeg = new mf("angXSNeg", xAng, -7, 6);
            angXZer = new mf("angXZer", xAng, 0, 10);
            //angXSPos = new mf("angXSPos", xAng, 7, 6);
            angXPos = new mf("angXPos", xAng, 15, 10);
            
            angYNeg = new mf("angYNeg", yAng, -7, 6);
            angYZer = new mf("angYZer", yAng, 0, 6);
            angYPos = new mf("angYPos", yAng, 7, 6);

            Console.WriteLine("membership function created");
        }
        public double[] getAngle(double x,double y,double dx,double dy)
        {
            FuzzyOperation op = new FuzzyOperation();
            //dapatkan firing strenght x dari tiap mf di universe X
            double[] xFire = new double[3];
            xFire[0] = locXNeg.getValue(x);//Console.WriteLine(xFire[0]);
            //xFire[1] = locXSNeg.getValue(x);
            xFire[1] = locXZer.getValue(x);//Console.WriteLine(xFire[1]);
            //xFire[3] = locXSPos.getValue(x);
            xFire[2] = locXPos.getValue(x);//Console.WriteLine(xFire[2]);

            double[] eXFire = new double[3];
            eXFire[0] = errXNeg.getValue(dx);
            eXFire[1] = errXZer.getValue(dx);
            eXFire[2] = errXPos.getValue(dx);

            double[] yFire = new double[3];
            yFire[0] = locYNeg.getValue(y);//Console.WriteLine(xFire[0]);
            yFire[1] = locYZer.getValue(y);//Console.WriteLine(xFire[1]);
            yFire[2] = locYPos.getValue(y);//Console.WriteLine(xFire[2]);

            double[] eYFire = new double[3];
            eYFire[0] = errYNeg.getValue(dy);
            eYFire[1] = errYZer.getValue(dy);
            eYFire[2] = errYPos.getValue(dy);

            //dapatkan perpotongan antara tiap firing strenght dan output untuk tiap rule pada yaw
            mf ruleA = op.min(angXPos, xFire[0],eXFire[0]);//N&N-P
            mf ruleB = op.min(angXPos, xFire[0],eXFire[1]); //N&Z-P
            mf ruleC = op.min(angXPos, xFire[0],eXFire[2]);//N&P-P
            mf ruleD = op.min(angXPos, xFire[1],eXFire[0]);//Z&N-P
            mf ruleE = op.min(angXZer, xFire[1],eXFire[1]);//Z&Z-Z
            mf ruleF = op.min(angXNeg, xFire[1], eXFire[2]);//Z&P-N
            mf ruleG = op.min(angXNeg, xFire[2], eXFire[0]);//P&N-N
            mf ruleH = op.min(angXNeg, xFire[2], eXFire[1]);//P&Z-N
            mf ruleI = op.min(angXNeg, xFire[2], eXFire[2]);//P&P-N
            List<mf> firingX = new List<mf> { ruleA,ruleB,ruleC,ruleD,ruleE,ruleF,ruleG,ruleH,ruleI };

            mf ruleJ = op.min(angYNeg, yFire[0], eYFire[0]);//N&N-N
            mf ruleK = op.min(angYNeg, yFire[0], eYFire[1]); //N&Z-N
            mf ruleL = op.min(angYNeg, yFire[0], eYFire[2]);//N&P-N
            mf ruleM = op.min(angYNeg, yFire[1], eYFire[0]);//Z&N-N
            mf ruleN = op.min(angYZer, yFire[1], eYFire[1]);//Z&Z-Z
            mf ruleO = op.min(angYPos, yFire[1], eYFire[2]);//Z&P-P
            mf ruleP = op.min(angYPos, yFire[2], eYFire[0]);//P&N-P
            mf ruleQ = op.min(angYPos, yFire[2], eYFire[1]);//P&Z-P
            mf ruleR = op.min(angYPos, yFire[2], eYFire[2]);//P&P-P
            List<mf> firingY = new List<mf> { ruleJ, ruleK, ruleL, ruleM, ruleN, ruleO, ruleP, ruleQ, ruleR };

            //mf firePY = op.min(angYPos, yFire[0]);//N->P;
            //mf fireZY = op.min(angYZer, yFire[1]);//Z->Z
            //mf fireNY = op.min(angYNeg, yFire[2]);//P->N
            //List<mf> firingY = new List<mf> { firePY, fireZY, fireNY };

            //dapatkan hasil maximimasi dari semua firing 
            mf concX = op.agregator(firingX);
            concX.x = this.xAng;
            mf concY = op.agregator(firingY);
            concY.x = this.yAng;

            //dapatkan centroid dari aggregator
            double angleX = op.centroid(concX);
            double angleY = op.centroid(concY);
            return new double[2] { angleX, angleY };
        }
        public void start()
        {
            Console.WriteLine("Starting connection...");
            motion = new MotionProxy("169.254.89.225", 9559);
            posture = new RobotPostureProxy("169.254.89.225", 9559);
            motion.setStiffnesses(new ArrayList() { "HeadYaw", "HeadPitch" }, new ArrayList() { 0.5f, 0.5f });
            ConnectionFactory factory = new ConnectionFactory();
            factory.Uri = "amqp://guest:guest@localhost/%2F";
            IConnection connection = factory.CreateConnection();
            IModel channelGet = connection.CreateModel();
            QueueDeclareOk queue = channelGet.QueueDeclare("", true, false, true, null);
            channelGet.QueueBind(queue.QueueName, "amq.topic", "lumen.visual.face.detection");
            EventingBasicConsumer consumer = new EventingBasicConsumer(channelGet);
            channelGet.BasicConsume(queue.QueueName, true, consumer);
            Console.WriteLine("connection ready");
            consumer.Received +=new BasicDeliverEventHandler(consumer_Received);
            X = 0; Y = 0; prevX = 0; prevY = 0;
        }
        public void consumer_Received(object sender, BasicDeliverEventArgs ev)
        {
            Console.WriteLine("incoming...");
            string body = Encoding.UTF8.GetString(ev.Body);
            FaceLocation loc;
            JsonSerializerSettings setting = new JsonSerializerSettings { TypeNameHandling = TypeNameHandling.Objects };
            loc = JsonConvert.DeserializeObject<FaceLocation>(body, setting);
            X = loc.x;
            double dx = X- prevX;
            prevX = X;
            Y = loc.y;
            double dy = Y - prevY;
            prevY = Y;
            double[] angle = getAngle(X, Y, dx, dy);
            //Console.WriteLine("face location : {0},{1},{2},{3}",X, Y,dx,dy);
            //Console.WriteLine("angle : {0:F2},{1:F2}", angle[0], angle[1]);
            motion.changeAngles(new ArrayList(){"HeadYaw","HeadPitch"} ,new ArrayList(){toRad(angle[0]), toRad(angle[1])} , 0.1f);
            float currentAngle=motion.getAngles("HeadYaw", false)[0];
            if (Math.Abs(currentAngle)>0.8f)
            {
                Console.WriteLine(currentAngle);
                motion.setAngles(new ArrayList() { "HeadYaw" }, new ArrayList() { 0.0f }, 0.4f);
                motion.moveTo(0.0f,0.0f,currentAngle);
                posture.goToPosture("Stand",0.7f);
            }
        }
        public float toRad(double input)
        {
            return (float)input * 2.0f * (float)Math.PI / 360.0f;
        }
        
        
    }
}
