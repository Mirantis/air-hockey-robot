using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using X360Gamepad;
using System.Diagnostics;

namespace MotorControlArduino
{
    public partial class Form1 : Form
    {
        private GamepadState gpState;
        private SerialPort serial;

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            gpState = new GamepadState(0);
            serial = new SerialPort("COM41", 19200);
        }

        private void updateSpeedButton_Click(object sender, EventArgs e)
        {
            float leftSpd = float.Parse(leftSpeedBox.Text);
            float rightSpd = float.Parse(rightSpeedBox.Text);
            char cFlag = (char)1;
            UpdateSpeed(leftSpd,rightSpd,cFlag);
            serial.Close();
        }

        private void travelDistanceButton_Click(object sender, EventArgs e)
        {
            float xTarget = float.Parse(xTargetBox.Text);
            float yTarget = float.Parse(yTargetBox.Text);
            float tTarget = float.Parse(tTargetBox.Text);
            SendDefenseCommand(xTarget, yTarget, tTarget);
            serial.Close();
        }

        private void attackButton_Click(object sender, EventArgs e)
        {
            float xTarget = float.Parse(xAttackBox.Text);
            float yTarget = float.Parse(yAttackBox.Text);
            float tTarget = float.Parse(tAttackBox.Text);
            float aTarget = float.Parse(aAttackBox.Text) * (float)Math.PI/180;

            SendAttackCommand(xTarget, yTarget, tTarget, aTarget);
            serial.Close();
        }

        //defend+attack button
        private void button1_Click(object sender, EventArgs e)
        {
            float xD = float.Parse(xTargetBox.Text);
            float yD = float.Parse(yTargetBox.Text);
            float tD = float.Parse(tTargetBox.Text);
            float xA = float.Parse(xAttackBox.Text);
            float yA = float.Parse(yAttackBox.Text);
            float tA = float.Parse(tAttackBox.Text);
            float aA = float.Parse(aAttackBox.Text) * (float)Math.PI / 180;
            SendDefenseAttackCommand(xD, yD, tD, xA, yA, tA, aA);
            serial.Close();
        }

        private void X360PollTimer_Tick(object sender, EventArgs e)
        {
            gpState.Update();
            float xL = gpState.LeftStick.Position.X;
            float yL = gpState.LeftStick.Position.Y;
            float xR = gpState.RightStick.Position.X;
            float yR = gpState.RightStick.Position.Y;

            if (Math.Abs(xL) < 0.01) { xL = 0; }
            if (Math.Abs(yL) < 0.01) { yL = 0; }
            if (Math.Abs(xR) < 0.01) { xR = 0; }
            if (Math.Abs(yR) < 0.01) { yR = 0; }

            char cFlag = (char)1;
            float xSpdMax = 800, ySpdMax = 900;
            leftSpeedBox.Text = -1 * xL * xSpdMax + ""; rightSpeedBox.Text = yR * ySpdMax + "";
            //System.Diagnostics.Debug.Write(xL * 300 + "," + yL * 300 + "\n");
            UpdateSpeed(-1 * xL * xSpdMax, yR * ySpdMax, cFlag);
            //short state = GetState();
            //float speedCMD = GetSpeedCMD();
            //System.Diagnostics.Debug.Write(speedCMD + "\n");

        }

        private void SendDefenseCommand(float xT, float yT, float tT)
        {
            if (!serial.IsOpen) { OpenPort(); }

            byte[] sdCmd = new byte[18];
            sdCmd[0] = Convert.ToByte('s'); sdCmd[1] = Convert.ToByte('d');
            byte[] xTarray = BitConverter.GetBytes(xT);
            byte[] yTarray = BitConverter.GetBytes(yT);
            byte[] tTarray = BitConverter.GetBytes(tT);

            Array.Copy(xTarray, 0, sdCmd, 2, 4); Array.Copy(yTarray, 0, sdCmd, 6, 4);
            Array.Copy(tTarray, 0, sdCmd, 10, 4);

            //compute checksum, add to array
            UInt16 checksum = 0;
            for (int i = 2; i < sdCmd.Length - 2; i++) {
                checksum += sdCmd[i];
            }
            Array.Copy(BitConverter.GetBytes(checksum), 0, sdCmd, 14, 2);

            //add suffix bytes
            sdCmd[16] = 0x08; sdCmd[17] = 0x09;

            serial.Write(sdCmd, 0, sdCmd.Length);
        }

        private void SendAttackCommand(float xT, float yT, float tT, float aT)
        {
            if (!serial.IsOpen) { OpenPort(); }

            byte[] saCmd = new byte[18];
            saCmd[0] = Convert.ToByte('s'); saCmd[1] = Convert.ToByte('a');
            byte[] xTarray = BitConverter.GetBytes(xT);
            byte[] yTarray = BitConverter.GetBytes(yT);
            byte[] tTarray = BitConverter.GetBytes(tT);

            Array.Copy(xTarray, 0, saCmd, 2, 4); Array.Copy(yTarray, 0, saCmd, 6, 4);
            Array.Copy(tTarray, 0, saCmd, 10, 4);

            //compute checksum, add to array
            UInt16 checksum = 0;
            for (int i = 2; i < saCmd.Length - 2; i++) {
                checksum += saCmd[i];
            }
            Array.Copy(BitConverter.GetBytes(checksum), 0, saCmd, saCmd.Length - 4, 2);

            //add suffix bytes
            saCmd[saCmd.Length - 2] = 0x08; saCmd[saCmd.Length - 1] = 0x09;

            serial.Write(saCmd, 0, saCmd.Length);
        }

        private void SendDefenseAttackCommand(float xD, float yD, float tD, float xA,
            float yA, float tA, float aA)
        {
            if (!serial.IsOpen) { OpenPort(); }

            byte[] daCmd = new byte[34];
            daCmd[0] = Convert.ToByte('d'); daCmd[1] = Convert.ToByte('a');
            byte[] xDarray = BitConverter.GetBytes(xD);
            byte[] yDarray = BitConverter.GetBytes(yD);
            byte[] tDarray = BitConverter.GetBytes(tD);
            byte[] xAarray = BitConverter.GetBytes(xA);
            byte[] yAarray = BitConverter.GetBytes(yA);
            byte[] tAarray = BitConverter.GetBytes(tA);
            byte[] aAarray = BitConverter.GetBytes(aA);

            Array.Copy(xDarray, 0, daCmd, 2, 4); Array.Copy(yDarray, 0, daCmd, 6, 4);
            Array.Copy(tDarray, 0, daCmd, 10, 4);
            Array.Copy(xAarray, 0, daCmd, 14, 4); Array.Copy(yAarray, 0, daCmd, 18, 4);
            Array.Copy(tAarray, 0, daCmd, 22, 4); Array.Copy(aAarray, 0, daCmd, 26, 4);

            //compute checksum, add to array
            UInt16 checksum = 0;
            for (int i = 2; i < daCmd.Length - 2; i++) {
                checksum += daCmd[i];
            }
            Array.Copy(BitConverter.GetBytes(checksum), 0, daCmd, daCmd.Length - 4, 2);

            //add suffix bytes
            daCmd[daCmd.Length - 2] = 0x08; daCmd[daCmd.Length - 1] = 0x09;

            serial.Write(daCmd, 0, daCmd.Length);
        }

        private void OpenPort()
        {
            if (!serial.IsOpen)
            {
                try {
                    serial.Open();
                }
                catch {
                    MessageBox.Show("Error opening serial port!");
                }
            }
        }

        private void UpdateSpeed(float leftSpeedRPM, float rightSpeedRPM, char cFlag)
        {
            if (!serial.IsOpen) { OpenPort(); }

            byte[] ssCmd = new byte[13];
            ssCmd[0] = Convert.ToByte('s'); ssCmd[1] = Convert.ToByte('s');
            byte[] leftSpdArray = BitConverter.GetBytes(leftSpeedRPM);
            byte[] rightSpdArray = BitConverter.GetBytes(rightSpeedRPM);
            byte[] cfArray = BitConverter.GetBytes(cFlag);

            Array.Copy(leftSpdArray, 0, ssCmd, 2, 4); Array.Copy(rightSpdArray, 0, ssCmd, 6, 4);
            Array.Copy(cfArray, 0, ssCmd, 10, 1);

            //compute checksum, add to array
            UInt16 checksum = 0;
            for (int i = 2; i < ssCmd.Length - 2; i++) {
                checksum += ssCmd[i];
            }
            Array.Copy(BitConverter.GetBytes(checksum), 0, ssCmd, 11, 2);

            //Debug.Write(checksum + "\n");

            //send target SS speed cmd
            /*
            serial.Write("s");
            serial.Write("s");
            serial.Write(leftSpdArray, 0, 4);
            serial.Write(rightSpdArray, 0, 4);

            //send continuous rotation flag (1 byte)
            byte[] cfArray = BitConverter.GetBytes(cFlag);
            serial.Write(cfArray,0,1);
            */

            serial.Write(ssCmd, 0, ssCmd.Length);
            //Debug.Write(leftSpeedRPM + "," + rightSpeedRPM + "\n");

            //read response in chars
            /*
            byte[] response = new byte[8];
            for (int i = 0; i < 8; i++)
                response[i] = (byte)serial.ReadByte();

            float leftEchoSpeed = BitConverter.ToSingle(response, 0);
            float rightEchoSpeed = BitConverter.ToSingle(response, 4);
            //short state = GetState();
            //serial.Close();
            textBox1.Text = leftEchoSpeed + "," + rightEchoSpeed;
            */
        }

        private short GetState()
        {
            if (!serial.IsOpen) { OpenPort(); }
            //send get state command
            //serial.Write("f");
            //serial.Write("s");
            byte[] fsCmd = { Convert.ToByte('f'), Convert.ToByte('s') };
            serial.Write(fsCmd, 0, 2);

            //read response in chars
            byte[] response = new byte[2];
            for (int i = 0; i < 2; i++)
                response[i] = (byte)serial.ReadByte();

            return BitConverter.ToInt16(response, 0);
        }

        private float GetSpeedCMD()
        {
            if (!serial.IsOpen) { OpenPort(); }
            //send get speedCMD command
            serial.Write("g");
            serial.Write("s");

            //read response in chars
            byte[] response = new byte[4];
            for (int i = 0; i < 4; i++)
                response[i] = (byte)serial.ReadByte();

            return BitConverter.ToSingle(response, 0);
        }

        private void getTimerConfig()
        {
            if (!serial.IsOpen) { OpenPort(); }
            //send get current speed command
            serial.Write("t");
            serial.Write("c");

            //read response in chars
            byte[] response = new byte[10];
            for (int i = 0; i < 10; i++)
                response[i] = (byte)serial.ReadByte();

            float fpulse = BitConverter.ToSingle(response, 0);
            float prescaler = BitConverter.ToSingle(response, 4);
            short ocraVal = BitConverter.ToInt16(response, 8);
            serial.Close();

            xTargetBox.Text = fpulse + "," + prescaler + "," + ocraVal;
        }

        private void getPosition()
        {
            if (!serial.IsOpen) { OpenPort(); }
            //send get position command
            serial.Write("g");
            serial.Write("p");

            //read response in chars
            byte[] response = new byte[10];
            for (int i = 0; i < 10; i++)
                response[i] = (byte)serial.ReadByte();

            float leftPos = BitConverter.ToSingle(response, 0);
            float rightPos = BitConverter.ToSingle(response, 4);
            int ocraVal = BitConverter.ToInt16(response, 8);
            //serial.Close();

            System.Diagnostics.Debug.Write(leftPos+"\t"+rightPos+"\t"+ocraVal+"\n");
        }

        private void enable360Button_CheckedChanged(object sender, EventArgs e)
        {
            DistancePollTimer.Stop();
            X360PollTimer.Start();
        }

        private void disable360Button_CheckedChanged(object sender, EventArgs e)
        {
            UpdateSpeed(0F, 0F, (char)0);
            X360PollTimer.Stop();
            serial.Close();
        }

        private void DistancePollTimer_Tick(object sender, EventArgs e)
        {
            getPosition();
        }

        private void setGainsButton_Click(object sender, EventArgs e)
        {
            //float kp = float.Parse(textBox2.Text);
            //float ki = float.Parse(textBox3.Text);
            //float kd = float.Parse(textBox4.Text);
            //SetPIDGains(kp,ki,kd);
            getTimerConfig();
        }

        private void SetPIDGains(float kp,float ki,float kd)
        {
            if (!serial.IsOpen) { OpenPort(); }

            byte[] kpArray = BitConverter.GetBytes(kp);
            byte[] kiArray = BitConverter.GetBytes(ki);
            byte[] kdArray = BitConverter.GetBytes(kd);

            //send target SS speed cmd
            serial.Write("p");
            serial.Write("i");
            serial.Write(kpArray, 0, 4);
            serial.Write(kiArray, 0, 4);
            serial.Write(kdArray, 0, 4);

            serial.Close();
        }

        private void getStateButton_Click(object sender, EventArgs e)
        {
            short state = GetState();
            float speedCMD = GetSpeedCMD();
            textBox1.Text = state + "," + speedCMD;
        }

        // stop
        private void stopButton_Click(object sender, EventArgs e)
        {
            UpdateSpeed(0, 0, (char)0);
            textBox1.Text = 0+""; textBox2.Text = 0+"";
        }
    }
}
 