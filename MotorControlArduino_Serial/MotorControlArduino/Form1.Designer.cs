namespace MotorControlArduino
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.leftSpeedBox = new System.Windows.Forms.TextBox();
            this.updateSpeedButton = new System.Windows.Forms.Button();
            this.X360PollTimer = new System.Windows.Forms.Timer(this.components);
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.enable360Button = new System.Windows.Forms.RadioButton();
            this.disable360Button = new System.Windows.Forms.RadioButton();
            this.label1 = new System.Windows.Forms.Label();
            this.travelDistanceButton = new System.Windows.Forms.Button();
            this.xTargetBox = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.DistancePollTimer = new System.Windows.Forms.Timer(this.components);
            this.rightSpeedBox = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.yTargetBox = new System.Windows.Forms.TextBox();
            this.getStateButton = new System.Windows.Forms.Button();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.textBox4 = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.setGainsButton = new System.Windows.Forms.Button();
            this.stopButton = new System.Windows.Forms.Button();
            this.tTargetBox = new System.Windows.Forms.TextBox();
            this.tAttackBox = new System.Windows.Forms.TextBox();
            this.yAttackBox = new System.Windows.Forms.TextBox();
            this.attackButton = new System.Windows.Forms.Button();
            this.xAttackBox = new System.Windows.Forms.TextBox();
            this.aAttackBox = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // leftSpeedBox
            // 
            this.leftSpeedBox.Location = new System.Drawing.Point(44, 41);
            this.leftSpeedBox.Margin = new System.Windows.Forms.Padding(2);
            this.leftSpeedBox.Name = "leftSpeedBox";
            this.leftSpeedBox.Size = new System.Drawing.Size(35, 20);
            this.leftSpeedBox.TabIndex = 0;
            this.leftSpeedBox.Text = "0";
            // 
            // updateSpeedButton
            // 
            this.updateSpeedButton.Location = new System.Drawing.Point(44, 63);
            this.updateSpeedButton.Margin = new System.Windows.Forms.Padding(2);
            this.updateSpeedButton.Name = "updateSpeedButton";
            this.updateSpeedButton.Size = new System.Drawing.Size(88, 19);
            this.updateSpeedButton.TabIndex = 1;
            this.updateSpeedButton.Text = "Update Speed";
            this.updateSpeedButton.UseVisualStyleBackColor = true;
            this.updateSpeedButton.Click += new System.EventHandler(this.updateSpeedButton_Click);
            // 
            // X360PollTimer
            // 
            this.X360PollTimer.Interval = 25;
            this.X360PollTimer.Tick += new System.EventHandler(this.X360PollTimer_Tick);
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(22, 152);
            this.textBox1.Margin = new System.Windows.Forms.Padding(2);
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(139, 20);
            this.textBox1.TabIndex = 2;
            // 
            // enable360Button
            // 
            this.enable360Button.AutoSize = true;
            this.enable360Button.Location = new System.Drawing.Point(23, 234);
            this.enable360Button.Margin = new System.Windows.Forms.Padding(2);
            this.enable360Button.Name = "enable360Button";
            this.enable360Button.Size = new System.Drawing.Size(58, 17);
            this.enable360Button.TabIndex = 3;
            this.enable360Button.Text = "Enable";
            this.enable360Button.UseVisualStyleBackColor = true;
            this.enable360Button.CheckedChanged += new System.EventHandler(this.enable360Button_CheckedChanged);
            // 
            // disable360Button
            // 
            this.disable360Button.AutoSize = true;
            this.disable360Button.Checked = true;
            this.disable360Button.Location = new System.Drawing.Point(104, 234);
            this.disable360Button.Margin = new System.Windows.Forms.Padding(2);
            this.disable360Button.Name = "disable360Button";
            this.disable360Button.Size = new System.Drawing.Size(60, 17);
            this.disable360Button.TabIndex = 4;
            this.disable360Button.TabStop = true;
            this.disable360Button.Text = "Disable";
            this.disable360Button.UseVisualStyleBackColor = true;
            this.disable360Button.CheckedChanged += new System.EventHandler(this.disable360Button_CheckedChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(32, 217);
            this.label1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(115, 13);
            this.label1.TabIndex = 5;
            this.label1.Text = "X360 Controller Setting";
            // 
            // travelDistanceButton
            // 
            this.travelDistanceButton.Location = new System.Drawing.Point(43, 285);
            this.travelDistanceButton.Margin = new System.Windows.Forms.Padding(2);
            this.travelDistanceButton.Name = "travelDistanceButton";
            this.travelDistanceButton.Size = new System.Drawing.Size(88, 19);
            this.travelDistanceButton.TabIndex = 7;
            this.travelDistanceButton.Text = "Defend";
            this.travelDistanceButton.UseVisualStyleBackColor = true;
            this.travelDistanceButton.Click += new System.EventHandler(this.travelDistanceButton_Click);
            // 
            // xTargetBox
            // 
            this.xTargetBox.Location = new System.Drawing.Point(22, 261);
            this.xTargetBox.Margin = new System.Windows.Forms.Padding(2);
            this.xTargetBox.Name = "xTargetBox";
            this.xTargetBox.Size = new System.Drawing.Size(35, 20);
            this.xTargetBox.TabIndex = 6;
            this.xTargetBox.Text = "10";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(41, 136);
            this.label2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(96, 13);
            this.label2.TabIndex = 8;
            this.label2.Text = "State, Speed CMD";
            // 
            // DistancePollTimer
            // 
            this.DistancePollTimer.Interval = 10;
            this.DistancePollTimer.Tick += new System.EventHandler(this.DistancePollTimer_Tick);
            // 
            // rightSpeedBox
            // 
            this.rightSpeedBox.Location = new System.Drawing.Point(98, 41);
            this.rightSpeedBox.Margin = new System.Windows.Forms.Padding(2);
            this.rightSpeedBox.Name = "rightSpeedBox";
            this.rightSpeedBox.Size = new System.Drawing.Size(36, 20);
            this.rightSpeedBox.TabIndex = 16;
            this.rightSpeedBox.Text = "0";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(45, 24);
            this.label6.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(25, 13);
            this.label6.TabIndex = 17;
            this.label6.Text = "Left";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(97, 24);
            this.label7.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(32, 13);
            this.label7.TabIndex = 18;
            this.label7.Text = "Right";
            // 
            // yTargetBox
            // 
            this.yTargetBox.Location = new System.Drawing.Point(76, 261);
            this.yTargetBox.Margin = new System.Windows.Forms.Padding(2);
            this.yTargetBox.Name = "yTargetBox";
            this.yTargetBox.Size = new System.Drawing.Size(35, 20);
            this.yTargetBox.TabIndex = 19;
            this.yTargetBox.Text = "20";
            // 
            // getStateButton
            // 
            this.getStateButton.Location = new System.Drawing.Point(56, 177);
            this.getStateButton.Name = "getStateButton";
            this.getStateButton.Size = new System.Drawing.Size(75, 23);
            this.getStateButton.TabIndex = 20;
            this.getStateButton.Text = "Get State";
            this.getStateButton.UseVisualStyleBackColor = true;
            this.getStateButton.Click += new System.EventHandler(this.getStateButton_Click);
            // 
            // textBox2
            // 
            this.textBox2.Location = new System.Drawing.Point(33, 409);
            this.textBox2.Margin = new System.Windows.Forms.Padding(2);
            this.textBox2.Name = "textBox2";
            this.textBox2.Size = new System.Drawing.Size(26, 20);
            this.textBox2.TabIndex = 9;
            // 
            // textBox3
            // 
            this.textBox3.Location = new System.Drawing.Point(76, 409);
            this.textBox3.Margin = new System.Windows.Forms.Padding(2);
            this.textBox3.Name = "textBox3";
            this.textBox3.Size = new System.Drawing.Size(26, 20);
            this.textBox3.TabIndex = 10;
            // 
            // textBox4
            // 
            this.textBox4.Location = new System.Drawing.Point(121, 409);
            this.textBox4.Margin = new System.Windows.Forms.Padding(2);
            this.textBox4.Name = "textBox4";
            this.textBox4.Size = new System.Drawing.Size(26, 20);
            this.textBox4.TabIndex = 11;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(41, 431);
            this.label3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(14, 13);
            this.label3.TabIndex = 12;
            this.label3.Text = "P";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(86, 431);
            this.label4.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(10, 13);
            this.label4.TabIndex = 13;
            this.label4.Text = "I";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(127, 431);
            this.label5.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(15, 13);
            this.label5.TabIndex = 14;
            this.label5.Text = "D";
            // 
            // setGainsButton
            // 
            this.setGainsButton.Location = new System.Drawing.Point(56, 449);
            this.setGainsButton.Margin = new System.Windows.Forms.Padding(2);
            this.setGainsButton.Name = "setGainsButton";
            this.setGainsButton.Size = new System.Drawing.Size(67, 19);
            this.setGainsButton.TabIndex = 15;
            this.setGainsButton.Text = "Set Gains";
            this.setGainsButton.UseVisualStyleBackColor = true;
            this.setGainsButton.Click += new System.EventHandler(this.setGainsButton_Click);
            // 
            // stopButton
            // 
            this.stopButton.Location = new System.Drawing.Point(22, 96);
            this.stopButton.Name = "stopButton";
            this.stopButton.Size = new System.Drawing.Size(139, 32);
            this.stopButton.TabIndex = 21;
            this.stopButton.Text = "STOP!!";
            this.stopButton.UseVisualStyleBackColor = true;
            this.stopButton.Click += new System.EventHandler(this.stopButton_Click);
            // 
            // tTargetBox
            // 
            this.tTargetBox.Location = new System.Drawing.Point(126, 261);
            this.tTargetBox.Margin = new System.Windows.Forms.Padding(2);
            this.tTargetBox.Name = "tTargetBox";
            this.tTargetBox.Size = new System.Drawing.Size(35, 20);
            this.tTargetBox.TabIndex = 22;
            this.tTargetBox.Text = "0.01";
            // 
            // tAttackBox
            // 
            this.tAttackBox.Location = new System.Drawing.Point(97, 323);
            this.tAttackBox.Margin = new System.Windows.Forms.Padding(2);
            this.tAttackBox.Name = "tAttackBox";
            this.tAttackBox.Size = new System.Drawing.Size(35, 20);
            this.tAttackBox.TabIndex = 26;
            this.tAttackBox.Text = "0.5";
            // 
            // yAttackBox
            // 
            this.yAttackBox.Location = new System.Drawing.Point(56, 323);
            this.yAttackBox.Margin = new System.Windows.Forms.Padding(2);
            this.yAttackBox.Name = "yAttackBox";
            this.yAttackBox.Size = new System.Drawing.Size(35, 20);
            this.yAttackBox.TabIndex = 25;
            this.yAttackBox.Text = "15";
            // 
            // attackButton
            // 
            this.attackButton.Location = new System.Drawing.Point(44, 347);
            this.attackButton.Margin = new System.Windows.Forms.Padding(2);
            this.attackButton.Name = "attackButton";
            this.attackButton.Size = new System.Drawing.Size(88, 19);
            this.attackButton.TabIndex = 24;
            this.attackButton.Text = "Attack";
            this.attackButton.UseVisualStyleBackColor = true;
            this.attackButton.Click += new System.EventHandler(this.attackButton_Click);
            // 
            // xAttackBox
            // 
            this.xAttackBox.Location = new System.Drawing.Point(11, 323);
            this.xAttackBox.Margin = new System.Windows.Forms.Padding(2);
            this.xAttackBox.Name = "xAttackBox";
            this.xAttackBox.Size = new System.Drawing.Size(35, 20);
            this.xAttackBox.TabIndex = 23;
            this.xAttackBox.Text = "20";
            // 
            // aAttackBox
            // 
            this.aAttackBox.Location = new System.Drawing.Point(139, 323);
            this.aAttackBox.Margin = new System.Windows.Forms.Padding(2);
            this.aAttackBox.Name = "aAttackBox";
            this.aAttackBox.Size = new System.Drawing.Size(35, 20);
            this.aAttackBox.TabIndex = 27;
            this.aAttackBox.Text = "0";
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(18, 381);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(143, 23);
            this.button1.TabIndex = 28;
            this.button1.Text = "Defend + Attack";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(185, 492);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.aAttackBox);
            this.Controls.Add(this.tAttackBox);
            this.Controls.Add(this.yAttackBox);
            this.Controls.Add(this.attackButton);
            this.Controls.Add(this.xAttackBox);
            this.Controls.Add(this.tTargetBox);
            this.Controls.Add(this.stopButton);
            this.Controls.Add(this.getStateButton);
            this.Controls.Add(this.yTargetBox);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.rightSpeedBox);
            this.Controls.Add(this.setGainsButton);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.textBox4);
            this.Controls.Add(this.textBox3);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.travelDistanceButton);
            this.Controls.Add(this.xTargetBox);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.disable360Button);
            this.Controls.Add(this.enable360Button);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.updateSpeedButton);
            this.Controls.Add(this.leftSpeedBox);
            this.Margin = new System.Windows.Forms.Padding(2);
            this.Name = "Form1";
            this.Text = "Form1";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox leftSpeedBox;
        private System.Windows.Forms.Button updateSpeedButton;
        private System.Windows.Forms.Timer X360PollTimer;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.RadioButton enable360Button;
        private System.Windows.Forms.RadioButton disable360Button;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button travelDistanceButton;
        private System.Windows.Forms.TextBox xTargetBox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Timer DistancePollTimer;
        private System.Windows.Forms.TextBox rightSpeedBox;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox yTargetBox;
        private System.Windows.Forms.Button getStateButton;
        private System.Windows.Forms.TextBox textBox2;
        private System.Windows.Forms.TextBox textBox3;
        private System.Windows.Forms.TextBox textBox4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Button setGainsButton;
        private System.Windows.Forms.Button stopButton;
        private System.Windows.Forms.TextBox tTargetBox;
        private System.Windows.Forms.TextBox tAttackBox;
        private System.Windows.Forms.TextBox yAttackBox;
        private System.Windows.Forms.Button attackButton;
        private System.Windows.Forms.TextBox xAttackBox;
        private System.Windows.Forms.TextBox aAttackBox;
        private System.Windows.Forms.Button button1;
    }
}

