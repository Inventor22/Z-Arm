namespace Z_Arm_Sliders
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
            this.connectButton = new System.Windows.Forms.Button();
            this.debugTextbox = new System.Windows.Forms.TextBox();
            this.wristTrackbar = new System.Windows.Forms.TrackBar();
            this.labelWrist = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.elbowTrackbar = new System.Windows.Forms.TrackBar();
            this.shoulderTrackBar = new System.Windows.Forms.TrackBar();
            this.zAxisTrackbar = new System.Windows.Forms.TrackBar();
            this.speedTextbox = new System.Windows.Forms.TextBox();
            this.speedLabel = new System.Windows.Forms.Label();
            this.wristTextbox = new System.Windows.Forms.TextBox();
            this.elbowTextbox = new System.Windows.Forms.TextBox();
            this.shoulderTextbox = new System.Windows.Forms.TextBox();
            this.zAxisTextbox = new System.Windows.Forms.TextBox();
            this.homeButton = new System.Windows.Forms.Button();
            this.angularSpeedTextbox = new System.Windows.Forms.TextBox();
            this.angularSpeedLabel = new System.Windows.Forms.Label();
            this.stopStartButton = new System.Windows.Forms.Button();
            this.teachButton = new System.Windows.Forms.Button();
            this.circleButton = new System.Windows.Forms.Button();
            this.printCoords = new System.Windows.Forms.Button();
            this.servoStateButton = new System.Windows.Forms.Button();
            this.rndButton = new System.Windows.Forms.Button();
            this.textBoxRnd1 = new System.Windows.Forms.TextBox();
            this.textBoxRnd2 = new System.Windows.Forms.TextBox();
            this.recordBtn = new System.Windows.Forms.Button();
            this.pipeButton = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.wristTrackbar)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.elbowTrackbar)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.shoulderTrackBar)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.zAxisTrackbar)).BeginInit();
            this.SuspendLayout();
            // 
            // connectButton
            // 
            this.connectButton.Location = new System.Drawing.Point(13, 13);
            this.connectButton.Name = "connectButton";
            this.connectButton.Size = new System.Drawing.Size(126, 49);
            this.connectButton.TabIndex = 0;
            this.connectButton.Text = "Connect";
            this.connectButton.UseVisualStyleBackColor = true;
            this.connectButton.Click += new System.EventHandler(this.connectButton_Click);
            // 
            // debugTextbox
            // 
            this.debugTextbox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.debugTextbox.Location = new System.Drawing.Point(12, 324);
            this.debugTextbox.Multiline = true;
            this.debugTextbox.Name = "debugTextbox";
            this.debugTextbox.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.debugTextbox.Size = new System.Drawing.Size(497, 292);
            this.debugTextbox.TabIndex = 1;
            // 
            // wristTrackbar
            // 
            this.wristTrackbar.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.wristTrackbar.Location = new System.Drawing.Point(73, 120);
            this.wristTrackbar.Maximum = 180;
            this.wristTrackbar.Minimum = -180;
            this.wristTrackbar.Name = "wristTrackbar";
            this.wristTrackbar.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.wristTrackbar.RightToLeftLayout = true;
            this.wristTrackbar.Size = new System.Drawing.Size(436, 45);
            this.wristTrackbar.TabIndex = 2;
            this.wristTrackbar.TickFrequency = 10;
            this.wristTrackbar.Scroll += new System.EventHandler(this.wristTrackbar_Scroll);
            this.wristTrackbar.ValueChanged += new System.EventHandler(this.wristTrackbar_ValueChanged);
            this.wristTrackbar.MouseUp += new System.Windows.Forms.MouseEventHandler(this.wristTrackbar_MouseUp);
            // 
            // labelWrist
            // 
            this.labelWrist.AutoSize = true;
            this.labelWrist.Location = new System.Drawing.Point(18, 120);
            this.labelWrist.Name = "labelWrist";
            this.labelWrist.Size = new System.Drawing.Size(31, 13);
            this.labelWrist.TabIndex = 3;
            this.labelWrist.Text = "Wrist";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(18, 171);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(36, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Elbow";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(18, 222);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(49, 13);
            this.label3.TabIndex = 5;
            this.label3.Text = "Shoulder";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(18, 273);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(36, 13);
            this.label4.TabIndex = 6;
            this.label4.Text = "Z Axis";
            // 
            // elbowTrackbar
            // 
            this.elbowTrackbar.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.elbowTrackbar.Location = new System.Drawing.Point(74, 171);
            this.elbowTrackbar.Maximum = 164;
            this.elbowTrackbar.Minimum = -164;
            this.elbowTrackbar.Name = "elbowTrackbar";
            this.elbowTrackbar.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.elbowTrackbar.RightToLeftLayout = true;
            this.elbowTrackbar.Size = new System.Drawing.Size(434, 45);
            this.elbowTrackbar.TabIndex = 7;
            this.elbowTrackbar.TickFrequency = 10;
            this.elbowTrackbar.Scroll += new System.EventHandler(this.elbowTrackbar_Scroll);
            this.elbowTrackbar.ValueChanged += new System.EventHandler(this.elbowTrackbar_ValueChanged);
            this.elbowTrackbar.MouseUp += new System.Windows.Forms.MouseEventHandler(this.elbowTrackbar_MouseUp);
            // 
            // shoulderTrackBar
            // 
            this.shoulderTrackBar.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.shoulderTrackBar.Location = new System.Drawing.Point(74, 222);
            this.shoulderTrackBar.Maximum = 90;
            this.shoulderTrackBar.Minimum = -90;
            this.shoulderTrackBar.Name = "shoulderTrackBar";
            this.shoulderTrackBar.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
            this.shoulderTrackBar.RightToLeftLayout = true;
            this.shoulderTrackBar.Size = new System.Drawing.Size(435, 45);
            this.shoulderTrackBar.TabIndex = 8;
            this.shoulderTrackBar.TickFrequency = 10;
            this.shoulderTrackBar.Scroll += new System.EventHandler(this.shoulderTrackBar_Scroll);
            this.shoulderTrackBar.ValueChanged += new System.EventHandler(this.shoulderTrackBar_ValueChanged);
            this.shoulderTrackBar.MouseUp += new System.Windows.Forms.MouseEventHandler(this.shoulderTrackBar_MouseUp);
            // 
            // zAxisTrackbar
            // 
            this.zAxisTrackbar.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.zAxisTrackbar.Location = new System.Drawing.Point(74, 273);
            this.zAxisTrackbar.Maximum = 0;
            this.zAxisTrackbar.Minimum = -310;
            this.zAxisTrackbar.Name = "zAxisTrackbar";
            this.zAxisTrackbar.Size = new System.Drawing.Size(435, 45);
            this.zAxisTrackbar.TabIndex = 9;
            this.zAxisTrackbar.TickFrequency = 10;
            this.zAxisTrackbar.Scroll += new System.EventHandler(this.zAxisTrackbar_Scroll);
            this.zAxisTrackbar.ValueChanged += new System.EventHandler(this.zAxisTrackbar_ValueChanged);
            this.zAxisTrackbar.MouseUp += new System.Windows.Forms.MouseEventHandler(this.zAxisTrackbar_MouseUp);
            // 
            // speedTextbox
            // 
            this.speedTextbox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.speedTextbox.Location = new System.Drawing.Point(409, 42);
            this.speedTextbox.Name = "speedTextbox";
            this.speedTextbox.Size = new System.Drawing.Size(100, 20);
            this.speedTextbox.TabIndex = 10;
            this.speedTextbox.Text = "100";
            this.speedTextbox.TextChanged += new System.EventHandler(this.speedTextbox_TextChanged);
            // 
            // speedLabel
            // 
            this.speedLabel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.speedLabel.AutoSize = true;
            this.speedLabel.Location = new System.Drawing.Point(286, 45);
            this.speedLabel.Name = "speedLabel";
            this.speedLabel.Size = new System.Drawing.Size(117, 13);
            this.speedLabel.TabIndex = 11;
            this.speedLabel.Text = "Vertical Velocity [mm/s]";
            // 
            // wristTextbox
            // 
            this.wristTextbox.Location = new System.Drawing.Point(21, 137);
            this.wristTextbox.Name = "wristTextbox";
            this.wristTextbox.Size = new System.Drawing.Size(46, 20);
            this.wristTextbox.TabIndex = 12;
            // 
            // elbowTextbox
            // 
            this.elbowTextbox.Location = new System.Drawing.Point(21, 188);
            this.elbowTextbox.Name = "elbowTextbox";
            this.elbowTextbox.Size = new System.Drawing.Size(46, 20);
            this.elbowTextbox.TabIndex = 13;
            // 
            // shoulderTextbox
            // 
            this.shoulderTextbox.Location = new System.Drawing.Point(21, 239);
            this.shoulderTextbox.Name = "shoulderTextbox";
            this.shoulderTextbox.Size = new System.Drawing.Size(46, 20);
            this.shoulderTextbox.TabIndex = 14;
            // 
            // zAxisTextbox
            // 
            this.zAxisTextbox.Location = new System.Drawing.Point(21, 290);
            this.zAxisTextbox.Name = "zAxisTextbox";
            this.zAxisTextbox.Size = new System.Drawing.Size(46, 20);
            this.zAxisTextbox.TabIndex = 15;
            // 
            // homeButton
            // 
            this.homeButton.Location = new System.Drawing.Point(146, 13);
            this.homeButton.Name = "homeButton";
            this.homeButton.Size = new System.Drawing.Size(49, 23);
            this.homeButton.TabIndex = 16;
            this.homeButton.Text = "Home";
            this.homeButton.UseVisualStyleBackColor = true;
            this.homeButton.Click += new System.EventHandler(this.homeButton_Click);
            // 
            // angularSpeedTextbox
            // 
            this.angularSpeedTextbox.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.angularSpeedTextbox.Location = new System.Drawing.Point(409, 16);
            this.angularSpeedTextbox.Name = "angularSpeedTextbox";
            this.angularSpeedTextbox.Size = new System.Drawing.Size(100, 20);
            this.angularSpeedTextbox.TabIndex = 17;
            this.angularSpeedTextbox.Text = "100";
            this.angularSpeedTextbox.TextChanged += new System.EventHandler(this.angularSpeedTextbox_TextChanged);
            // 
            // angularSpeedLabel
            // 
            this.angularSpeedLabel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.angularSpeedLabel.AutoSize = true;
            this.angularSpeedLabel.Location = new System.Drawing.Point(283, 19);
            this.angularSpeedLabel.Name = "angularSpeedLabel";
            this.angularSpeedLabel.Size = new System.Drawing.Size(120, 13);
            this.angularSpeedLabel.TabIndex = 18;
            this.angularSpeedLabel.Text = "Angular Velocity [deg/s]";
            // 
            // stopStartButton
            // 
            this.stopStartButton.Location = new System.Drawing.Point(146, 39);
            this.stopStartButton.Name = "stopStartButton";
            this.stopStartButton.Size = new System.Drawing.Size(49, 23);
            this.stopStartButton.TabIndex = 16;
            this.stopStartButton.Text = "Loop";
            this.stopStartButton.UseVisualStyleBackColor = true;
            this.stopStartButton.Click += new System.EventHandler(this.stopButton_Click);
            // 
            // teachButton
            // 
            this.teachButton.Location = new System.Drawing.Point(198, 13);
            this.teachButton.Name = "teachButton";
            this.teachButton.Size = new System.Drawing.Size(49, 23);
            this.teachButton.TabIndex = 16;
            this.teachButton.Text = "Teach";
            this.teachButton.UseVisualStyleBackColor = true;
            this.teachButton.Click += new System.EventHandler(this.teachButton_Click);
            // 
            // circleButton
            // 
            this.circleButton.Location = new System.Drawing.Point(202, 39);
            this.circleButton.Name = "circleButton";
            this.circleButton.Size = new System.Drawing.Size(45, 23);
            this.circleButton.TabIndex = 19;
            this.circleButton.Text = "Circle";
            this.circleButton.UseVisualStyleBackColor = true;
            this.circleButton.Click += new System.EventHandler(this.circleButton_Click);
            // 
            // printCoords
            // 
            this.printCoords.Location = new System.Drawing.Point(146, 69);
            this.printCoords.Name = "printCoords";
            this.printCoords.Size = new System.Drawing.Size(49, 23);
            this.printCoords.TabIndex = 20;
            this.printCoords.Text = "print";
            this.printCoords.UseVisualStyleBackColor = true;
            this.printCoords.Click += new System.EventHandler(this.printCoordsButton_Click);
            // 
            // servoStateButton
            // 
            this.servoStateButton.Location = new System.Drawing.Point(202, 69);
            this.servoStateButton.Name = "servoStateButton";
            this.servoStateButton.Size = new System.Drawing.Size(72, 23);
            this.servoStateButton.TabIndex = 21;
            this.servoStateButton.Text = "ServoOff";
            this.servoStateButton.UseVisualStyleBackColor = true;
            this.servoStateButton.Click += new System.EventHandler(this.ServoState_Click);
            // 
            // rndButton
            // 
            this.rndButton.Location = new System.Drawing.Point(286, 68);
            this.rndButton.Name = "rndButton";
            this.rndButton.Size = new System.Drawing.Size(75, 23);
            this.rndButton.TabIndex = 22;
            this.rndButton.Text = "Function1";
            this.rndButton.UseVisualStyleBackColor = true;
            this.rndButton.Click += new System.EventHandler(this.RndButton_Click);
            // 
            // textBoxRnd1
            // 
            this.textBoxRnd1.Location = new System.Drawing.Point(408, 68);
            this.textBoxRnd1.Name = "textBoxRnd1";
            this.textBoxRnd1.Size = new System.Drawing.Size(100, 20);
            this.textBoxRnd1.TabIndex = 23;
            this.textBoxRnd1.Text = "50";
            // 
            // textBoxRnd2
            // 
            this.textBoxRnd2.Location = new System.Drawing.Point(409, 94);
            this.textBoxRnd2.Name = "textBoxRnd2";
            this.textBoxRnd2.Size = new System.Drawing.Size(100, 20);
            this.textBoxRnd2.TabIndex = 24;
            this.textBoxRnd2.Text = "100";
            // 
            // recordBtn
            // 
            this.recordBtn.Location = new System.Drawing.Point(13, 68);
            this.recordBtn.Name = "recordBtn";
            this.recordBtn.Size = new System.Drawing.Size(54, 23);
            this.recordBtn.TabIndex = 25;
            this.recordBtn.Text = "Record";
            this.recordBtn.UseVisualStyleBackColor = true;
            this.recordBtn.Click += new System.EventHandler(this.Button1_Click);
            // 
            // pipeButton
            // 
            this.pipeButton.Location = new System.Drawing.Point(74, 68);
            this.pipeButton.Name = "pipeButton";
            this.pipeButton.Size = new System.Drawing.Size(65, 23);
            this.pipeButton.TabIndex = 26;
            this.pipeButton.Text = "Pipe";
            this.pipeButton.UseVisualStyleBackColor = true;
            this.pipeButton.Click += new System.EventHandler(this.PipeButton_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(522, 628);
            this.Controls.Add(this.pipeButton);
            this.Controls.Add(this.recordBtn);
            this.Controls.Add(this.textBoxRnd2);
            this.Controls.Add(this.textBoxRnd1);
            this.Controls.Add(this.rndButton);
            this.Controls.Add(this.servoStateButton);
            this.Controls.Add(this.printCoords);
            this.Controls.Add(this.circleButton);
            this.Controls.Add(this.angularSpeedLabel);
            this.Controls.Add(this.angularSpeedTextbox);
            this.Controls.Add(this.stopStartButton);
            this.Controls.Add(this.teachButton);
            this.Controls.Add(this.homeButton);
            this.Controls.Add(this.zAxisTextbox);
            this.Controls.Add(this.shoulderTextbox);
            this.Controls.Add(this.elbowTextbox);
            this.Controls.Add(this.wristTextbox);
            this.Controls.Add(this.speedLabel);
            this.Controls.Add(this.speedTextbox);
            this.Controls.Add(this.zAxisTrackbar);
            this.Controls.Add(this.shoulderTrackBar);
            this.Controls.Add(this.elbowTrackbar);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.labelWrist);
            this.Controls.Add(this.wristTrackbar);
            this.Controls.Add(this.debugTextbox);
            this.Controls.Add(this.connectButton);
            this.Name = "Form1";
            this.Text = "Form1";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            ((System.ComponentModel.ISupportInitialize)(this.wristTrackbar)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.elbowTrackbar)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.shoulderTrackBar)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.zAxisTrackbar)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button connectButton;
        private System.Windows.Forms.TextBox debugTextbox;
        private System.Windows.Forms.TrackBar wristTrackbar;
        private System.Windows.Forms.Label labelWrist;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TrackBar elbowTrackbar;
        private System.Windows.Forms.TrackBar shoulderTrackBar;
        private System.Windows.Forms.TrackBar zAxisTrackbar;
        private System.Windows.Forms.TextBox speedTextbox;
        private System.Windows.Forms.Label speedLabel;
        private System.Windows.Forms.TextBox wristTextbox;
        private System.Windows.Forms.TextBox elbowTextbox;
        private System.Windows.Forms.TextBox shoulderTextbox;
        private System.Windows.Forms.TextBox zAxisTextbox;
        private System.Windows.Forms.Button homeButton;
        private System.Windows.Forms.TextBox angularSpeedTextbox;
        private System.Windows.Forms.Label angularSpeedLabel;
        private System.Windows.Forms.Button stopStartButton;
        private System.Windows.Forms.Button teachButton;
        private System.Windows.Forms.Button circleButton;
        private System.Windows.Forms.Button printCoords;
        private System.Windows.Forms.Button servoStateButton;
        private System.Windows.Forms.Button rndButton;
        private System.Windows.Forms.TextBox textBoxRnd1;
        private System.Windows.Forms.TextBox textBoxRnd2;
        private System.Windows.Forms.Button recordBtn;
        private System.Windows.Forms.Button pipeButton;
    }
}

