namespace ZArm_Test
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
            this.buttonConnect = new System.Windows.Forms.Button();
            this.debugBox = new System.Windows.Forms.TextBox();
            this.zUpButton = new System.Windows.Forms.Button();
            this.zDownButton = new System.Windows.Forms.Button();
            this.shoulderCcwButton = new System.Windows.Forms.Button();
            this.shoulderCwButton = new System.Windows.Forms.Button();
            this.elbowCcwButton = new System.Windows.Forms.Button();
            this.elbowCwButton = new System.Windows.Forms.Button();
            this.wristCcwButton = new System.Windows.Forms.Button();
            this.wristCwButton = new System.Windows.Forms.Button();
            this.zPosTextbox = new System.Windows.Forms.TextBox();
            this.shoulderPosTextbox = new System.Windows.Forms.TextBox();
            this.elbowPosTextbox = new System.Windows.Forms.TextBox();
            this.wristPosTextbox = new System.Windows.Forms.TextBox();
            this.deltaTextbox = new System.Windows.Forms.TextBox();
            this.speedTextbox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.idButton = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // buttonConnect
            // 
            this.buttonConnect.Location = new System.Drawing.Point(12, 12);
            this.buttonConnect.Name = "buttonConnect";
            this.buttonConnect.Size = new System.Drawing.Size(111, 38);
            this.buttonConnect.TabIndex = 0;
            this.buttonConnect.Text = "Connect";
            this.buttonConnect.UseVisualStyleBackColor = true;
            this.buttonConnect.Click += new System.EventHandler(this.buttonConnect_Click);
            // 
            // debugBox
            // 
            this.debugBox.Location = new System.Drawing.Point(12, 174);
            this.debugBox.Multiline = true;
            this.debugBox.Name = "debugBox";
            this.debugBox.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.debugBox.Size = new System.Drawing.Size(369, 134);
            this.debugBox.TabIndex = 1;
            // 
            // zUpButton
            // 
            this.zUpButton.Location = new System.Drawing.Point(12, 72);
            this.zUpButton.Name = "zUpButton";
            this.zUpButton.Size = new System.Drawing.Size(75, 23);
            this.zUpButton.TabIndex = 2;
            this.zUpButton.Text = "Z Up";
            this.zUpButton.UseVisualStyleBackColor = true;
            this.zUpButton.Click += new System.EventHandler(this.zUpButton_Click);
            // 
            // zDownButton
            // 
            this.zDownButton.Location = new System.Drawing.Point(12, 101);
            this.zDownButton.Name = "zDownButton";
            this.zDownButton.Size = new System.Drawing.Size(75, 23);
            this.zDownButton.TabIndex = 2;
            this.zDownButton.Text = "Z Down";
            this.zDownButton.UseVisualStyleBackColor = true;
            this.zDownButton.Click += new System.EventHandler(this.zDownButton_Click);
            // 
            // shoulderCcwButton
            // 
            this.shoulderCcwButton.Location = new System.Drawing.Point(93, 72);
            this.shoulderCcwButton.Name = "shoulderCcwButton";
            this.shoulderCcwButton.Size = new System.Drawing.Size(99, 23);
            this.shoulderCcwButton.TabIndex = 2;
            this.shoulderCcwButton.Text = "Shoulder Left";
            this.shoulderCcwButton.UseVisualStyleBackColor = true;
            this.shoulderCcwButton.Click += new System.EventHandler(this.shoulderCcwButton_Click);
            // 
            // shoulderCwButton
            // 
            this.shoulderCwButton.Location = new System.Drawing.Point(93, 101);
            this.shoulderCwButton.Name = "shoulderCwButton";
            this.shoulderCwButton.Size = new System.Drawing.Size(99, 23);
            this.shoulderCwButton.TabIndex = 2;
            this.shoulderCwButton.Text = "Shoulder Right";
            this.shoulderCwButton.UseVisualStyleBackColor = true;
            this.shoulderCwButton.Click += new System.EventHandler(this.shoulderCwButton_Click);
            // 
            // elbowCcwButton
            // 
            this.elbowCcwButton.Location = new System.Drawing.Point(225, 72);
            this.elbowCcwButton.Name = "elbowCcwButton";
            this.elbowCcwButton.Size = new System.Drawing.Size(75, 23);
            this.elbowCcwButton.TabIndex = 2;
            this.elbowCcwButton.Text = "Elbow Left";
            this.elbowCcwButton.UseVisualStyleBackColor = true;
            this.elbowCcwButton.Click += new System.EventHandler(this.elbowCcwButton_Click);
            // 
            // elbowCwButton
            // 
            this.elbowCwButton.Location = new System.Drawing.Point(225, 101);
            this.elbowCwButton.Name = "elbowCwButton";
            this.elbowCwButton.Size = new System.Drawing.Size(75, 23);
            this.elbowCwButton.TabIndex = 2;
            this.elbowCwButton.Text = "Elbow Right";
            this.elbowCwButton.UseVisualStyleBackColor = true;
            this.elbowCwButton.Click += new System.EventHandler(this.elbowCwButton_Click);
            // 
            // wristCcwButton
            // 
            this.wristCcwButton.Location = new System.Drawing.Point(306, 72);
            this.wristCcwButton.Name = "wristCcwButton";
            this.wristCcwButton.Size = new System.Drawing.Size(75, 23);
            this.wristCcwButton.TabIndex = 2;
            this.wristCcwButton.Text = "Wrist Left";
            this.wristCcwButton.UseVisualStyleBackColor = true;
            this.wristCcwButton.Click += new System.EventHandler(this.wristCcwButton_Click);
            // 
            // wristCwButton
            // 
            this.wristCwButton.Location = new System.Drawing.Point(306, 101);
            this.wristCwButton.Name = "wristCwButton";
            this.wristCwButton.Size = new System.Drawing.Size(75, 23);
            this.wristCwButton.TabIndex = 2;
            this.wristCwButton.Text = "Wrist Right";
            this.wristCwButton.UseVisualStyleBackColor = true;
            this.wristCwButton.Click += new System.EventHandler(this.wristCwButton_Click);
            // 
            // zPosTextbox
            // 
            this.zPosTextbox.Location = new System.Drawing.Point(13, 131);
            this.zPosTextbox.Name = "zPosTextbox";
            this.zPosTextbox.Size = new System.Drawing.Size(74, 20);
            this.zPosTextbox.TabIndex = 3;
            // 
            // shoulderPosTextbox
            // 
            this.shoulderPosTextbox.Location = new System.Drawing.Point(93, 131);
            this.shoulderPosTextbox.Name = "shoulderPosTextbox";
            this.shoulderPosTextbox.Size = new System.Drawing.Size(99, 20);
            this.shoulderPosTextbox.TabIndex = 3;
            // 
            // elbowPosTextbox
            // 
            this.elbowPosTextbox.Location = new System.Drawing.Point(225, 130);
            this.elbowPosTextbox.Name = "elbowPosTextbox";
            this.elbowPosTextbox.Size = new System.Drawing.Size(75, 20);
            this.elbowPosTextbox.TabIndex = 3;
            // 
            // wristPosTextbox
            // 
            this.wristPosTextbox.Location = new System.Drawing.Point(306, 130);
            this.wristPosTextbox.Name = "wristPosTextbox";
            this.wristPosTextbox.Size = new System.Drawing.Size(75, 20);
            this.wristPosTextbox.TabIndex = 3;
            // 
            // deltaTextbox
            // 
            this.deltaTextbox.Location = new System.Drawing.Point(306, 12);
            this.deltaTextbox.Name = "deltaTextbox";
            this.deltaTextbox.Size = new System.Drawing.Size(75, 20);
            this.deltaTextbox.TabIndex = 3;
            this.deltaTextbox.TextChanged += new System.EventHandler(this.deltaTextbox_TextChanged);
            // 
            // speedTextbox
            // 
            this.speedTextbox.Location = new System.Drawing.Point(306, 38);
            this.speedTextbox.Name = "speedTextbox";
            this.speedTextbox.Size = new System.Drawing.Size(75, 20);
            this.speedTextbox.TabIndex = 3;
            this.speedTextbox.TextChanged += new System.EventHandler(this.speedTextbox_TextChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(243, 15);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(57, 13);
            this.label1.TabIndex = 4;
            this.label1.Text = "Delta [mm]";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(226, 41);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(74, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Speed [mm/2]";
            // 
            // idButton
            // 
            this.idButton.Location = new System.Drawing.Point(129, 15);
            this.idButton.Name = "idButton";
            this.idButton.Size = new System.Drawing.Size(72, 35);
            this.idButton.TabIndex = 2;
            this.idButton.Text = "Get Id";
            this.idButton.UseVisualStyleBackColor = true;
            this.idButton.Click += new System.EventHandler(this.idButton_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(393, 320);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.speedTextbox);
            this.Controls.Add(this.deltaTextbox);
            this.Controls.Add(this.wristPosTextbox);
            this.Controls.Add(this.elbowPosTextbox);
            this.Controls.Add(this.shoulderPosTextbox);
            this.Controls.Add(this.zPosTextbox);
            this.Controls.Add(this.wristCwButton);
            this.Controls.Add(this.elbowCwButton);
            this.Controls.Add(this.shoulderCwButton);
            this.Controls.Add(this.wristCcwButton);
            this.Controls.Add(this.elbowCcwButton);
            this.Controls.Add(this.shoulderCcwButton);
            this.Controls.Add(this.zDownButton);
            this.Controls.Add(this.idButton);
            this.Controls.Add(this.zUpButton);
            this.Controls.Add(this.debugBox);
            this.Controls.Add(this.buttonConnect);
            this.Name = "Form1";
            this.Text = "Form1";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.Load += new System.EventHandler(this.Form1_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button buttonConnect;
        private System.Windows.Forms.TextBox debugBox;
        private System.Windows.Forms.Button zUpButton;
        private System.Windows.Forms.Button zDownButton;
        private System.Windows.Forms.Button shoulderCcwButton;
        private System.Windows.Forms.Button shoulderCwButton;
        private System.Windows.Forms.Button elbowCcwButton;
        private System.Windows.Forms.Button elbowCwButton;
        private System.Windows.Forms.Button wristCcwButton;
        private System.Windows.Forms.Button wristCwButton;
        private System.Windows.Forms.TextBox zPosTextbox;
        private System.Windows.Forms.TextBox shoulderPosTextbox;
        private System.Windows.Forms.TextBox elbowPosTextbox;
        private System.Windows.Forms.TextBox wristPosTextbox;
        private System.Windows.Forms.TextBox deltaTextbox;
        private System.Windows.Forms.TextBox speedTextbox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button idButton;
    }
}

