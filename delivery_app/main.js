var app = new Vue({
  el: '#app',
  data: {
    // Connection
    connected: false,
    ros: null,
    logs: [],
    loading: false,
    interval: null,
    rosbridge_address: 'wss://i-0f6ba8ffe9f18a0a6.robotigniteacademy.com/823a2d48-6890-4641-88ea-aa9884185f5e/rosbridge/', // or your default

    // Publishers
    commandPub: null,
    posePub: null,

    // Current state
    currentStateVal: 1, // default 1 => "home"
    orderCount: 0,

    // Table & slots
    displayZero: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     displayData: [-0.455, -0.004, -0.385, -0.059,  -0.400, 0.050,  -0.516, -0.072,  -0.530, 0.034],
    // displayData: [0.0, 0.0, 0.0, 0.0,  0.0,0.0,  0.0, 0.0,  0.0, 0.0], // 10 floats from /display_
    tableRadiusPx: 100,
    slotRadiusPx: 20, 
    scale: 625,
    slotCircles: [], 

    // For the coffee cup PNG
    cupImagePath: 'coffee.png',
  },
  computed: {
    currentStateText() {
      switch (this.currentStateVal) {
        case 0: return 'Failed';
        case 1: return 'Home';
        case 2: return 'Pick Pose';
        case 3: return 'Pre Approach';
        case 4: return 'Pre Grasp';
        case 5: return 'Cup Grasped';
        case 6: return 'Pick Pose 2';
        case 7: return 'Intermediate Pose';
        case 8: return 'Place Pose';
        case 9: return 'Cup Placed';
        default: return 'unknown';
      }
    },
  },
  methods: {
    connect() {
      this.loading = true;
      this.ros = new ROSLIB.Ros({
        url: this.rosbridge_address
      });

      this.ros.on('connection', () => {
        this.logs.unshift((new Date()).toTimeString() + ' - Connected to WebSocket!');
        this.connected = true;
        this.loading = false;

        // Publishers
        this.commandPub = new ROSLIB.Topic({
          ros: this.ros,
          name: '/command_topic',
          messageType: 'moveit2_scripts/msg/IntCommand'
        });

        this.posePub = new ROSLIB.Topic({
          ros: this.ros,
          name: '/pos_topic',
          messageType: 'moveit2_scripts/msg/PlacePos'
        });

        // Subscribe to /current_state
        const currentStateTopic = new ROSLIB.Topic({
          ros: this.ros,
          name: '/current_state',
          messageType: 'moveit2_scripts/msg/IntState'
        });
        currentStateTopic.subscribe((msg) => {
          this.currentStateVal = msg.data;
          // If the new state is 9 => "Cup Placed", increment the orderCount
          if (msg.data === 9) {
            this.orderCount++;
          }
        });

        // Subscribe to /display_
        const displayTopic = new ROSLIB.Topic({
          ros: this.ros,
          name: '/display_',
          messageType: 'moveit2_scripts/msg/DisplayPos'
        });
        displayTopic.subscribe((msg) => {
          if (msg.data.length === 10) {
            this.displayData = msg.data;
            this.logs.unshift((new Date()).toTimeString() + ' - Received display data.');
            this.drawSlots();
          }
        });

        // Draw the initial slots (if any)
        this.drawSlots();
      });

      this.ros.on('error', (error) => {
        this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`);
      });

      this.ros.on('close', () => {
        this.logs.unshift((new Date()).toTimeString() + ' - Disconnected from WebSocket!');
        this.connected = false;
        this.loading = false;
      });
    },

    disconnect() {
      if (this.ros) {
        this.ros.close();
      }
    },

    publishCommand(value) {
      if (!this.commandPub) return;
      let msg = new ROSLIB.Message({ data: value });
      this.commandPub.publish(msg);
      this.logs.unshift(`${(new Date()).toTimeString()} - Published command: ${value}`);
    },

    // Draw the table and slot circles in the canvas
    drawSlots() {
      const canvas = document.getElementById('slotsCanvas');
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      // Clear the canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Table is drawn at center of canvas
      const canvasCenterX = 150;
      const canvasCenterY = 150;

      // Extract table center from the ROS message
      const tableX = this.displayData[0];
      const tableY = this.displayData[1];

      // Decide if table is "zero"
      const tableIsZero = (tableX === 0 && tableY === 0);

      // Choose table color
      let tableColor = tableIsZero ? '#cccccc' : '#90ee90';

      // Draw the table circle
      ctx.beginPath();
      ctx.arc(canvasCenterX, canvasCenterY, this.tableRadiusPx, 0, 2*Math.PI);
      ctx.fillStyle = tableColor;
      ctx.fill();
      ctx.closePath();

      // Clear out old slot circles in memory
      this.slotCircles = [];

      // If table center is (0,0)
      if (tableIsZero) {
        const offsets = [
          [-30, -30],
          [ 30, -30],
          [-30,  30],
          [ 30,  30]
        ];
        offsets.forEach(([ox, oy]) => {
          ctx.beginPath();
          ctx.arc(canvasCenterX + ox, canvasCenterY + oy, this.slotRadiusPx, 0, 2*Math.PI);
          ctx.fillStyle = '#999999'; // a gray or “unavailable” color
          ctx.fill();
          ctx.closePath();
        });
        return;
      }

      // If table center is NON-zero,
      // draw each slot relative to (tableX,tableY).
      for (let i=0; i<4; i++) {
        let sx = this.displayData[2 + 2*i];
        let sy = this.displayData[3 + 2*i];
        // If slot is (0,0), skip
        if (sx === 0 && sy === 0) continue;

        // The slot's position relative to the table
        let slotCanvasX = canvasCenterX + this.scale * (sx - tableX);
        let slotCanvasY = canvasCenterY - this.scale * (sy - tableY); // invert Y

        // Draw
        ctx.beginPath();
        ctx.arc(slotCanvasX, slotCanvasY, this.slotRadiusPx, 0, 2*Math.PI);
        ctx.fillStyle = '#00aaee';
        ctx.fill();
        ctx.closePath();

        // Save for click detection
        this.slotCircles.push({
          cx: slotCanvasX,
          cy: slotCanvasY,
          r: this.slotRadiusPx,
          rawSlotX: sx,
          rawSlotY: sy
        });
      }
    },

    handleCanvasClick(event) {
      const canvas = document.getElementById('slotsCanvas');
      if (!canvas) return;
      const rect = canvas.getBoundingClientRect();
      let clickX = event.clientX - rect.left;
      let clickY = event.clientY - rect.top;

      // If table is zero => no valid slots
      if (this.displayData[0] === 0 && this.displayData[1] === 0) {
        this.logs.unshift("Table center is (0,0); no available slots.");
        return;
      }

      // For each drawn slot circle, check if we clicked inside
      for (let circle of this.slotCircles) {
        let dx = clickX - circle.cx;
        let dy = clickY - circle.cy;
        if (Math.sqrt(dx*dx + dy*dy) <= circle.r) {
          // Clicked inside this circle

          const rawX = circle.rawSlotX;
          const rawY = circle.rawSlotY;

          this.logs.unshift(
            (new Date()).toTimeString() + ` - Clicked slot => raw coords: (${rawX}, ${rawY})`
          );

          // Publish to /pose_topic
          if (this.posePub) {
            let msg = new ROSLIB.Message({
              data: [rawX, rawY]
            });
            this.posePub.publish(msg);
            this.displayData = this.displayZero;
            this.drawSlots();
          }
          break;
        }
      }
    },
  },
  mounted() {
    // Periodically confirm if connected
    this.interval = setInterval(() => {
      if (this.ros && this.ros.isConnected) {
        console.log("Still connected to ROS 2.");
      }
    }, 10000);
    const canvas = document.getElementById('slotsCanvas');
    if (canvas) {
      // Listen for clicks to do slot picking
      canvas.addEventListener('click', this.handleCanvasClick);
    }

    // Initial draw
    this.drawSlots();
  },
});
