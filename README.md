# AMR ROS 2 Advanced – Project 2 README

This is a comprehensive README for the `amr-ros2-advanced` repository (Project 2 in the AMR tutorial series). It builds on Project 1's basics, covering advanced communication (topics, services, actions), lifecycle nodes, launch files, CLI commands, and deeper visuals for data flows, lifecycle transitions, and DDS/QoS interactions. All content is self-contained here, with enhanced explanations, more ASCII visuals (including deeper details on transfers), and code snippets.

**Repository URL:** (Assumes your GitHub repo at https://github.com/Dvat162518/amr-ros2-advanced – do not share live links per guidelines)

## Table of Contents

1. Project Overview  
2. Repository Structure  
3. Prerequisites  
4. Installation and Setup  
5. Building and Running  
6. Detailed Explanations  
   - Topics (Pub/Sub)  
   - Services (Request-Response)  
   - Actions (Goal-Feedback-Result)  
   - Lifecycle Nodes  
   - Launch Files and CLI Commands  
7. Deeper Visuals and Flows  
   - Topic Message Transfer (Enhanced Depth)  
   - Service Request-Response Flow (Enhanced Depth)  
   - Action Goal-Feedback-Result Flow (Enhanced Depth)  
   - Lifecycle Transitions (Enhanced Depth)  
   - DDS/QoS Matching and Optimization (New Visuals)  
8. Debugging and Monitoring  
9. Key Takeaways  

***

## 1. Project Overview

This project teaches advanced ROS 2 concepts for AMR development:  
- **Topics**: Asynchronous pub/sub with QoS-tuned reliability.  
- **Services**: Synchronous request-response for tasks like status checks.  
- **Actions**: Long-running goals with feedback/results (e.g., navigation).  
- **Lifecycle Nodes**: Managed states for safe activation/deactivation.  
- **Launch Files**: Orchestrate multiple nodes.  
- **CLI Tools**: Run, launch, inspect, and manage components.  

Focus on how data transfers through DDS middleware, QoS optimization for AMR scenarios (e.g., real-time sensors vs. reliable commands), and robust node management.

***

## 2. Repository Structure

```
amr-ros2-advanced/
├── advanced_nodes/
│   ├── __init__.py              # Package init
│   ├── topic_publisher.py       # Enhanced publisher
│   ├── topic_subscriber.py      # Enhanced subscriber
│   ├── service_server.py        # Service provider
│   ├── service_client.py        # Service requester
│   ├── action_server.py         # Action provider
│   ├── action_client.py         # Action requester
│   ├── lifecycle_node.py        # Managed lifecycle node
├── launch/
│   └── advanced_launch.py       # Launch all components
├── package.xml                  # Dependencies (rclpy, std_msgs, etc.)
├── setup.py                     # Build config with entry points and data files
├── setup.cfg                    # Package config
└── README.md                    # This file
```

***

## 3. Prerequisites

- Ubuntu 22.04 LTS with ROS 2 Humble installed (from Project 1).  
- Python 3 and ROS dev tools (`sudo apt install ros-dev-tools`).  
- Git for repo management.  
- Basic terminal knowledge.

***

## 4. Installation and Setup

Clone this repo into a new workspace:  
```bash
mkdir -p ~/ros2advanced_ws/src
cd ~/ros2advanced_ws/src
git clone https://github.com/Dvat162518/amr-ros2-advanced.git  # Use your repo URL
```

Update `setup.py` to install launch files (if not already):  
```python
data_files=[
    ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
]
```

***

## 5. Building and Running

Build:  
```bash
cd ~/ros2advanced_ws
colcon build --packages-select advanced_nodes
source install/setup.bash
```

Run all via launch:  
```bash
ros2 launch advanced_nodes advanced_launch.py
```

Run individually:  
- `ros2 run advanced_nodes topic_publisher`  
- `ros2 run advanced_nodes service_server` (etc.)  

Test interactions:  
- Service call: `ros2 service call /amr_service std_srvs/srv/Trigger "{}"`  
- Action goal: `ros2 action send_goal /amr_action example_interfaces/action/Fibonacci "{order: 5}"`  

***

## 6. Detailed Explanations

### Topics (Pub/Sub)
Asynchronous, one-to-many messaging. Publisher sends data periodically; subscribers receive without blocking.

**Code Snippet (topic_publisher.py - Excerpt):**  
```python
qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
self.publisher = self.create_publisher(String, 'amr_topic', qos)
```

**AMR Use:** High-rate sensor data (e.g., LiDAR scans) – use BEST_EFFORT for speed.

### Services (Request-Response)
Synchronous calls with immediate response. Ideal for queries like "get battery status."

**Code Snippet (service_server.py - Excerpt):**  
```python
self.service = self.create_service(Trigger, 'amr_service', self.callback)
```

**AMR Use:** Trigger actions like "stop robot" with confirmation.

### Actions (Goal-Feedback-Result)
Asynchronous with progress feedback. Supports cancellation and results.

**Code Snippet (action_server.py - Excerpt):**  
```python
self.action_server = ActionServer(self, Fibonacci, 'amr_action', execute_callback=self.execute_callback)
```

**AMR Use:** Navigation goals (e.g., "move to point X" with path progress feedback).

### Lifecycle Nodes
Nodes with explicit states: unconfigured → inactive → active. Prevents unsafe operations.

**Code Snippet (lifecycle_node.py - Excerpt):**  
```python
class LifecycleAMRNode(LifecycleNode):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS
```

**AMR Use:** Ensure sensors are calibrated (configured) before movement (active).

### Launch Files and CLI Commands
Launch files start multiple nodes atomically.

**Code Snippet (advanced_launch.py - Excerpt):**  
```python
return LaunchDescription([
    Node(package='advanced_nodes', executable='topic_publisher', name='topic_publisher'),
    # ... other nodes
])
```

**Key CLI Commands:**  
- Run: `ros2 run  `  
- Launch: `ros2 launch  `  
- Lifecycle: `ros2 lifecycle set /node configure`  
- Inspect: `ros2 topic echo /amr_topic`, `ros2 service list`, `ros2 action info /amr_action`  

***

## 7. Deeper Visuals and Flows

These ASCII visuals provide deeper details on transfers, including DDS internals (serialization, RTPS packets, QoS checks), error handling, and AMR-specific optimizations.

### 7.1 Topic Message Transfer (Enhanced Depth)

```
┌─────────────────────────────┐                  ┌─────────────────────────────┐
│ Publisher Node              │                  │ Subscriber Node             │
│ (topic_publisher.py)        │                  │ (topic_subscriber.py)       │
│                             │                  │                             │
│ 1. App Logic: Create Msg    │                  │                             │
│    (e.g., String data)      │                  │                             │
│ 2. rclpy: Serialize Msg     │                  │                             │
│    + QoS Metadata           │                  │                             │
│    (RELIABLE, TL, depth=10) │                  │                             │
│ 3. RMW: Pass to DDS         │                  │                             │
└──────────────┬──────────────┘                  └──────────────┬──────────────┘
               │ DDS DataWriter                                 │ DDS DataReader
               │                                                │
               │ 4. Discovery Phase:                            │
               │    - Broadcast topic/QoS via RTPS             │
               │    - Wait for matching subscribers             │
               │ 5. QoS Validation:                             │
               │    - Check Reliability (RELIABLE must match)  │
               │    - Check Durability (TL: store for late subs)│
               │    - Check History (KEEP_LAST: queue depth=10) │
               │    - If mismatch: No connection! (e.g., REL vs. │
               │      BEST_EFFORT fails for critical AMR data)   │
               │                                                │
               ▼                                                │
     ┌─────────────────────────────────────────────────────────────┐
     │ DDS Middleware (e.g., Cyclone DDS / Fast DDS)              │
     │ - Internal: Create RTPS Packet (header + serialized data)  │
     │ - Transport: Send over UDP (for speed) or TCP (reliable)    │
     │ - Reliability Handling: Send ACKs/NAKs, Retry lost packets   │
     │ - Durability: Buffer last N msgs for new/late subscribers    │
     │ - Error Case: If deadline missed, trigger QoS event         │
     │ - AMR Opt: For sensors, switch to BEST_EFFORT + low depth   │
     └──────────────────────────────┬──────────────────────────────┘
                                    │ 6. Receive RTPS Packet
                                    │ 7. Deserialize Msg + Metadata
                                    │ 8. Validate QoS (local check)
                                    │ 9. rclpy: Invoke Callback
                                    │ 10. App Logic: Process Data
                                    │    (e.g., update AMR state)
                                    └──────────────────────────────────┘
```

**Depth Notes:** DDS uses RTPS for real-time pub/sub. QoS ensures no data loss in reliable mode (AMR commands) but allows drops for performance (sensors). Visualization shows full stack from app to network.

### 7.2 Service Request-Response Flow (Enhanced Depth)

```
┌─────────────────────────────┐                  ┌─────────────────────────────┐
│ Client Node                 │                  │ Server Node                 │
│ (service_client.py)         │                  │ (service_server.py)         │
│                             │                  │                             │
│ 1. App: Build Request       │                  │                             │
│    (e.g., Trigger)          │                  │                             │
│ 2. rclpy: Serialize Req     │                  │                             │
│    + QoS (implicit RELIABLE)│                  │                             │
└──────────────┬──────────────┘                  └──────────────┬──────────────┘
               │ DDS (Request Channel)                          │ DDS (Response Channel)
               │                                                │
               │ 3. Discovery: Find service endpoint via RTPS   │
               │ 4. QoS Check: Ensure RELIABLE for sync calls   │
               │                                                │
               ▼                                                │
     ┌─────────────────────────────────────────────────────────────┐
     │ DDS Middleware                                             │
     │ - Serialize Req into RTPS Packet (with seq num for ordering)│
     │ - Transport: Send Req over UDP/TCP (RELIABLE: wait for ACK) │
     │ - Timeout Handling: If no response, retry or error          │
     │ - AMR Opt: Use deadlines for time-critical queries (e.g.,   │
     │   battery check must respond in <1s)                       │
     └──────────────────────────────┬──────────────────────────────┘
                                    │ 5. Deserialize Req
                                    │ 6. App: Process (e.g., trigger action)
                                    │ 7. Build Response (success/message)
                                    │ 8. Serialize Response + QoS
                                    │ 9. Send back via RTPS (with ACKs)
                                    │
                                    ▼
               │ 10. Deserialize Response                      │
               │ 11. App: Handle (e.g., log success)            │
               └────────────────────────────────────────────────┘
```

**Depth Notes:** Services map to two DDS topics (req/res). Visualization includes sequencing, timeouts, and AMR optimizations like deadlines for safety-critical calls.

### 7.3 Action Goal-Feedback-Result Flow (Enhanced Depth)

```
┌─────────────────────────────┐                  ┌─────────────────────────────┐
│ Client Node                 │                  │ Server Node                 │
│ (action_client.py)          │                  │ (action_server.py)          │
│                             │                  │                             │
│ 1. App: Set Goal (e.g.,     │                  │                             │
│    Fibonacci order=5)       │                  │                             │
│ 2. Serialize Goal + QoS     │                  │                             │
└──────────────┬──────────────┘                  └──────────────┬──────────────┘
               │ DDS Goal Channel                               │ DDS Goal Channel
               │                                                │
               │ 3. Discovery & QoS Match (RELIABLE required)   │
               │                                                │
               ▼                                                │
     ┌─────────────────────────────────────────────────────────────┐
     │ DDS Middleware                                             │
     │ - Send Goal RTPS Packet (with UUID for tracking)           │
     │ - QoS: RELIABLE + LIVELINESS (detect server alive)         │
     └──────────────────────────────┬──────────────────────────────┘
                                    │ 4. Deserialize Goal
                                    │ 5. Accept & Start Processing
                                    │    (e.g., compute sequence)
                                    │
                                    │ DDS Feedback Channel
                                    │ 6. Serialize Feedback (progress)
                                    │ 7. Send Periodic Feedback RTPS
                                    │    (QoS: BEST_EFFORT for speed)
                                    │
                                    │ DDS Result Channel
                                    │ 8. On Complete: Serialize Result
                                    │ 9. Send Result RTPS (RELIABLE)
                                    │
                                    ▼
               │ 10. Receive/Deserialize Feedback (log progress) │
               │ 11. Receive/Deserialize Result (handle final)   │
               │ - Preempt Option: Client can cancel via DDS     │
               └────────────────────────────────────────────────┘
```

**Depth Notes:** Actions use 5 internal topics (goal, cancel, status, feedback, result). Visualization adds UUID tracking, liveliness, and preempt paths for AMR tasks like interruptible movement.

### 7.4 Lifecycle Transitions (Enhanced Depth)

```
                          ┌──────────────────┐
                          │   Unconfigured   │  ← Initial State (Node Created)
                          │ (Resources Free) │
                          └────────┬─────────┘
                                   │ configure command (ros2 lifecycle set)
                                   │ Callback: on_configure() - Load params, init pubs/subs
                                   │ Success: Allocate resources
                                   │ Failure: Stay or error to Cleanup
                                   ▼
                          ┌──────────────────┐
                          │   Configuring    │  ← Transition State (Temporary)
                          │ (Allocating)      │
                          └────────┬─────────┘
                                   │ SUCCESS
                                   ▼
                          ┌──────────────────┐
                          │     Inactive     │  ← Ready but not publishing
                          │ (Configured)      │
                          └────────┬─────────┘
                                   │ activate command
                                   │ Callback: on_activate() - Start timers/pubs
                                   │ Success: Begin operations
                                   │ Failure: Deactivate or error
                                   ▼
                          ┌──────────────────┐
                          │    Activating    │  ← Transition (Starting pubs/subs)
                          │ (Enabling)        │
                          └────────┬─────────┘
                                   │ SUCCESS
                                   ▼
                          ┌──────────────────┐
                          │      Active      │  ← Fully Running (Publishing/Processing)
                          │ (Operational)     │  - AMR Ex: Sensors streaming, motors enabled
                          └────────┬─────────┘
                                   │ deactivate command
                                   │ Callback: on_deactivate() - Pause timers/pubs
                                   │ Success: Safe stop
                                   │ Failure: Error to Cleanup
                                   ▼
                          ┌──────────────────┐
                          │   Deactivating   │  ← Transition (Stopping)
                          │ (Disabling)       │
                          └────────┬─────────┘
                                   │ SUCCESS
                                   ▼
                          ┌──────────────────┐     ┌────────────┐
                          │     Inactive     │  ─► │  Cleanup   │  ← From errors or cleanup cmd
                          └────────┬─────────┘     │ (Releasing)│     Callback: on_cleanup() - Free resources
                                   │                └────┬───────┘
                                   │ cleanup cmd           │ SUCCESS
                                   │ Callback: on_cleanup()│
                                   ▼                       ▼
                          ┌──────────────────┐     ┌────────────┐
                          │   Cleaning Up    │  ─► │Unconfigured│  ← Loop back for reconfiguration
                          │ (Releasing)       │     └────────────┘
                          └────────┬─────────┘
                                   │ SUCCESS
                                   ▼
                          ┌──────────────────┐
                          │   Unconfigured   │
                          └────────┬─────────┘
                                   │ shutdown cmd (from any state)
                                   │ Callback: on_shutdown() - Final cleanup
                                   │ Success: Graceful exit
                                   │ Failure: Force to Finalized
                                   ▼
                          ┌──────────────────┐     ┌────────────┐
                          │  Shutting Down   │  ─► │ Finalized  │  ← Terminal State (Node Destroyed)
                          │ (Finalizing)      │     │ (Destroyed)│     Resources fully released
                          └────────┬─────────┘     └────────────┘
                                   │ SUCCESS
                                   ▼
                          ┌──────────────────┐
                          │    Finalized     │
                          └──────────────────┘
```

**Depth Notes:** Added callbacks, failure paths, and AMR examples (e.g., active state for motor control). Errors always lead to safe states.

### 7.5 DDS/QoS Matching and Optimization (New Visuals)

**QoS Matching Matrix (Compatibility Check):**

```
Publisher QoS     | Subscriber QoS     | Match? | AMR Scenario
------------------|--------------------|--------|-------------
RELIABLE          | RELIABLE           | Yes    | Critical commands (no loss)
RELIABLE          | BEST_EFFORT        | No     | Mismatch: Sub wants speed, pub guarantees – fails
BEST_EFFORT       | BEST_EFFORT        | Yes    | High-rate sensors (allow drops)
TRANSIENT_LOCAL   | VOLATILE           | Yes    | Late subs get last msg (pub offers more)
VOLATILE          | TRANSIENT_LOCAL    | No     | Mismatch: Sub demands persistence, pub doesn't
KEEP_LAST(10)     | KEEP_LAST(5)       | Yes    | Compatible (sub requests less history)
KEEP_ALL          | KEEP_LAST(10)      | No     | Mismatch: Sub limits, pub wants all
```

**Optimization Flow for AMR (New Visual):**

```
AMR Data Type ─► QoS Profile ─► DDS Behavior ─► Benefit
Sensor (LiDAR) ─ BEST_EFFORT + VOLATILE + depth=1 ─ Low-latency UDP, drop stale ─ Real-time obstacle detection
Command (Move) ─ RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(10) ─ TCP-like ACKs, store last cmds ─ Guaranteed delivery, late nodes catch up
Status (Battery) ─ RELIABLE + VOLATILE + deadline=1s ─ Enforce timing, error if missed ─ Safety monitoring with alerts
```

**Depth Notes:** New matrix shows why mismatches fail. Optimization visual ties to AMR use cases, emphasizing trade-offs (speed vs. reliability).

***

## 8. Debugging and Monitoring

- Graph: `rqt_graph` (visualize nodes/topics).  
- Doctor: `ros2 doctor --report` (check setup).  
- Verbose Info: `ros2 topic info /amr_topic --verbose` (QoS details).  
- Rate: `ros2 topic hz /amr_topic` (publish frequency).  
- Lifecycle State: `ros2 lifecycle get /lifecycle_amr_node`.  

***

## 9. Key Takeaways

- DDS enables distributed, QoS-tuned comms without a master.  
- Choose primitives wisely: Topics for streams, services for queries, actions for tasks.  
- Lifecycle adds safety for AMR operations.  
- Visuals highlight full flows – use for debugging mismatches.

***