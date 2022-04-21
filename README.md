# ah_ros_barrier
ROS node for receiving lap time data from the optic barrier using Arrowhead Framework.
_Note: This is built on [ah-ros-bridge-example](https://github.com/jara001/ah-ros-bridge-example)._


This application serves as an Arrowhead wrapper for the **Optic barrier** of [f1tenth-scoreapp](https://github.com/CTU-IIG/f1tenth-scoreapp) race measuring system. It register itself as an Arrowhead Service `laptime` consumer and uses Arrowhead Core orchestration to find available barriers (providers).

The Service `laptime` has following metadata:
- `address` -- preferred IP address of the barrier

_Note: The node currently connects to the first received barrier only using preferred address (if available)._

Data received from the optic barrier (lap time) are published to topic `lap_time` using [std_msgs/Time](http://docs.ros.org/en/kinetic/api/std_msgs/html/msg/Time.html) message.


## Requirements
- `autopsy >= 0.4.1`
  - GitHub: [https://github.com/jara001/autopsy](https://github.com/jara001/autopsy)
  - Wheel: [v0.4.1](https://github.com/jara001/autopsy/releases/download/v0.4.1/autopsy-0.4.1-py2.py3-none-any.whl)
- `aclpy >= 0.2.0`
  - GitHub: [https://github.com/CTU-IIG/ah-acl-py](https://github.com/CTU-IIG/ah-acl-py)
  - Wheel: [v0.2.0](https://github.com/jara001/ah-acl-py/releases/download/v0.2.0/aclpy-0.2.0-py3-none-any.whl)
- `websocket-client`
  - `python3 -m pip install websocket-client`


## Starting up

1. Obtain certificates (`.p12`, `.pub`) for your system from your local Arrowhead Core.
2. Obtain also the certificate authority `.ca` for your cloud.
3. Create a configuration file `configuration.py`.
4. Run the ROS node.


### Example configuration
_Note: This is currently stored inside `./ah_ros_barrier/module/configuration.py`._

```python
Server = ArrowheadServer(
    address = "127.0.0.1",
)

Interface = ArrowheadInterface(
    name = "HTTP-INSECURE-JSON",
)

Service = ArrowheadService(
    name = "laptime",
)

Client = ArrowheadClient(
    name = "ros-machine",
    address = "192.168.1.1",
    port = 0,
    pubfile = "/home/machine/keys/ros-machine.pub",
    p12file = "/home/machine/keys/ros-machine.p12",
    p12pass = "abcd",
    cafile = "/home/machine/keys/cloud.ca",
    server = Server,
    interfaces = [Interface],
)
```
