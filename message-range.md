* 0x100 to 0x109 - Alerts and error messages; the node will not need to process these
* 0x110 to 0x1FF - Output commands; control physical output from the node
* 0x200 to 0x2FF - Display commands: These are "user interface" type outputs, such as indicator lights and lcd displays
* 0x320 - 0x3FF - Producer configuration commands. The range 0x300-0x31F is reserved and will be handled before the consumer by the router library.
* 0x400 - 0x4FF - Identity request and config commands.
* 0x500 - 0x5FF - Submodule data messages, the producer uses these to transmit data, the router might use these to ack on data coming in.
* 0x600 - 0x6FF - Reserved, not implemented
* 0x700 - 0x7FF - Introduction messages for submodules and parent nodes.
