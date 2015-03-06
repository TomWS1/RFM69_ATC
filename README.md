# RFM69_ATC
Example derived class for the lowpowerlabs RFM69 library update enabling derived classes.  This is preliminary, but has passed all early testing.

This derived class extends the RFM69 library by providing a mechanism for automatic transmit power control.  On startup you tell the RFM69_ATC object a target RSSI value (your signal strength at the receiving node).  I use -59dB as a target because it provides a good reliable connection, but generally allows an RFM69HW to dial back the transmit power significantly.  Usually to its minimum if the two nodes are within the same building.

The library also provides a better linear control on power settings for the RFM69HW.

Originally I was going to propose incorporating this into the RFM69 library, but current discussions have led to implementing this as a derived class instead.
