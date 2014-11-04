RSD2014MarkerLocator
====================

The marker locator ros node for passing the marker positions from tcp to tranmerc

The rosnode publishes a gpgga_tranmerc message with the name "/fmInformation/gpgga_tranmerc". This is the same used by the kalmann filter.

The node can be started as:

rosrun socket_tranmerc tcp_client.py

or one can include the order of marker as a parameter:

rosrun socket_tranmerc tcp_client.py _order:=7

The node can also be included in a launch file as seen in launch/example.launch
