# rmf\_core
The `rmf_core` packages provide the centralized functions of
the Robotics Middleware Framework (RMF). These include task
queuing, conflict-free resource scheduling, utilities to help
create robot fleet adapters, and so on.

All packages in this repo will be written in ROS 2.

To create a useful deployment, `rmf_core` must be connected
to many other subsystems, as shown in the following diagram.

![integration-diagram](/docs/rmf_core_integration_diagram.png)
