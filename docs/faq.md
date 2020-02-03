# Frequently Asked Questions

#### What is the difference between RoMi-H and rmf_core, in high level?

RoMi-H is an umbrella term for a wide range of open specifications and software
tools that aim to ease the integration and interoperability of robotic systems,
building infrastructure, smart medical devices, and user interfaces with a focus
on the healthcare sector.

rmf_core is a repository for an implementation of some of the core systems that
will compose RoMi-H.


#### Why is RoMi-H spread across multiple GitHub accounts and repos?

The development of RoMi-H is a collaborative research and development effort.
Several different organizations are involved in its development, and there is
not yet a fixed protocol for where and how the constituent parts of RoMi-H will
be deposited. The organization of these packages may converge as the project
continues to progress.


#### In the actual deployment, is RoMi-H something that is deployed into a server or a robot?

RoMi-H is a collection of open specifications and software tools. Some of the
specifications and software that falls under the RoMi-H umbrella might run on
robots, but for the most part it will be used as an intermediary to communicate
and negotiate between standalone systems. Since robots are usually deployed with
their own proprietary fleet managers, in most cases we expect RoMi-H to
communicate with a fleet manager instead of running directly on a robot. However
users will be provided with RoMi-H software tools that can run directly on a
robot to assist in cases where a robot platform does not come with its own fleet
manager.

In short, some parts of RoMi-H may run on servers, some on desktops, some on
mobile devices, and some on robot platforms.


#### Does RoMi-H support High Availability (e.g failover cluster)? Can it be load-balanced?

The systems within RoMi-H are heterogeneous. Some systems are fully distributed,
so there is no master that would be a failure point. Other systems do require
certain centralized services, and those services are being designed to have
failovers as well as ways to distribute their load, e.g. by using mirror
servers.


#### Does RoMi-H run on ROS nodes?

Certain components in RoMi-H are being implemented using ROS2. For non-ROS2
systems, we are working on various options for bridging between different
middlewares. Much of that effort is concentrated in the
[SOSS](https://github.com/osrf/soss) project.


#### What is SOSS?

Please refer to the [SOSS GitHub page](https://github.com/osrf/soss). SOSS is a
plugin-based framework for performing simple translations between pub/sub
middlewares. Since we expect RoMi-H to bridge many systems that are already
running their own middlewares, we are providing SOSS as one option for
integrating a middleware into RoMi-H.


#### Are there design guidelines for integrating with RoMi-H?

RoMi-H is still in research and development, moving towards production
deployment, and many of the APIs and specifications are under active
development. Design and integration guidelines will be coming out as the core
APIs solidify.


#### If I want a CI/CD pipeline to build my custom RoMi-H components, is there already a template or docker image to help with this?

There is an ongoing effort to provide this, but it is not ready for public consumption yet.


#### Is RoMi-H production-ready?

RoMi-H is still in research and development, but we are aggressively moving
towards deployability. We aim to have the APIs stable and key features
implemented by mid-2020.


#### Is there python version of RoMi-H?

We intend to provide Python bindings for the core APIs of RoMi-H, especially for
robot fleet management. This work has not yet begun, but it should be
straightforward once the C++ APIs have stabilized.


#### Is RoMi-H constrained to a particular DDS?

Just like how ROS2 is not constrained to any particular DDS, neither is RoMi-H.
The choice of which DDS implementation to use will be determined by the system
integrators who deploy a RoMi-H system in a given facility.


#### How do we specify the map layouts of a building, and tie together multiple floors for that building?

Our tool for managing map layouts is available at
https://github.com/osrf/traffic_editor. Specifying multiple floors for a
building is a feature that should be finished in the very near future.


#### How can we specify the schedule of a fleet?

The API for specifying robot traffic schedules is undergoing enormous changes
right now. A preliminary version of it already exists, but I do not recommend
familiarizing yourself with it, because it will be completely different very
soon.


#### Which distribution of ROS2 is compatible with RoMi-H for production purpose?

Currently rmf_core requires ROS2 eloquent for certain launch file features. In
general, we are likely to be using the latest release of ROS2 while doing
research and development on RoMi-H.
