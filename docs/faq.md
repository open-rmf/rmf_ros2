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

#### How does `rmf_traffic` avoid mobile robot traffic conflicts?

When we are done implementing the traffic management solution, we will be doing a more
extensive write-up on the conflict avoidance and negotiation methods than what can reasonably
fit in an FAQ, but here is a quick outline of the methodology. There are two levels to
traffic deconfliction: (1) prevention, and (2) resolution.

1. **Prevention.** Whenever possible, it would be good to prevent traffic conflicts from
happening in the first place. To facilitate this, we have implemented a platform agnostic
[Traffic Schedule Database](https://github.com/osrf/rmf_core/blob/8cad142e5a5f14133e4e865beeac98fd46edb0e7/rmf_traffic/include/rmf_traffic/schedule/Database.hpp). The traffic schedule is a living
database whose contents will change over time to reflect delays, cancelations, or route changes.
All fleet managers that are integrated into RoMi-H must report the expected itineraries of their
vehicles to the traffic schedule. With the information available on the schedule, compliant fleet
managers can plan routes for their vehicles that avoid conflicts with any other vehicles (no matter
which fleet they belong to). `rmf_traffic` provides a [Planner](https://github.com/osrf/rmf_core/blob/8cad142e5a5f14133e4e865beeac98fd46edb0e7/rmf_traffic/include/rmf_traffic/agv/Planner.hpp) class to
help facilitate this for vehicles that behave like standard AGVs. In the future we intend to provide
a similar utility for AMRs.

2. **Resolution.** It is not always possible to perfectly prevent traffic conflicts. Mobile robots
may experience delays because of unanticipated obstacles in their environment, or the predicted
schedule may be flawed for any number of reasons. In cases where a conflict does arise, `rmf_traffic`
has a [Negotiation](https://github.com/osrf/rmf_core/blob/8cad142e5a5f14133e4e865beeac98fd46edb0e7/rmf_traffic/include/rmf_traffic/schedule/Negotiation.hpp) scheme. When the Traffic Schedule Database detects an
upcoming conflict between two or more schedule participants, it will send a conflict notice out to
the relevant fleet managers, and a negotiation between the fleet managers will begin. Each fleet manager
will submit its preferred itineraries, and each will respond with itineraries that can accommodate the
others. A third-party judge (deployed by the system integrator) will choose the set of proposals that
is considered preferable and notify the fleet managers about which itineraries they should follow.

#### Why is this traffic management system so complicated?

RoMi-H has a number of system design constraints that create unique challenges for traffic management.
The core goal of RoMi-H is to facilitate system integration for heterogeneous mobile robot fleets that
may be provided by different vendors and may have different technical capabilities.

1. Vendors tend to want to keep their computing systems independent from other vendors. Since vendors
are often responsible for ensuring uptime and reliability on their computing infrastructure, they may
view it as an unacceptable liability to share computing resources with another vendor. This means that
the traffic management system must be able to function while being distributed across different machines
on a network.

2. Different robot platforms may have different capabilities. Many valuable AGV platforms that are
currently deployed are not able to change their itineraries dynamically. Some AGV platforms can
change course when instructed to, as long as they stick to a predefined navigation graph. Some AMR
platforms can dynamically navigate themselves around unanticipated obstacles in their environment.
Since RoMi-H is meant to be an enabling technology, it is important that we design a system that can
maximize the utility of all these different types of systems without placing detrimental constraints
on any of them.

These considerations led to the current design of distributed conflict prevention and distributed
schedule negotiation. There is plenty of space within the design to create simpler and more efficient
subsets for categories of mobile robots that fit certain sets of requirements, but these optimizations
can be added later, building on top of the existing completely generalized framework.
