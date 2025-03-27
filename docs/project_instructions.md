# Project Overview

We have a mock project called **med_mon**, which is a medical monitoring dashboard in the very early stages of development. It is built on ROS (Robot Operating System) and QT. Currently, we are injecting simulated signals into the dashboard via simulation nodes located in the `sensors/sensor_sim` directory.

## Task Description

The EKG graph is currently too noisy. We would like to implement filtering of the EKG signal via an additional ROS node. The filtering should be modular, as we may want to apply similar filtering to other signals in the future. At a minimum, the user should be able to enable or disable the filtering at runtime.

## Setup

### Linux (Ubuntu):

1. Install docker following the instructions here: [Docker Installation](https://docs.docker.com/engine/install/ubuntu/)
   - Note: Recommend installing via the steps ‘Install using the apt repository’.
2. Copy the **med_mon** source code to a local directory.
3. Follow the instructions for launching the **med_mon** project located in `med_mon/README.md`.

### Windows / Mac:

1. If you are using Windows or Mac, you will need to download and configure a VM with a tool such as UTM (Mac) or VirtualBox (Windows).
2. Follow the instructions for launching the **med_mon** project located in `med_mon/README.md`.

## Project Deliverables:

1. **Modular Filtering Node:** Implement signal filtering on the EKG signal. The filter should be implemented as an additional ROS node. You may choose any filter design you find suitable.
2. **Runtime Filter Control:** Implement a mechanism to control the filter during runtime via the existing GUI window, or a new window (at a minimum, the ability to enable or disable the filter and see it reflected on the EKG graph).
3. **Documentation:** Provide simple documentation that describes the filter implementation. Include instructions for use, information about the filter design, and information about the data flow within the ROS network as it relates to the filter node.
4. **General Improvements:** While working on the **med_mon** project, feel free to implement any general improvements you deem necessary, or document the need for such improvements in the future.
