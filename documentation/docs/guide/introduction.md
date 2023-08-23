# Introduction to the OSAVC

The Open Source Real-time Controller for Resource-constrained Autonomous Vehicles and Systems (OSAVC) is a project aimed at designing and implementing an open-source real-time hardware controller specifically tailored for resource-constrained autonomous vehicles and systems.

## Motivation

The field of autonomous systems is rapidly growing across various domains, from scientific and industrial to military applications. As technology advances, smaller yet more powerful microprocessors and microcontrollers are becoming available. OSAVC addresses the need for a dedicated, modular real-time controller that can be integrated into distributed control architectures for autonomous vehicles and systems.

## Problem Statement

The lack of a vehicle-agnostic, real-time controller for autonomous vehicles and systems is a significant challenge. OSAVC aims to address this by providing a flexible, open-source solution that supports real-time computation, adaptability to different vehicle configurations, and accessibility to a broad community of users and contributors.

## Architecture Overview

OSAVC consists of several key components:

- **Real-time Autopilot:** Provides real-time control and computation capabilities, allowing integration with different vehicle types and algorithms.
- **Single Board Computer (SBC):** Enables non-real-time tasks and access to advanced computational packages, extending the system's capabilities beyond real-time functions.
- **Tensor Processing Unit (TPU):** Facilitates onboard machine learning capabilities, such as object detection and image classification, without overburdening the SBC.

## Features

- Modular and adaptable design for various vehicle types.
- Real-time computation for precise control and estimation algorithms.
- Integration with SBC for non-real-time tasks and advanced computation.
- Onboard machine learning capabilities for object detection and classification.
- Open-source hardware and firmware, promoting collaboration and innovation.

## Getting Started

For detailed instructions on setting up and using the OSAVC controller, please refer to the [Building your first projects section](FirstProject.md).

## Contributions

Contributions to the OSAVC project are welcome! If you're interested in contributing, please review the [Contribution Guidelines](../contribute/code/getting-started.md) for more information.
