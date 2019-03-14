FIROS
=====

FIROS is a tool that helps connecting robots to the cloud. For this purpose it uses the [Robot Operating System (ROS)]( http://www.ros.org/) and the [FIWARE  Context Broker](http://catalogue.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker) as a way to publish and listen robot's data.

FIROS works as a translator between the robotics field and the cloud world, transforming ROS messages into NGSI to publish them in the cloud, and vice versa.

This project is a fork from the **outdated** [Ikergunes FIROS package](https://github.com/Ikergune/firos). We have made a lot of changes - basically nothing is at it was.

| :books: [Documentation](https://github.com/iml130/firos/blob/master/doc/index.md) | :dart: [Roadmap](https://github.com/iml130/firos/blob/master/doc/roadmap.md) |
| -------------------------------------------------------- |  -------------------------------------------------------------------------------------------- |

FIWARE Integration
================
The following figure depicts the integration of FIROS inside the FIWARE platform.

![alt text](https://github.com/iml130/firos/blob/master/doc/firos.png "FIROS Integration")

FIROS converts ROS topics into NGSI v2 Entities and vice versa.

Roadmap/ Changes
================
We have some ideas what we would like to change in FIROS. There are already some ideas implemented. The following table shows the current features we are working on and what is actually planned:

| # | Feature            | Description                                                                                       | Status            |
|---|--------------------|---------------------------------------------------------------------------------------------------|-------------------|
| 1 | FiwareObjectConverter | Our own, more efficient and easier to use Object<>Fiware -Converter. There are some small flaws in converting objects, which is related to the NGSv1-API. | Beta |
| 2 | Nested Custom ROS Msg | It is not possible to convert a nested ROS-Msg, with an array of an own object -> More details: [Issue 1](https://github.com/iml130/firos/issues/1 ) | Beta (Done) |
| 3 | NGSI v1->v2     | NGSIv1 is used in FIROS, we are changing the NGSIv1 (Orion support is depricated) to NGSIv2 | Done |
| 4 | Incremental Update | FIROS is also updating the whole entity. We want to update only the required attributes to have a more efficient communication (NGSIv2) | Done |
| 5 | Unit Tests | The whole code is currently untested. We will create Tests and checks to guarantee expected behaviour | Development    |
| 6 | Dockerize it | A Docker will be composed on some Point for an easier integration. | Planned |
| 7 | Continoues Integration (CI) | It is easier to run an automated pipeline for Unit-Testing, Deployment, etc.. FIROS will also have badges as other projects! | Planned |
| 8 | Documentation | Currently only this Readme.md is available. It is not practical to just leave it as is. We will add more howTo's and information about FIROS soon! | Development |


Status:
- **Development**: We are working on it, it will take some time.
- **Beta**: Ready for testing. 
- **Done**: We have finished this feature. Next!
- **Planned**: Give us some time, we will come to this feature.

If you have any feature requests, comments, ideas - feel free to contact us :)

License
=======

FIROS is licensed under [MIT License](https://opensource.org/licenses/MIT).

Contributors
=======

Dominik Lux  
Peter Detzner

Presentations
=======
FIROS-Helping Robots to be Context Aware ([Slideshare](https://de.slideshare.net/FI-WARE/fiware-global-summit-FIROS-helping-robots-to-be-context-aware), 28.11.2018 Fiware Global Summit, Malaga)
