# Upgrading from [Ikergune/firos](https://github.com/Ikergune/firos)

In case you have an old version of FIROS. You need to apply the following procedure, so that you can use the newer
version:

## Clearing Context-Broker's Content

First you need to make sure that every FIROS-instance has stopped. Check the contents of the
[Orion Context Broker](https://fiware-orion.readthedocs.io/en/master/) whether there still is some old content of the
old FIROS.

You can check it e.g via:

> CONTEXT_BROKER_ADDRESS:PORT/v2/entities?type=ROBOT

If it returns content use the NGSI v2 API to remove it.

(You can also remove the robot-descriptions-entity, since the new FIROS does not depend on it anymore.)

There might also exist subscriptions created by the old FIROS. You can see that there are referencing to the following
URL `FIROS_ADRESS/firos` in:

> CONTEXT_BROKER_ADDRESS:PORT/v2/subscriptions

Again: Delete them using the NGSI v2 API.

## Migrating the Configuration-Files

Most of the configuration files do not need to be touched. You only need to change some values inside the `config.json`
as follows:

| Attribute             | Old Value                                           | New Value                            |
| --------------------- | --------------------------------------------------- | ------------------------------------ |
| "throttling"          | Values like `PT0S` can be directly set in seconds   | Corresponds to `0`                   |
| "subscription_length" | Values like `P1D` need to be converted into seconds | Corresponds to `24*60*60 ->` `86400` |

We also changed them to different default values in this
[commit](https://github.com/iml130/firos/commit/88b4b201c4a948b5d8d58e4af2a5a46146222d2d) which also can be adopted.
