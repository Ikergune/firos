# Deprecated Features

## REST-API `whitelist` on FIROS

FIROS currently has a REST-API, where someone can manipulate the whitelist of an FIROS-instance (see:
[API](user/api.md)). We are currently not planning to expand this functionality, because the `whitelist.json` is
definitely known prior and is overwritten by the configuration in `robots.json`.

Specifically, the following methods are provided as is and are not maintained:

-   `FIROS:/whitelist/write`
-   `FIROS:/whitelist/remove`
-   `FIROS:/whitelist/restore`
